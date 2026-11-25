"""Stage 08: physical hexagonal lens facets on two curved optical domes.
Local +X is the optical axis, +Z is dorsal; all design dimensions are mm.
"""
from pathlib import Path
import numpy as np
import trimesh
from step_01_chassis import Module, create_chassis
from step_02_power_core import attach_module
from step_07_head import build_stage_07

PITCH = .78
PARAM_RADIUS = 6.8


def facet_centers() -> np.ndarray:
    """Hexagonal lattice, keeping complete cells within the optical aperture."""
    points = []
    for r in range(-12, 13):
        for q in range(-12, 13):
            p = PITCH * np.array([q + r / 2, r * np.sqrt(3) / 2])
            if np.linalg.norm(p) + PITCH / np.sqrt(3) <= PARAM_RADIUS:
                points.append(p)
    return np.asarray(points)


def optical_surface(uv, offset=0.):
    """Map flat polar coordinates onto a half ellipsoid, then offset normally."""
    uv = np.asarray(uv, float)
    r = np.linalg.norm(uv, axis=-1)
    theta = r / PARAM_RADIUS * np.pi / 2
    direction = np.divide(uv, r[..., None], out=np.zeros_like(uv), where=r[..., None] > 0)
    x = 5.7 * np.cos(theta)
    yz = 6.8 * np.sin(theta)[..., None] * direction
    xyz = np.concatenate([x[..., None], yz], axis=-1)
    normal = xyz / np.array([5.7, 6.8, 6.8])**2
    normal /= np.linalg.norm(normal, axis=-1, keepdims=True)
    return xyz + np.array([.6, 0, 0]) + offset * normal


def closed_mesh(vertices, faces):
    mesh = trimesh.Trimesh(vertices=vertices, faces=faces, process=False)
    mesh.fix_normals()
    return mesh


def create_facet(center, radius=PITCH / np.sqrt(3)):
    """Return separate closed metallic bezel and convex hex lens meshes in mm."""
    a = np.arange(6) * np.pi / 3 + np.pi / 6
    offsets = radius * np.column_stack([np.cos(a), np.sin(a)])
    # Four loops: outer bottom/top and inner top/bottom. Connect cyclically.
    loops = [optical_surface(center + offsets * f, h) for f, h in
             [(0.98,-.06),(.98,.09),(.81,.09),(.81,-.06)]]
    faces = []
    for j in range(4):
        for i in range(6):
            k=(i+1)%6; a0=j*6+i; b0=j*6+k; a1=((j+1)%4)*6+i; b1=((j+1)%4)*6+k
            faces.extend([(a0,b0,b1),(a0,b1,a1)])
    bezel = closed_mesh(np.vstack(loops), faces)
    # Slight seating overlap with bezel; convex raised center on each lens.
    vertices = np.vstack([optical_surface(center + offsets*.83,-.045),
                          optical_surface(center + offsets*.83,.12),
                          optical_surface(center,.27), optical_surface(center,-.045)])
    faces=[]
    for i in range(6):
        k=(i+1)%6
        faces.extend([(i,k,k+6),(i,k+6,i+6),(6+i,6+k,12),(i,k,13)])
    return bezel, closed_mesh(vertices,faces)


def backing_dome():
    """Closed dark half-ellipsoid underneath the individual optical cells."""
    n=96; rows=24; vertices=[optical_surface([0,0],-.12)]
    angles=np.arange(n)*2*np.pi/n
    for i in range(1,rows+1):
        uv=PARAM_RADIUS*i/rows*np.column_stack([np.cos(angles),np.sin(angles)])
        vertices.extend(optical_surface(uv,-.12))
    vertices.append([.6,0,0]); bottom=len(vertices)-1; faces=[]
    for i in range(n):
        k=(i+1)%n; faces.append((0,1+i,1+k))
        for j in range(rows-1):
            a=1+j*n+i; b=1+j*n+k; c=a+n; d=b+n
            faces.extend([(a,c,d),(a,d,b)])
        faces.append((1+(rows-1)*n+i,bottom,1+(rows-1)*n+k))
    return closed_mesh(vertices,faces)


def create_compound_eye(side: str, scale: float = 1.0) -> Module:
    """Generate one eye; its mount aligns to the head's outward +X frame."""
    if side not in ('left','right'):
        raise ValueError('side must be left or right')
    if not np.isfinite(scale) or scale <= 0:
        raise ValueError('scale must be positive and finite')
    mm=.001*scale; scene=trimesh.Scene()
    def mat(name,rgb,metal,rough,alpha=1.):
        return trimesh.visual.material.PBRMaterial(name=name,baseColorFactor=[*rgb,alpha],
            metallicFactor=metal,roughnessFactor=rough,alphaMode='BLEND' if alpha<1 else 'OPAQUE')
    ti=mat('Optical mount titanium',[.52,.56,.60],.94,.3)
    steel=mat('Optical screws steel',[.72,.76,.79],.95,.19)
    bezel_mat=mat('Dark titanium micro-bezels',[.16,.20,.23],.88,.3)
    dark=mat('Light absorbing optical substrate',[.022,.042,.047],.05,.48)
    pcb=mat('Imaging board',[.025,.15,.11],.12,.5)
    gold=mat('Gold sensor contacts',[.78,.56,.19],.9,.28)
    sensor=mat('Imaging sensor silicon',[.09,.17,.20],.55,.22)
    lenses=[mat('Tinted optical glass '+str(i),rgb,.12,.12,.90) for i,rgb in enumerate([
        [.065,.22,.27],[.08,.28,.31],[.09,.32,.34],[.11,.25,.32],[.16,.32,.35],[.12,.20,.27]])]
    def add(name,mesh,material):
        mesh.apply_scale(mm); mesh.visual=trimesh.visual.TextureVisuals(material=material)
        scene.add_geometry(mesh,node_name=name,geom_name=name)
    def tube(name,x0,x1,outer,material,inner=None,y=0,z=0,sections=48):
        mesh=(trimesh.creation.cylinder(radius=outer,height=x1-x0,sections=sections) if inner is None else
              trimesh.creation.annulus(r_min=inner,r_max=outer,height=x1-x0,sections=sections))
        mesh.apply_transform(trimesh.transformations.rotation_matrix(np.pi/2,[0,1,0]))
        mesh.apply_translation([(x0+x1)/2,y,z]); add(name,mesh,material)
    def box(name,size,center,material):
        m=trimesh.creation.box(extents=size); m.apply_translation(center); add(name,m,material)
    # Mount footprint matches the head's 6.15–7.10 mm socket annulus.
    tube('mount_flange',0,.6,7.05,ti,6.20,sections=96)
    tube('retaining_ring',.6,.88,6.94,bezel_mat,6.63,sections=96)
    tube('sensor_board',.12,.32,5.95,pcb,sections=64)
    box('imaging_sensor',[.20,3.1,3.1],[.43,0,0],sensor)
    for i in range(12):
        a=2*np.pi*i/12
        tube(f'flange_screw_{i:02}',.60,.78,.22,steel,y=6.65*np.cos(a),z=6.65*np.sin(a),sections=16)
        box(f'sensor_contact_{i:02}',[.08,.20,.42],[.08,5.3*np.cos(a),5.3*np.sin(a)],gold)
    add('optical_substrate',backing_dome(),dark)
    for i,p in enumerate(facet_centers()):
        bezel,lens=create_facet(p)
        # Subtle color variation is deterministic and symmetric between eyes.
        color=int((np.linalg.norm(p)*1.7 + np.cos(np.arctan2(p[1],p[0])*2)*1.3 + 12)) % len(lenses)
        add(f'facet_{i:03}_bezel',bezel,bezel_mat)
        add(f'facet_{i:03}_lens',lens,lenses[color])
    anchors={'mount':np.eye(4),'optical_axis':np.eye(4),'data':np.eye(4)}
    anchors['optical_axis'][:3,3]=[6.57*mm,0,0]
    anchors['data'][:3,3]=[.04*mm,0,-4.9*mm]
    return Module(f'eye_{side}',scene,anchors)


def build_stage_08(scale: float = 1.0) -> tuple[dict[str, Module], trimesh.Scene]:
    """Attach both eyes with the existing head anchors in world coordinates."""
    head,prior=build_stage_07(scale)
    neck=create_chassis(scale).anchors['neck']
    anchors={side:neck @ head.anchors[f'eye_{side}'] for side in ('left','right')}
    eyes={side:create_compound_eye(side,scale) for side in anchors}
    scene=prior
    for side,eye in eyes.items():
        scene=attach_module(Module('stage_08',scene,anchors),side,eye)
    return eyes,scene


if __name__=='__main__':
    out=Path(__file__).resolve().parents[1]/'models'; out.mkdir(exist_ok=True)
    eyes,scene=build_stage_08()
    eyes['left'].scene.export(str(out/'step_08_eye.glb'))
    scene.export(str(out/'step_08_assembled.glb'))
    print('Facets per eye:',len(facet_centers()))
    print('Meshes per eye:',len(eyes['left'].scene.geometry))
    print('Assembly meshes:',len(scene.geometry))
    print('Assembly dimensions (mm):',scene.extents*1000)
