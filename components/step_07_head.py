"""Stage 07: open cranial skeleton and neutral-pose two-axis neck.
Local +X forward, +Y left, +Z dorsal. Geometry is authored in mm.
Eye mounts use local +X as the outward optical axis.
"""
from pathlib import Path
import numpy as np
import trimesh
from step_01_chassis import Module, create_chassis
from step_02_power_core import attach_module
from step_06_wing_joints import build_stage_06


def create_head(scale: float = 1.0) -> Module:
    """Return the head skeleton, neck, and future sensor mounting frames."""
    if not np.isfinite(scale) or scale <= 0:
        raise ValueError('scale must be positive and finite')
    mm = .001 * scale
    scene = trimesh.Scene()

    def material(name, rgb, metal, roughness):
        return trimesh.visual.material.PBRMaterial(name=name, baseColorFactor=[*rgb,1], metallicFactor=metal, roughnessFactor=roughness)
    ti = material('Cranial titanium', [.58,.61,.64], .94,.32)
    steel = material('Neck polished steel', [.75,.78,.81], .97,.19)
    bronze = material('Neck bearing bronze', [.65,.40,.17], .85,.31)
    dark = material('Cranial carbon composite', [.055,.065,.075], .05,.46)
    copper = material('Neck servo copper', [.72,.36,.16], .9,.28)
    cyan = material('Neck encoder marker', [.08,.60,.72], .1,.28)

    def add(name, mesh, mat):
        # Construct at millimeter magnitude to avoid tiny-mesh tolerances.
        mesh.apply_scale(mm)
        mesh.visual = trimesh.visual.TextureVisuals(material=mat)
        scene.add_geometry(mesh, node_name=name, geom_name=name)

    def box(name, size, center, mat):
        mesh = trimesh.creation.box(extents=size)
        mesh.apply_translation(center)
        add(name,mesh,mat)

    def tube(name, a, b, radius, mat, inner=None, sections=32):
        a,b = np.asarray(a,float),np.asarray(b,float)
        delta=b-a; length=np.linalg.norm(delta)
        if inner is None:
            mesh=trimesh.creation.cylinder(radius=radius,height=length,sections=sections)
        else:
            mesh=trimesh.creation.annulus(r_min=inner,r_max=radius,height=length,sections=sections)
        mesh.apply_transform(trimesh.geometry.align_vectors([0,0,1],delta))
        mesh.apply_translation((a+b)/2)
        add(name,mesh,mat)

    def ring_x(name, x, outer, inner, flatten, mat):
        mesh=trimesh.creation.annulus(r_min=inner,r_max=outer,height=.9,sections=48)
        mesh.apply_transform(trimesh.transformations.rotation_matrix(np.pi/2,[0,1,0]))
        mesh.apply_scale([1,1,flatten])
        mesh.apply_translation([x,0,0])
        add(name,mesh,mat)

    # Spider-shaped mounting plate contacts the four original chassis rails.
    # All vertices stay in x >= 0, beyond the chassis's front mounting plane.
    for y in (-8,8):
        for z in (-6,6):
            key=f'{y}_{z}'
            box('fixed_rail_pad_'+key,[.8,3,3],[.4,y,z],ti)
            tube('fixed_spider_'+key,[.8,y,z],[1.65,0,0],.63,ti)
            tube('fixed_pad_screw_'+key,[.8,y,z],[1.05,y,z],.72,steel,sections=20)
    box('fixed_lower_pad',[.8,3,3],[.4,0,-9.5],ti)
    tube('fixed_lower_strut',[.8,0,-9.5],[1.65,0,0],.63,ti)
    tube('fixed_neck_base',[1.1,0,0],[3,0,0],2.25,ti)

    # Yaw stage: vertical pin held in a rear fork. Pitch stage is carried by
    # the yaw hub and has a transverse pin. Parts are named for future motion.
    for s in (-1,1):
        box(f'fixed_yaw_fork_{s}',[2.8,1.55,.7],[4.1,0,s*1.9],ti)
        tube(f'fixed_yaw_bushing_{s}',[5.25,0,s*1.55],[5.25,0,s*2.25],.78,bronze,.39)
    tube('fixed_yaw_pin',[5.25,0,-2.6],[5.25,0,2.6],.37,steel)
    tube('yaw_hub',[5.25,0,-1.45],[5.25,0,1.45],1.23,ti,.40)
    box('yaw_carrier',[2.8,3.9,1.2],[7.0,0,0],ti)
    for s in (-1,1):
        box(f'yaw_pitch_fork_{s}',[2.1,.65,1.3],[8.2,s*1.62,0],ti)
        tube(f'yaw_pitch_bushing_{s}',[9.15,s*1.28,0],[9.15,s*1.98,0],.82,bronze,.42)
    tube('yaw_pitch_pin',[9.15,-2.55,0],[9.15,2.55,0],.40,steel)
    tube('pitch_hub',[9.15,-1.19,0],[9.15,1.19,0],1.18,ti,.43)
    box('pitch_neck_link',[2.4,1.55,1.4],[11.1,0,0],ti)
    # Servo housings, visible rings, terminal blocks and index markers.
    tube('fixed_yaw_servo',[5.25,0,2.6],[5.25,0,4.5],1.18,dark)
    tube('fixed_yaw_servo_cap',[5.25,0,4.5],[5.25,0,4.75],1.18,ti)
    tube('yaw_pitch_servo',[9.15,-2.55,0],[9.15,-4.35,0],1.15,dark)
    tube('yaw_pitch_servo_cap',[9.15,-4.35,0],[9.15,-4.6,0],1.15,ti)
    for i in range(3):
        tube(f'fixed_yaw_servo_band_{i}',[5.25,0,2.88+.45*i],[5.25,0,3.0+.45*i],1.23,copper)
        tube(f'yaw_pitch_servo_band_{i}',[9.15,-2.8-.45*i,0],[9.15,-2.92-.45*i,0],1.20,copper)
    box('fixed_yaw_index',[.35,.35,.12],[5.25,.65,4.81],cyan)
    box('yaw_pitch_index',[.35,.12,.35],[9.15,-4.66,.65],cyan)

    # Open cranial cage: three elliptical ribs connected by longitudinal
    # struts. The cranial cage moves with pitch, not with the fixed mount.
    specs=[(12.3,3.1,2.25,.90),(18.0,8.0,7.05,.76),(25.0,6.0,5.05,.84)]
    for i,(x,outer,inner,flatten) in enumerate(specs):
        ring_x(f'pitch_cranial_rib_{i}',x,outer,inner,flatten,ti)
    for i,angle in enumerate(np.deg2rad([45,135,225,315,90,270])):
        points=[np.array([x,(outer-.4)*np.cos(angle),(outer-.4)*flatten*np.sin(angle)]) for x,outer,_,flatten in specs]
        for j,(a,b) in enumerate(zip(points,points[1:])):
            tube(f'pitch_cranial_strut_{i}_{j}',a,b,.46, dark if i<4 else ti)
        for j,p in enumerate(points):
            # Spherical joint bosses on the rib/strut interfaces.
            boss=trimesh.creation.icosphere(subdivisions=2,radius=.62)
            boss.apply_translation(p); add(f'pitch_rib_boss_{i}_{j}',boss,steel)
    tube('pitch_cranial_center_rail',[12.3,0,-2.3],[25,0,-4.4],.52,ti)
    anchors={'mount':np.eye(4)}
    def frame(name, pos, axis=(1,0,0)):
        # Local +X is the attachment normal, +Z is dorsal where possible.
        x=np.asarray(axis,float); x/=np.linalg.norm(x)
        up=np.array([0.,0.,1.]) if abs(x[2])<.95 else np.array([0.,1.,0.])
        y=np.cross(up,x); y/=np.linalg.norm(y); z=np.cross(x,y)
        f=np.eye(4); f[:3,:3]=np.column_stack([x,y,z]); f[:3,3]=np.asarray(pos)*mm
        anchors[name]=f
    # Outward, slightly forward eye socket rings with a hollow aperture.
    # Eye modules will mount on the exposed outer face of each ring.
    for s,side in [(1,'left'),(-1,'right')]:
        center=np.array([19.5,s*9.0,2.2]); axis=np.array([.32,s*.9474,0.]); axis/=np.linalg.norm(axis)
        a=center-axis*.55; b=center+axis*.55
        tube(f'pitch_eye_socket_{side}',a,b,7.1,ti,6.15,sections=64)
        # Three stays tie the socket to the cage; no lenses are added yet.
        tangent=np.cross([0,0,1],axis)
        for i,ang in enumerate([0,2*np.pi/3,4*np.pi/3]):
            p=center+6.65*(np.cos(ang)*tangent+np.sin(ang)*np.array([0,0,1]))-axis*.45
            q=np.array([18 if i else 25,s*(7.6 if i else 5.6), np.clip(p[2],-4.4,4.4)])
            tube(f'pitch_eye_stay_{side}_{i}',p,q,.52,ti)
        frame(f'eye_{side}',b,axis)
    # Future face, three ocelli, antennae and data connection interfaces.
    box('pitch_face_mount',[.9,4,2.4],[25.75,0,-1.6],ti)
    for s in (-1,1):
        tube(f'pitch_antenna_boss_{s}',[24,s*2.5,4.3],[24,s*2.5,5.3],.68,ti,.32)
    frame('yaw_pivot',(5.25,0,0),(0,0,1))
    frame('pitch_pivot',(9.15,0,0),(0,1,0))
    frame('facial_armor',(26.2,0,-1.6))
    frame('antenna_left',(24,2.5,5.3),(0,0,1))
    frame('antenna_right',(24,-2.5,5.3),(0,0,1))
    frame('head_data',(2.2,0,-2.3),(-1,0,0))
    return Module('head',scene,anchors)


def build_stage_07(scale: float = 1.0) -> tuple[Module, trimesh.Scene]:
    """Attach the head to the original chassis neck frame."""
    _, prior=build_stage_06(scale)
    head=create_head(scale)
    root=Module('stage_06',prior,create_chassis(scale).anchors)
    return head,attach_module(root,'neck',head)


if __name__=='__main__':
    out=Path(__file__).resolve().parents[1]/'models'; out.mkdir(exist_ok=True)
    head,assembled=build_stage_07()
    head.scene.export(str(out/'step_07_head.glb'))
    assembled.export(str(out/'step_07_assembled.glb'))
    print('Head meshes:',len(head.scene.geometry))
    print('Head anchors:',len(head.anchors))
    print('Assembly meshes:',len(assembled.geometry))
    print('Assembly dimensions (mm):',np.round(assembled.extents*1000,3))
