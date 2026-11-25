"""Stage 05: modular electromagnetic actuators and static wing linkages.
Local +X points outward from the transmission plate. Dimensions in mm;
mesh coordinates and attachment translations in meters. Concept geometry only.
"""
from pathlib import Path
import numpy as np
import trimesh
from step_01_chassis import Module, create_chassis
from step_02_power_core import attach_module
from step_04_transmission import build_stage_04

CHANNELS = tuple((side, pair) for pair in ('fore', 'hind') for side in ('left', 'right'))


def create_wing_actuator(side: str, pair: str, scale: float = 1.0) -> Module:
    """Build one actuator with mount, output, gear_input and electrical anchors.

    The same local geometry is reused for all four channels; the parent socket
    supplies its orientation. The crank/rocker is a static concept pose.
    """
    if side not in ('left', 'right') or pair not in ('fore', 'hind'):
        raise ValueError('invalid side or wing pair')
    if not np.isfinite(scale) or scale <= 0:
        raise ValueError('scale must be positive and finite')
    mm = .001 * scale
    scene = trimesh.Scene()
    anchors = {'mount': np.eye(4)}

    def material(name, color, metal, roughness):
        return trimesh.visual.material.PBRMaterial(name=name, baseColorFactor=color,
            metallicFactor=metal, roughnessFactor=roughness)
    ti = material('Actuator titanium', [.57,.60,.64,1], .94,.32)
    steel = material('Actuator polished steel', [.75,.78,.81,1], .96,.19)
    copper = material('Enamelled copper winding', [.76,.32,.12,1], .78,.27)
    dark = material('Black insulating polymer', [.045,.05,.06,1], 0,.48)
    bronze = material('Rocker bronze bushings', [.61,.35,.13,1], .85,.34)
    gold = material('Electrical contacts', [.84,.63,.22,1], .9,.24)

    def add(name, mesh, mat):
        # Construct in millimeters first to avoid tiny-feature merge tolerances.
        mesh.apply_scale(mm)
        mesh.visual = trimesh.visual.TextureVisuals(material=mat)
        scene.add_geometry(mesh, node_name=name, geom_name=name)

    def box(name, dims, pos, mat):
        m = trimesh.creation.box(extents=dims)
        m.apply_translation(pos)
        add(name,m,mat)

    def cylinder(name, radius, start, end, mat, sections=32):
        a,b = np.asarray(start,float),np.asarray(end,float)
        m = trimesh.creation.cylinder(radius=radius, height=np.linalg.norm(b-a),
            sections=sections)
        m.apply_transform(trimesh.geometry.align_vectors([0,0,1],b-a))
        m.apply_translation((a+b)/2)
        add(name,m,mat)

    def ring(name, ri, ro, start, end, mat):
        a,b = np.asarray(start,float),np.asarray(end,float)
        m = trimesh.creation.annulus(r_min=ri,r_max=ro,height=np.linalg.norm(b-a),sections=40)
        m.apply_transform(trimesh.geometry.align_vectors([0,0,1],b-a))
        m.apply_translation((a+b)/2)
        add(name,m,mat)

    def ax(x): return (x,0,0)

    # Narrow mounting tongue clears the two pre-existing plate fasteners.
    box('mounting_tongue',[.65,1.5,1.7],[.325,0,0],ti)
    cylinder('rear_end_cap',1.55,ax(.65),ax(1.15),ti)
    cylinder('insulated_bobbin',1.05,ax(1.15),ax(5.15),dark)
    cylinder('front_end_cap',1.55,ax(5.15),ax(5.75),ti)
    # Model a continuous helical copper winding, not stacked decorative rings.
    # The wire is swept along the helix with an orthogonal radial frame.
    turns=14; n=turns*32+1; sides=10
    t=np.linspace(0,2*np.pi*turns,n)
    x=np.linspace(1.35,4.95,n); radius=1.19; wire=.12
    centers=np.column_stack([x,radius*np.cos(t),radius*np.sin(t)])
    radial=np.column_stack([np.zeros(n),np.cos(t),np.sin(t)])
    tangent=np.column_stack([np.full(n,(x[-1]-x[0])/t[-1]),-radius*np.sin(t),radius*np.cos(t)])
    tangent/=np.linalg.norm(tangent,axis=1)[:,None]
    normal=np.cross(tangent,radial)
    angles=np.arange(sides)*2*np.pi/sides
    vertices=(centers[:,None,:]+wire*(radial[:,None,:]*np.cos(angles)[None,:,None]+normal[:,None,:]*np.sin(angles)[None,:,None])).reshape(-1,3)
    vertices=np.vstack([vertices,centers[0],centers[-1]])
    faces=[]
    for i in range(n-1):
        for j in range(sides):
            k=(j+1)%sides; a=i*sides+j; b=i*sides+k; c=(i+1)*sides+k; d=(i+1)*sides+j
            faces.extend([(a,b,c),(a,c,d)])
    for j in range(sides):
        k=(j+1)%sides
        faces.extend([(n*sides,k,j),(n*sides+1,(n-1)*sides+j,(n-1)*sides+k)])
    winding=trimesh.Trimesh(vertices=vertices,faces=faces,process=True)
    winding.fix_normals(); add('continuous_copper_helix',winding,copper)
    # Open cage exposes the winding. Two collars and four longitudinal ribs.
    for label,x0,x1 in [('rear',1.15,1.30),('front',5.00,5.15)]:
        ring(label+'_cage_collar',1.08,1.53,ax(x0),ax(x1),ti)
    for j in range(4):
        theta=np.pi/4+j*np.pi/2
        cylinder(f'cage_rail_{j}',.12,(1.23,1.47*np.cos(theta),1.47*np.sin(theta)),
                 (5.07,1.47*np.cos(theta),1.47*np.sin(theta)),ti,16)
    # Electrical connector and two separate contacts on the underside.
    box('connector_body',[1.35,1.25,.65],[2.2,0,-1.43],dark)
    for j,y in enumerate([-.29,.29]):
        cylinder(f'contact_{j}',.13,(2.2,y,-1.75),(2.2,y,-2.05),gold,16)
    # Piston guide, exposed rod and clevis.
    ring('piston_guide',.34,.72,ax(5.75),ax(6.25),bronze)
    cylinder('piston_rod',.32,ax(5.65),ax(7.3),steel)
    for sign in [-1,1]:
        box(f'clevis_{sign}',[.75,.22,.75],[7.22,sign*.47,0],ti)
    cylinder('clevis_pin',.23,(7.3,-.69,0),(7.3,.69,0),steel)
    # Fixed support fork and rocker pivot; two supports leave the center open.
    for sign in [-1,1]:
        y=sign*1.22
        box(f'fork_riser_{sign}',[.55,.30,1.6],[5.5,y,.8],ti)
        cylinder(f'fork_rail_{sign}',.21,(5.5,y,1.55),(8,y,1.55),ti)
        ring(f'pivot_bushing_{sign}',.30,.57,(8,y-.15,1.55),(8,y+.15,1.55),bronze)
    cylinder('rocker_pivot_pin',.30,(8,-1.45,1.55),(8,1.45,1.55),steel)
    lower=(7.3,0,0); pivot=(8,0,1.55); upper=(9.3,0,3.2)
    cylinder('rocker_lower_arm',.38,lower,pivot,ti)
    cylinder('rocker_upper_arm',.38,pivot,upper,ti)
    for label,p in [('lower',lower),('upper',upper)]:
        a=np.array(p); a[1]=-.64; b=np.array(p); b[1]=.64
        cylinder(label+'_rocker_pin',.23,a,b,steel)
    # Connect the existing vertical gear output to the rocker. The socket
    # offset (-5.5,0,2.35) is inherited from Stage 04's geometry.
    cylinder('gear_crank_hub',.68,(-5.5,0,2.35),(-5.5,0,3.0),bronze)
    cylinder('gear_crank_arm',.23,(-5.5,0,3.0),(-3.8,0,3.0),ti)
    cylinder('gear_crank_pin',.22,(-3.8,0,3.0),(-3.8,0,3.55),steel)
    cylinder('gear_to_rocker_link',.20,(-3.8,0,3.35),(9.3,0,3.2),steel)
    # Outward-facing interface for the next wing-root hinge.
    box('output_mount_lug',[.8,.9,.8],[9.7,0,3.2],ti)
    for name,pos in [('output',(10.1,0,3.2)),('gear_input',(-5.5,0,2.35)),
                     ('electrical_positive',(2.2,-.29,-2.05)),('electrical_negative',(2.2,.29,-2.05)),
                     ('rocker_pivot',(8,0,1.55))]:
        a=np.eye(4); a[:3,3]=np.array(pos)*mm; anchors[name]=a
    return Module(f'actuator_{pair}_{side}',scene,anchors)


def build_stage_05(scale: float=1.0) -> tuple[dict[str,Module],trimesh.Scene]:
    """Assemble four actuators, retaining all earlier meshes and transforms."""
    transmission, assembled=build_stage_04(scale)
    t=create_chassis(scale).anchors['transmission']
    modules={}
    for side,pair in CHANNELS:
        module=create_wing_actuator(side,pair,scale)
        socket=module.name
        placement=t@transmission.anchors[socket]
        assembled=attach_module(Module('stage_05',assembled,{socket:placement}),socket,module)
        modules[socket]=module
    return modules,assembled


if __name__=='__main__':
    out=Path(__file__).resolve().parent
    actuators,assembled=build_stage_05()
    actuators['actuator_fore_left'].scene.export(str(out/'step_05_actuator.glb'))
    assembled.export(str(out/'step_05_assembled.glb'))
    print('Meshes per actuator:',len(actuators['actuator_fore_left'].scene.geometry))
    print('Actuator modules:',len(actuators))
    print('Assembly meshes:',len(assembled.geometry))
    print('Assembly dimensions (mm):',np.round(assembled.extents*1000,3))
