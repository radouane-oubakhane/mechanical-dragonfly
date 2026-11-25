"""Stage 06: four static, articulated wing-root hinge modules.
Local +X points outward, +Y is the flap axis, +Z is dorsal.
Build in millimeters, then convert to meters for all mesh and anchor data.
"""
from pathlib import Path
import numpy as np
import trimesh
from step_01_chassis import Module, create_chassis
from step_02_power_core import attach_module
from step_04_transmission import create_transmission
from step_05_actuators import CHANNELS, build_stage_05


def create_wing_joint(side: str, pair: str, scale: float = 1.0) -> Module:
    """Create a fixed clevis and neutral-pose pivoted wing socket.

    The parent actuator supplies the side-specific orientation. The `wing`
    frame is the future wing root; `flap_pivot` defines its local +Y axis.
    Geometry is grouped by fixed/rotor prefixes for future kinematics.
    """
    if side not in ('left', 'right') or pair not in ('fore', 'hind'):
        raise ValueError('invalid wing side or pair')
    if not np.isfinite(scale) or scale <= 0:
        raise ValueError('scale must be positive and finite')
    mm = .001 * scale
    scene = trimesh.Scene()
    def material(name, rgb, metal, roughness):
        return trimesh.visual.material.PBRMaterial(name=name, baseColorFactor=[*rgb, 1], metallicFactor=metal, roughnessFactor=roughness)
    ti = material('Wing joint titanium', [.58,.61,.64], .93,.32)
    steel = material('Hinge polished steel', [.75,.78,.81], .97,.19)
    bronze = material('Hinge bronze', [.65,.40,.17], .86,.32)
    dark = material('Wing socket carbon composite', [.055,.065,.075], .05,.44)
    copper = material('Torsion spring copper alloy', [.72,.39,.18], .82,.29)
    sensor = material('Encoder housing', [.10,.13,.16], .25,.36)
    cyan = material('Encoder index marker', [.08,.58,.7], .15,.27)
    def add(name, mesh, mat):
        mesh.apply_scale(mm)
        mesh.visual = trimesh.visual.TextureVisuals(material=mat)
        scene.add_geometry(mesh, node_name=name, geom_name=name)
    def box(name, size, center, mat):
        mesh = trimesh.creation.box(extents=size)
        mesh.apply_translation(center)
        add(name, mesh, mat)
    def round_part(name, start, end, outer, mat, inner=None, sections=40):
        a,b = np.asarray(start,float),np.asarray(end,float)
        length = np.linalg.norm(b-a)
        if inner is None:
            mesh = trimesh.creation.cylinder(radius=outer,height=length,sections=sections)
        else:
            mesh = trimesh.creation.annulus(r_min=inner,r_max=outer,height=length,sections=sections)
        mesh.apply_transform(trimesh.geometry.align_vectors([0,0,1],b-a))
        mesh.apply_translation((a+b)/2)
        add(name,mesh,mat)
    def along_y(name, y0, y1, radius, mat, inner=None):
        round_part(name,(2.2,y0,0),(2.2,y1,0),radius,mat,inner)
    # Fixed mounting face touches, but does not penetrate, the actuator lug.
    box('fixed_mount_tongue',[.65,.9,.8],[.325,0,0],ti)
    box('fixed_clevis_bridge',[.55,2.65,1.15],[.925,0,0],ti)
    for sign in (-1,1):
        y=sign*1.12
        box(f'fixed_fork_arm_{sign}',[1.05,.40,.85],[1.65,y,0],ti)
        along_y(f'fixed_fork_eye_{sign}',y-.2,y+.2,.82,ti,.47)
        along_y(f'fixed_bushing_{sign}',y-.23,y+.23,.46,bronze,.30)
    # Central pin and separate moving sleeve, clear of the fixed fork ears.
    along_y('fixed_pivot_pin',-2.0,2.0,.29,steel)
    along_y('rotor_hub',-.86,.86,.72,ti,.31)
    for sign in (-1,1):
        y=sign*.90
        along_y(f'rotor_thrust_washer_{sign}',y-.03,y+.03,.53,bronze,.31)
    # Wing tongue and composite clamp; slot is modeled as two opposing jaws.
    box('rotor_radial_arm',[1.05,.95,.75],[3.05,0,0],ti)
    box('rotor_socket_back',[.35,1.5,.88],[3.70,0,0],ti)
    for sign in (-1,1):
        z=sign*.35
        box(f'rotor_socket_jaw_{sign}',[1.05,1.5,.20],[4.40,0,z],dark)
        for y in (-.49,.49):
            round_part(f'rotor_jaw_fastener_{sign}_{y}',(4.35,y,sign*.45),(4.35,y,sign*.57),.15,steel,sections=16)
    # One continuous helical torsion spring on the exposed negative-Y pin.
    # Closed swept tube; end hooks are represented by separate straight legs.
    t=np.linspace(0,2*np.pi*3,145)
    yy=np.linspace(-1.94,-1.40,len(t)); radius=.52; wire=.065
    centers=np.column_stack([2.2+radius*np.cos(t),yy,radius*np.sin(t)])
    radial=np.column_stack([np.cos(t),np.zeros(len(t)),np.sin(t)])
    tangent=np.column_stack([-radius*np.sin(t),np.full(len(t),.54/t[-1]),radius*np.cos(t)])
    tangent/=np.linalg.norm(tangent,axis=1)[:,None]
    normal=np.cross(tangent,radial); sides=10
    angles=np.arange(sides)*2*np.pi/sides
    verts=(centers[:,None,:]+wire*(radial[:,None,:]*np.cos(angles)[None,:,None]+normal[:,None,:]*np.sin(angles)[None,:,None])).reshape(-1,3)
    n=len(t); verts=np.vstack([verts,centers[0],centers[-1]]); faces=[]
    for i in range(n-1):
        for j in range(sides):
            k=(j+1)%sides; a=i*sides+j; b=i*sides+k; c=(i+1)*sides+k; d=(i+1)*sides+j
            faces.extend([(a,b,c),(a,c,d)])
    for j in range(sides):
        k=(j+1)%sides; faces.extend([(n*sides,k,j),(n*sides+1,(n-1)*sides+j,(n-1)*sides+k)])
    spring=trimesh.Trimesh(vertices=verts,faces=faces,process=True); spring.fix_normals()
    add('fixed_torsion_spring',spring,copper)
    round_part('fixed_spring_leg',centers[0],(1.65,-1.94,0),wire,copper,sections=12)
    round_part('rotor_spring_leg',centers[-1],(3.15,-1.40,0),wire,copper,sections=12)
    # Encoder on the opposite side, with visible index mark and housing cap.
    along_y('fixed_encoder_shell',1.35,1.92,.76,sensor,.32)
    along_y('rotor_encoder_disk',1.45,1.80,.30,steel)
    along_y('fixed_encoder_cap',1.92,2.05,.76,ti)
    box('fixed_encoder_index',[.18,.12,.26],[2.65,2.07,0],cyan)
    # Two static stop blocks and a rotor tab; not a certified range limiter.
    for sign in (-1,1):
        box(f'fixed_stop_{sign}',[.40,.40,.33],[2.72,1.12,sign*.76],steel)
    box('rotor_stop_tab',[.40,.45,.30],[2.85,.65,0],bronze)
    anchors={'mount':np.eye(4)}
    for name,pos in [('wing',(4.925,0,0)),('flap_pivot',(2.2,0,0)),('encoder',(2.2,2.13,0)),('spring',(2.2,-1.67,0))]:
        frame=np.eye(4); frame[:3,3]=np.array(pos)*mm; anchors[name]=frame
    return Module(f'wing_joint_{pair}_{side}',scene,anchors)


def build_stage_06(scale: float = 1.0) -> tuple[dict[str, Module], trimesh.Scene]:
    """Snap each joint's mount to its existing actuator's output frame."""
    actuators, assembled=build_stage_05(scale)
    transmission=create_transmission(scale)
    root=create_chassis(scale).anchors['transmission']
    joints={}
    for side,pair in CHANNELS:
        key=f'actuator_{pair}_{side}'
        placement=root@transmission.anchors[key]@actuators[key].anchors['output']
        joint=create_wing_joint(side,pair,scale)
        assembled=attach_module(Module('stage_06',assembled,{key:placement}),key,joint)
        joints[joint.name]=joint
    return joints,assembled

if __name__=='__main__':
    out=Path(__file__).resolve().parent
    joints,assembled=build_stage_06()
    joints['wing_joint_fore_left'].scene.export(str(out/'step_06_wing_joint.glb'))
    assembled.export(str(out/'step_06_assembled.glb'))
    print('Meshes per joint:',len(joints['wing_joint_fore_left'].scene.geometry))
    print('Joint modules:',len(joints))
    print('Assembly meshes:',len(assembled.geometry))
    print('Assembly dimensions (mm):',np.round(assembled.extents*1000,3))
