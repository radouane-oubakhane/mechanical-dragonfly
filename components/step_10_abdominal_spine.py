"""Stage 10: ten tapered hollow vertebrae. Neutral pose; +X points forward.
Geometry is authored in millimeters and converted to meters at insertion.
"""
from pathlib import Path
import numpy as np
import trimesh
from step_01_chassis import Module, create_chassis
from step_09_facial_armor import build_stage_09

PITCHES = (19, 19, 18, 18, 17, 17, 16, 15, 14, 12)
RADII = (6.7, 6.5, 6.1, 5.7, 5.3, 4.9, 4.4, 3.9, 3.4, 2.9, 2.4)


def segment_dimensions(index: int) -> tuple[float, float, float]:
    """Return pitch, proximal radius, distal radius in nominal mm."""
    if isinstance(index, (bool, np.bool_)) or not isinstance(index, (int, np.integer)) or not 1 <= index <= 10:
        raise ValueError('index must be an integer from 1 to 10')
    return PITCHES[index-1], RADII[index-1], RADII[index]


def create_abdominal_segment(index: int, scale: float = 1.0) -> Module:
    """Generate one vertebra. Mount/next frames share the body orientation.
    The pivot frame's local Z is its hinge axis. Wire endpoints match neighbors.
    """
    L, r0, r1 = segment_dimensions(index)
    if not np.isfinite(scale) or scale <= 0:
        raise ValueError('scale must be positive and finite')
    mm = .001 * scale
    scene = trimesh.Scene(); anchors = {}
    def mat(name, rgb, metal, rough):
        return trimesh.visual.material.PBRMaterial(name=name,baseColorFactor=[*rgb,1],metallicFactor=metal,roughnessFactor=rough)
    ti=mat('Abdomen satin titanium',[.55,.58,.62],.93,.32)
    steel=mat('Abdomen polished steel',[.76,.79,.82],.98,.22)
    bronze=mat('Abdomen bronze bearing',[.60,.36,.16],.86,.30)
    carbon=mat('Abdomen carbon composite',[.045,.055,.065],.05,.44)
    copper=mat('Abdomen copper harness',[.74,.35,.15],.88,.30)
    insulation=mat('Abdomen cable insulation',[.025,.035,.045],0,.54)
    def add(name, mesh, material):
        mesh.apply_scale(mm); mesh.visual=trimesh.visual.TextureVisuals(material=material)
        scene.add_geometry(mesh,node_name=name,geom_name=name)
    def tube(name,a,b,r,material,inner=None,sections=20):
        a,b=np.asarray(a,float),np.asarray(b,float); d=b-a
        kwargs=dict(height=np.linalg.norm(d),sections=sections)
        m=(trimesh.creation.cylinder(radius=r,**kwargs) if inner is None else trimesh.creation.annulus(r_min=inner,r_max=r,**kwargs))
        m.apply_transform(trimesh.geometry.align_vectors([0,0,1],d)); m.apply_translation((a+b)/2); add(name,m,material)
    def anchor(name,pos,axis=None):
        f=np.eye(4); f[:3,3]=np.asarray(pos)*mm
        if axis is not None: f[:3,:3]=trimesh.geometry.align_vectors([0,0,1],axis)[:3,:3]
        anchors[name]=f
    def radius(t): return r0+(r1-r0)*t
    # Two hollow polygonal frames joined by four structural rails.
    for label,t in [('front',.24),('rear',.72)]:
        x=-L*t; r=radius(t); half=.08*r
        tube('frame_'+label,[x-half,0,0],[x+half,0,0],r,ti,inner=.74*r,sections=12)
        for k in range(4):
            theta=np.pi/4+k*np.pi/2; y,z=.86*r*np.cos(theta),.86*r*np.sin(theta)
            tube(f'fastener_{label}_{k}',[x-half-.16,y,z],[x-half,y,z],.065*r,steel,sections=12)
    for k in range(4):
        theta=np.pi/4+k*np.pi/2
        def point(t): return [-L*t,.86*radius(t)*np.cos(theta),.86*radius(t)*np.sin(theta)]
        tube(f'longitudinal_rail_{k}',point(.24),point(.72),.073*min(r0,r1),ti if k%2==0 else carbon)
    # The first mounting flange meets the chassis, while subsequent hubs
    # fit into the previous vertebra's fork. Joint axes alternate Y and Z.
    axis=np.array([0.,1.,0.]) if index%2 else np.array([0.,0.,1.])
    normal=np.cross([1.,0.,0.],axis)
    p=np.array([-2.1 if index==1 else 0.,0.,0.]); pin=.11*r0
    if index==1:
        tube('chassis_mount_flange',[-.65,0,0],[0,0,0],.90*r0,ti,inner=.45*r0)
        # Bridge the smaller flange to the rear adapter's hollow ring.
        # The forward ends intentionally engage the existing titanium frame.
        for s in (-1,1):
            tube(f'chassis_adapter_bridge_{s}',[.85,s*9.5,0],[-.38,s*5.4,0],.72,ti)
            tube(f'chassis_adapter_fastener_{s}',[.85,s*9.5,0],[-.9,s*9.5,0],.36,steel)
        for s in (-1,1):
            tube(f'chassis_mount_stay_{s}',[-.65,s*.65*r0,0],p+s*.43*r0*axis,.13*r0,ti)
            tube(f'chassis_mount_fork_{s}',p+s*.34*r0*axis,p+s*.52*r0*axis,.26*r0,ti,inner=pin*1.05)
    tube('moving_proximal_hub',p-.24*r0*axis,p+.24*r0*axis,.28*r0,ti,inner=pin*1.02)
    tube('proximal_hinge_pin',p-.56*r0*axis,p+.56*r0*axis,pin,steel)
    for s in (-1,1):
        tube(f'proximal_bushing_{s}',p+s*.24*r0*axis,p+s*.32*r0*axis,.20*r0,bronze,inner=pin*1.025)
        tube(f'proximal_pin_cap_{s}',p+s*.56*r0*axis,p+s*.62*r0*axis,.17*r0,steel)
        tube(f'proximal_stay_{s}',p+s*.18*r0*normal,[-.24*L,*(s*.81*radius(.24)*normal)[1:]],.10*r0,ti)
    # Distal fork cheeks have coaxial bores for the next segment's pin.
    if index<10:
        next_axis=np.array([0.,1.,0.]) if (index+1)%2 else np.array([0.,0.,1.]); q=np.array([-L,0.,0.])
        for s in (-1,1):
            tube(f'fixed_distal_fork_{s}',q+s*.34*r1*next_axis,q+s*.52*r1*next_axis,.26*r1,ti,inner=.11*r1*1.05)
            tube(f'fixed_distal_stay_{s}',[-.72*L,*(s*.82*radius(.72)*next_axis)[1:]],q+s*.43*r1*next_axis,.11*r1,ti)
    else:
        tube('terminal_adapter',[-L+.95,0,0],[-L,0,0],.70*r1,ti,inner=.38*r1)
        for s in (-1,1): tube(f'terminal_stay_{s}',[-.72*L,s*.82*radius(.72),0],[-L+.95,s*.48*r1,0],.12*r1,ti)
    # Two flexible harness paths, continuous across neutral-pose boundaries.
    for s,label in [(1,'left'),(-1,'right')]:
        pts=[np.array([-L*t,s*.36*radius(t),-.35*radius(t)+.30*np.sin(np.pi*t)]) for t in (0,.18,.42,.68,.86,1)]
        wire_r=.045*min(r0,r1)
        for k,(a,b) in enumerate(zip(pts,pts[1:])):
            tube(f'harness_{label}_{k}',a,b,wire_r,insulation if k in (0,4) else copper,sections=12)
        for k,v in enumerate(pts[1:-1]):
            m=trimesh.creation.icosphere(subdivisions=1,radius=wire_r); m.apply_translation(v); add(f'harness_elbow_{label}_{k}',m,insulation)
        anchor('wire_in_'+label,pts[0]); anchor('wire_out_'+label,pts[-1])
    anchor('mount',[0,0,0]); anchor('next',[-L,0,0]); anchor('armor',[-L/2,0,0]); anchor('pivot',p,axis)
    return Module(f'abdominal_segment_{index:02d}',scene,anchors)


def abdominal_placements(scale: float = 1.0) -> dict[str,np.ndarray]:
    """Segment frames in chassis coordinates for subsequent armor assembly."""
    transform=create_chassis(scale).anchors['abdomen'].copy(); result={}
    for index in range(1,11):
        segment=create_abdominal_segment(index,scale)
        placement=transform @ np.linalg.inv(segment.anchors['mount']); result[segment.name]=placement
        transform=placement @ segment.anchors['next']
    return result


def add_abdomen(scene: trimesh.Scene, origin: np.ndarray, scale: float=1.0) -> dict[str,Module]:
    """Insert into an owned scene; callers must not pass a shared scene."""
    transform=origin.copy(); modules={}
    for index in range(1,11):
        segment=create_abdominal_segment(index,scale)
        placement=transform @ np.linalg.inv(segment.anchors['mount'])
        for node in segment.scene.graph.nodes_geometry:
            local,geom=segment.scene.graph[node]; name=f'{segment.name}/{node}'
            scene.add_geometry(segment.scene.geometry[geom].copy(),node_name=name,geom_name=name,transform=placement@local)
        modules[segment.name]=segment; transform=placement @ segment.anchors['next']
    return modules


def build_stage_10(scale: float=1.0) -> tuple[dict[str,Module],trimesh.Scene]:
    """Attach ten vertebrae to a fresh Stage 09 assembly."""
    _,scene=build_stage_09(scale)
    modules=add_abdomen(scene,create_chassis(scale).anchors['abdomen'],scale)
    return modules,scene


if __name__=='__main__':
    out=Path(__file__).resolve().parents[1]/'models'; out.mkdir(exist_ok=True)
    modules,scene=build_stage_10()
    modules['abdominal_segment_01'].scene.export(str(out/'step_10_abdominal_segment.glb'))
    abdomen=trimesh.Scene(); add_abdomen(abdomen,np.eye(4))
    abdomen.export(str(out/'step_10_abdomen.glb')); scene.export(str(out/'step_10_assembled.glb'))
    print('Segments:',len(modules),'new meshes:',sum(len(m.scene.geometry) for m in modules.values()))
    print('Assembly meshes:',len(scene.geometry),'dimensions (mm):',scene.extents*1000)
