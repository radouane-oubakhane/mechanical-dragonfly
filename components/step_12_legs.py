"""Stage 12: six mirrored mechanical legs in a neutral landing pose.
Nominal geometry in mm; exported meshes and anchor translations in meters.
Joint anchors have local +Z along the hinge axis. No dynamic solver yet.
"""
from pathlib import Path
import numpy as np
import trimesh
from step_01_chassis import Module, create_chassis
from step_11_abdominal_armor import build_stage_11


def swept_wire(points, radius, sides=12):
    """Make a capped tube with a transported cross section along a smooth path."""
    p=np.asarray(points,float)
    tang=np.gradient(p,axis=0); tang/=np.linalg.norm(tang,axis=1)[:,None]
    normals=[]; u=np.eye(3)[np.argmin(np.abs(tang[0]))]
    for t in tang:
        u=u-np.dot(u,t)*t; u/=np.linalg.norm(u); normals.append(u.copy())
    u=np.asarray(normals); v=np.cross(tang,u)
    a=np.arange(sides)*2*np.pi/sides
    verts=(p[:,None,:]+radius*(u[:,None,:]*np.cos(a)[None,:,None]+v[:,None,:]*np.sin(a)[None,:,None])).reshape(-1,3)
    n=len(p); verts=np.vstack([verts,p[0],p[-1]])
    faces=[]
    for i in range(n-1):
        for j in range(sides):
            k=(j+1)%sides; b=i*sides+j; c=i*sides+k; d=(i+1)*sides+k; e=(i+1)*sides+j
            faces.extend([(b,c,d),(b,d,e)])
    for j in range(sides):
        k=(j+1)%sides; faces.extend([(n*sides,k,j),(n*sides+1,(n-1)*sides+j,(n-1)*sides+k)])
    m=trimesh.Trimesh(vertices=verts,faces=faces,process=True); m.fix_normals(); return m


def create_leg(leg_pair: int, side: str, scale: float=1.0) -> Module:
    """Build a single leg with two hip axes, knee, ankle and three tarsal links.
    Pair 1 sweeps forward, pair 2 extends sideways, pair 3 sweeps rearward.
    Right legs mirror the left geometry while maintaining right-handed frames.
    """
    if isinstance(leg_pair,(bool,np.bool_)) or not isinstance(leg_pair,(int,np.integer)) or leg_pair not in (1,2,3):
        raise ValueError('leg_pair must be 1, 2 or 3')
    if side not in ('left','right'): raise ValueError('side must be left or right')
    if not np.isfinite(scale) or scale<=0: raise ValueError('scale must be positive and finite')
    mm=.001*scale; scene=trimesh.Scene(); anchors={}; sign=1 if side=='left' else -1
    mirror=np.diag([1.,float(sign),1.,1.]); flip=mirror[:3,:3]
    def mat(name,rgb,metal,rough):
        return trimesh.visual.material.PBRMaterial(name=name,baseColorFactor=[*rgb,1],metallicFactor=metal,roughnessFactor=rough)
    ti=mat('Leg satin titanium',[.56,.59,.63],.94,.33); steel=mat('Leg polished steel',[.76,.79,.83],.98,.20)
    bronze=mat('Leg bronze bushings',[.62,.36,.16],.85,.32); carbon=mat('Leg carbon composite',[.045,.055,.065],.05,.44)
    copper=mat('Leg spring copper alloy',[.72,.38,.16],.84,.29); dark=mat('Leg insulated harness',[.025,.032,.04],0,.53)
    cyan=mat('Leg encoder index',[.07,.58,.69],.25,.29)
    def add(name,m,material):
        m.apply_transform(mirror); m.apply_scale(mm); m.visual=trimesh.visual.TextureVisuals(material=material)
        scene.add_geometry(m,node_name=name,geom_name=name)
    def tube(name,a,b,r,material,inner=None,sections=20):
        a,b=np.asarray(a,float),np.asarray(b,float); d=b-a
        kw=dict(height=np.linalg.norm(d),sections=sections)
        m=trimesh.creation.cylinder(radius=r,**kw) if inner is None else trimesh.creation.annulus(r_min=inner,r_max=r,**kw)
        m.apply_transform(trimesh.geometry.align_vectors([0,0,1],d)); m.apply_translation((a+b)/2); add(name,m,material)
    def box(name,size,p,material):
        m=trimesh.creation.box(extents=size); m.apply_translation(p); add(name,m,material)
    def anchor(name,p,axis=None):
        f=np.eye(4); f[:3,3]=flip@np.asarray(p)*mm
        if axis is not None: f[:3,:3]=trimesh.geometry.align_vectors([0,0,1],flip@np.asarray(axis))[:3,:3]
        anchors[name]=f
    def joint(label,p,axis,r):
        # Fixed pin, hollow rotor, separated bushings and retaining caps.
        p=np.asarray(p); a=np.asarray(axis,float); a/=np.linalg.norm(a)
        tube(label+'_pin',p-1.1*r*a,p+1.1*r*a,.30*r,steel)
        tube(label+'_rotor',p-.52*r*a,p+.52*r*a,.72*r,ti,.32*r)
        for s in (-1,1):
            tube(f'{label}_bushing_{s}',p+s*.57*r*a,p+s*.82*r*a,.60*r,bronze,.32*r)
            tube(f'{label}_fork_eye_{s}',p+s*.57*r*a,p+s*.94*r*a,.94*r,ti,.62*r)
            tube(f'{label}_cap_{s}',p+s*1.1*r*a,p+s*1.22*r*a,.48*r,steel)
        anchor(label+'_pivot',p,a)
    def beam(label,a,b,r):
        a,b=np.asarray(a),np.asarray(b); d=b-a; u=d/np.linalg.norm(d)
        # A central composite spar with paired titanium reinforcing rails.
        n=np.cross(u,[0,0,1.]); n/=np.linalg.norm(n)
        tube(label+'_spar',a+.65*u,b-.65*u,r,carbon)
        for s in (-1,1):
            tube(f'{label}_rail_{s}',a+.65*u+s*.67*r*n,b-.65*u+s*.67*r*n,.32*r,ti)
        for t in (.20,.80):
            q=a+t*d; tube(f'{label}_collar_{t}',q-.26*u,q+.26*u,1.04*r,ti)
    def piston(label,a,b,r):
        a,b=np.asarray(a),np.asarray(b); d=b-a; u=d/np.linalg.norm(d)
        tube(label+'_body',a,a+.58*d,r,ti); tube(label+'_guide',a+.58*d,a+.65*d,.81*r,bronze,.42*r)
        tube(label+'_rod',a+.54*d,b,.38*r,steel)
        for s,q in enumerate((a,b)):
            tube(f'{label}_pin_{s}',q-[.65*r,0,0],q+[.65*r,0,0],.38*r,steel)
    sweep={1:1.,2:0.,3:-1.}[leg_pair]; reach={1:0.,2:3.,3:0.}[leg_pair]
    root=np.array([0.,0.,0.]); yaw=np.array([0.,2.,-1.5]); hip=np.array([sweep*1.8,4.7,-5.])
    knee=np.array([sweep*8.,13.+reach,-12.]); ankle=np.array([sweep*13.,22.+reach,-27.])
    toe=[ankle,np.array([sweep*13.5,23.4+reach,-30.]),np.array([sweep*14.,24.4+reach,-32.]),np.array([sweep*14.,25.+reach,-33.])]
    # Chassis attachment intentionally engages the original ring at y=11 mm.
    box('fixed_mount_plate',[2.8,1.5,2.6],[0,.25,0],ti)
    for s in (-1,1): tube(f'fixed_mount_bolt_{s}',[s*.91,.85,0],[s*.91,1.2,0],.34,steel)
    tube('fixed_yaw_stay',root,yaw,.85,ti); joint('hip_yaw',yaw,[0,0,1],1.15)
    tube('yaw_coxa',yaw,hip,.76,ti); joint('hip_pitch',hip,[1,0,0],1.12)
    beam('femur',hip,knee,.90); joint('knee',knee,[1,0,0],1.02)
    beam('tibia',knee,ankle,.69); joint('ankle',ankle,[1,0,0],.73)
    piston('femur_actuator',hip+[0,1.1,.5],knee+[0,-1.5,1.2],.52)
    piston('tibia_actuator',knee+[0,1.2,.3],ankle+[0,-1.3,1.5],.43)
    # Continuous tension spring on the outer femur, with straight hook ends.
    a=hip+np.array([1.55,1.2,-.5]); b=knee+np.array([1.55,-1.4,1.2]); d=b-a; u=d/np.linalg.norm(d)
    n=np.cross(u,[1.,0,0]); n/=np.linalg.norm(n); v=np.cross(u,n)
    t=np.linspace(0,1,201); theta=2*np.pi*9*t
    helix=a+t[:,None]*d+.45*(np.cos(theta)[:,None]*n+np.sin(theta)[:,None]*v)
    add('femur_tension_spring',swept_wire(helix,.095),copper)
    tube('spring_hook_upper',hip,a+.45*n,.13,steel); tube('spring_hook_lower',b+.45*n,knee,.13,steel)
    # A continuous curved electrical cable down the leg with strain-relief collars.
    pts=np.array([root+[0,0,-.65],yaw+[.7,0,-.5],hip+[.9,0,-.4],hip*.45+knee*.55+[1.2,0,-.25],knee+[.95,0,-.25],knee*.45+ankle*.55+[.8,0,-.2],ankle+[.55,0,-.2]])
    # Interpolating cubic Hermite path, no scipy dependency in the module.
    tang=np.gradient(pts,axis=0); path=[]
    for i in range(len(pts)-1):
        for t in np.linspace(0,1,12,endpoint=False):
            path.append((2*t**3-3*t*t+1)*pts[i]+(t**3-2*t*t+t)*tang[i]+(-2*t**3+3*t*t)*pts[i+1]+(t**3-t*t)*tang[i+1])
    path.append(pts[-1]); add('leg_data_harness',swept_wire(path,.15),dark)
    for j,q in enumerate(pts[1:-1]):
        m=trimesh.creation.icosphere(subdivisions=1,radius=.24); m.apply_translation(q); add(f'wire_clip_{j}',m,ti)
    for label,p in [('hip',hip),('knee',knee),('ankle',ankle)]:
        tube(label+'_encoder',p+[1.25,0,0],p+[1.48,0,0],.38,carbon)
        tube(label+'_encoder_index',p+[1.48,0,0],p+[1.54,0,0],.18,cyan)
    # Three tarsal links, with distal pins and split curved gripping claws.
    for i,(a,b) in enumerate(zip(toe,toe[1:]),1):
        r=.50-.075*i; tube(f'tarsus_{i}_link',a,b,r,carbon)
        joint(f'tarsus_{i}',b,[1,0,0],r*1.25)
    for s,label in [(-1,'inner'),(1,'outer')]:
        p=toe[-1]+np.array([s*.42,0,0]); q=p+[s*.24,.82,-.28]; r=q+[s*.06,.40,-.72]; tip=r+[0,-.36,-.38]
        tube(f'claw_{label}_base',toe[-1],p,.23,ti)
        # Taper by using short, closed links; pin-like tip forms the hook.
        tube(f'claw_{label}_proximal',p,q,.22,ti); tube(f'claw_{label}_distal',q,r,.15,steel)
        m=trimesh.creation.cone(radius=.14,height=np.linalg.norm(tip-r),sections=16)
        m.apply_transform(trimesh.geometry.align_vectors([0,0,1],tip-r)); m.apply_translation(r); add(f'claw_{label}_tip',m,steel)
        anchor('claw_'+label,tip,[0,-1,-1])
    anchor('mount',root); anchor('foot',toe[-1]); anchor('data',pts[0])
    return Module(f'leg_{leg_pair}_{side}',scene,anchors)


def build_stage_12(scale: float=1.0) -> tuple[dict[str,Module],trimesh.Scene]:
    """Insert all six legs using the original chassis mount frames."""
    _,scene=build_stage_11(scale); chassis=create_chassis(scale); modules={}
    for pair in (1,2,3):
        for side in ('left','right'):
            leg=create_leg(pair,side,scale); modules[leg.name]=leg
            placement=chassis.anchors[leg.name]@np.linalg.inv(leg.anchors['mount'])
            for node in leg.scene.graph.nodes_geometry:
                local,g=leg.scene.graph[node]; name=f'{leg.name}/{node}'
                scene.add_geometry(leg.scene.geometry[g].copy(),node_name=name,geom_name=name,transform=placement@local)
    return modules,scene


if __name__=='__main__':
    out=Path(__file__).resolve().parents[1]/'models'; out.mkdir(exist_ok=True)
    modules,scene=build_stage_12(); modules['leg_1_left'].scene.export(str(out/'step_12_leg.glb'))
    legs=trimesh.Scene()
    for n in scene.graph.nodes_geometry:
        if n.startswith('leg_'):
            t,g=scene.graph[n]; legs.add_geometry(scene.geometry[g].copy(),node_name=n,geom_name=n,transform=t)
    legs.export(str(out/'step_12_legs.glb')); scene.export(str(out/'step_12_assembled.glb'))
    print('Meshes per leg:',len(modules['leg_1_left'].scene.geometry))
    print('Assembly meshes:',len(scene.geometry),'dimensions (mm):',scene.extents*1000)
