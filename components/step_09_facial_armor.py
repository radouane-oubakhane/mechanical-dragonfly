"""Stage 09: segmented face, three ocelli, antennae and neutral-pose mandibles.
Geometry authored in millimeters; exported in meters. +X is forward.
"""
from pathlib import Path
import numpy as np
import trimesh
from step_01_chassis import Module, create_chassis
from step_02_power_core import attach_module
from step_07_head import create_head
from step_08_eyes import build_stage_08


def create_facial_armor(scale: float = 1.0) -> Module:
    """Create the face module; antenna bases derive from the actual head frames."""
    if not np.isfinite(scale) or scale <= 0:
        raise ValueError('scale must be positive and finite')
    mm = .001 * scale
    scene = trimesh.Scene()
    anchors = {'mount': np.eye(4)}
    def mat(name, rgb, metal, rough, alpha=1):
        return trimesh.visual.material.PBRMaterial(name=name,baseColorFactor=[*rgb,alpha],metallicFactor=metal,roughnessFactor=rough,alphaMode='BLEND' if alpha<1 else 'OPAQUE')
    ti=mat('Face satin titanium',[.56,.59,.62],.94,.32)
    steel=mat('Face polished steel',[.75,.79,.81],.97,.20)
    dark=mat('Face carbon composite',[.045,.055,.064],.05,.48)
    bronze=mat('Face bronze bushings',[.63,.39,.16],.86,.31)
    gold=mat('Sensor contact gold',[.81,.58,.19],.9,.27)
    glass=mat('Ocellus tinted optical glass',[.07,.37,.42],.12,.10,.9)
    copper=mat('Antenna copper conductors',[.74,.36,.17],.90,.29)
    def add(name,mesh,material):
        mesh.apply_scale(mm)
        mesh.visual=trimesh.visual.TextureVisuals(material=material)
        scene.add_geometry(mesh,node_name=name,geom_name=name)
    def box(name,size,center,material):
        m=trimesh.creation.box(extents=size); m.apply_translation(center); add(name,m,material)
    def tube(name,a,b,r,material,inner=None,sections=24):
        a,b=np.asarray(a,float),np.asarray(b,float); d=b-a; length=np.linalg.norm(d)
        m=(trimesh.creation.cylinder(radius=r,height=length,sections=sections) if inner is None else trimesh.creation.annulus(r_min=inner,r_max=r,height=length,sections=sections))
        m.apply_transform(trimesh.geometry.align_vectors([0,0,1],d)); m.apply_translation((a+b)/2); add(name,m,material)
    def ellipsoid(name,center,radii,material):
        m=trimesh.creation.icosphere(subdivisions=3); m.apply_scale(radii); m.apply_translation(center); add(name,m,material)
    def plate(name,outline,x0,x1,material):
        # Convex, tapered front face and closed solid sidewalls.
        p=np.array(outline,float); c=p.mean(axis=0)
        vertices=np.vstack([np.column_stack([np.full(len(p),x0),p]),np.column_stack([np.full(len(p),x1),c+.84*(p-c)])])
        add(name,trimesh.convex.convex_hull(vertices),material)
    def anchor(name,pos,axis=(1,0,0)):
        x=np.array(axis,float); x/=np.linalg.norm(x); up=np.array([0.,0.,1.]) if abs(x[2])<.95 else np.array([0.,1.,0.]); y=np.cross(up,x); y/=np.linalg.norm(y)
        f=np.eye(4); f[:3,:3]=np.column_stack([x,y,np.cross(x,y)]); f[:3,3]=np.asarray(pos)*mm; anchors[name]=f
    # Rear mounting face is flush with the original head's face plate.
    box('mount_backplate',[.65,3.8,2.3],[.325,0,0],ti)
    plate('central_mask',[(-1.8,-1.1),(1.8,-1.1),(2.15,.75),(1.25,2.1),(-1.25,2.1),(-2.15,.75)],.65,1.35,dark)
    plate('brow_armor',[(-2.3,1.9),(2.3,1.9),(2.65,3.7),(1.25,5.2),(-1.25,5.2),(-2.65,3.7)],.18,.95,ti)
    box('brow_support',[.55,1.1,2.3],[.30,0,1.65],ti)
    for s,side in [(1,'left'),(-1,'right')]:
        outline=[(s*y,z) for y,z in [(1.9,-1.1),(3.25,-.7),(3.65,.55),(2.65,1.75),(2.0,1.3)]]
        plate('cheek_'+side,outline,.18,1.05,ti)
        tube('cheek_stay_'+side,[.25,s*1.6,.15],[.40,s*3.05,.25],.36,ti)
        for i in range(3):
            # Separate louver bars with open gaps, above dark backing.
            box(f'vent_{side}_{i}',[.22,.85,.14],[1.24,s*2.55,-.28+.34*i],dark)
        for i,z in enumerate([-.55,.95]):
            tube(f'cheek_screw_{side}_{i}',[1.05,s*2.23,z],[1.28,s*2.23,z],.20,steel,sections=16)
        # Mandible pivot pin along transverse axis, with a swept two-part jaw.
        p=np.array([.92,s*1.65,-1.05]); q=np.array([1.5,s*2.25,-2.25]); r=np.array([2.45,s*1.65,-3.30]); tip=np.array([3.0,s*.62,-2.98])
        tube('jaw_fixed_boss_'+side,p+[0,-.48,0],p+[0,.48,0],.65,ti)
        tube('jaw_pin_'+side,p+[0,-.68,0],p+[0,.68,0],.26,steel)
        tube('jaw_bushing_'+side,p+[0,s*.48,0],p+[0,s*.65,0],.43,bronze,.27)
        tube('jaw_moving_upper_'+side,p,q,.43,ti)
        tube('jaw_moving_lower_'+side,q,r,.36,ti)
        tube('jaw_moving_tip_'+side,r,tip,.23,steel)
        ellipsoid('jaw_moving_knuckle_'+side,q,[.47,.42,.47],dark)
        for i,t in enumerate([.40,.72]):
            a=q+t*(r-q); b=a+[.20,-s*.31,.14]
            tube(f'jaw_moving_tooth_{side}_{i}',a,b,.13,steel,sections=12)
        anchor('mandible_'+side,p,(0,1,0))
    # Three auxiliary simple eyes in a triangular arrangement.
    for name,y,z,r in [('median',0,4.17,.68),('left',1.36,2.95,.59),('right',-1.36,2.95,.59)]:
        tube('ocellus_housing_'+name,[.88,y,z],[1.38,y,z],r+.20,ti)
        tube('ocellus_bezel_'+name,[1.38,y,z],[1.60,y,z],r+.14,dark,r*.87)
        ellipsoid('ocellus_lens_'+name,[1.50,y,z],[.34,r*.88,r*.88],glass)
        tube('ocellus_contact_'+name,[.75,y,z],[.88,y,z],.23,gold,sections=16)
        anchor('ocellus_'+name,[1.84,y,z])
    # Antenna mounting locations are transformed from the head into face space.
    head=create_head(scale); inv=np.linalg.inv(head.anchors['facial_armor'])
    for s,side in [(1,'left'),(-1,'right')]:
        frame=inv @ head.anchors['antenna_'+side]; base=frame[:3,3]/mm
        anchor('antenna_'+side,base,(0,0,1))
        tube('antenna_stem_'+side,base,base+[0,0,.55],.30,steel)
        tube('antenna_collar_'+side,base+[0,0,.12],base+[0,0,.42],.52,bronze,.31)
        pts=[base+[0,0,.55],base+[.15,s*.22,1.3],base+[.65,s*.44,2.35],base+[1.38,s*.62,3.28],base+[2.12,s*.72,4.05]]
        for i,(a,b) in enumerate(zip(pts,pts[1:])):
            tube(f'antenna_{side}_segment_{i}',a,b,[.25,.21,.17,.12][i],ti if i%2==0 else dark)
            ellipsoid(f'antenna_{side}_joint_{i}',a,[.27-i*.035]*3,copper)
        ellipsoid('antenna_tip_'+side,pts[-1],[.16,.16,.18],gold)
    # Short protected sensor traces on front mask; no claim of electrical simulation.
    for s,side in [(1,'left'),(-1,'right')]:
        pts=[[1.36,s*.65,1.1],[1.36,s*.9,1.55],[1.05,s*.9,2.35]]
        for i,(a,b) in enumerate(zip(pts,pts[1:])): tube(f'sensor_trace_{side}_{i}',a,b,.09,copper,sections=12)
    anchor('face_data',[.25,0,-.8],(-1,0,0))
    return Module('facial_armor',scene,anchors)


def build_stage_09(scale: float = 1.0) -> tuple[Module,trimesh.Scene]:
    """Attach the face to the existing head while preserving all earlier geometry."""
    _,prior=build_stage_08(scale)
    head=create_head(scale); placement=create_chassis(scale).anchors['neck'] @ head.anchors['facial_armor']
    face=create_facial_armor(scale)
    return face,attach_module(Module('stage_09',prior,{'face':placement}),'face',face)


if __name__=='__main__':
    out=Path(__file__).resolve().parents[1]/'models'; out.mkdir(exist_ok=True)
    face,scene=build_stage_09()
    face.scene.export(str(out/'step_09_facial_armor.glb'))
    scene.export(str(out/'step_09_assembled.glb'))
    print('Face meshes:',len(face.scene.geometry)); print('Face anchors:',len(face.anchors))
    print('Assembly meshes:',len(scene.geometry)); print('Assembly dimensions (mm):',scene.extents*1000)
