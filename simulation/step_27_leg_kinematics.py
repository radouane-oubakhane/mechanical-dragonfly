"""Six-leg kinematics. Metres, radians, body frame +X forward/+Y left/+Z up.
Body-relative visual motion only; no contact dynamics, cable deformation or collision solver.
"""
from pathlib import Path
from functools import lru_cache
from copy import deepcopy
import argparse,json,tempfile,sys
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.optimize import least_squares
sys.path.insert(0,str(Path(__file__).resolve().parent))
from step_23_wing_kinematics import ROOT,WINGS,read_glb,write_glb,array,_append_accessor
from step_24_transmission import geometry,rig_source,transform
from step_16_geometry import optimize,stats
from step_01_chassis import create_chassis
from step_12_legs import create_leg
import step_17_metals,step_18_carbon,step_19_optics,step_20_wing_iridescence
SOURCE=ROOT/'models/dragonfly_abdomen_animated.glb'
NEUTRAL=ROOT/'models/dragonfly_animated.glb'
OUTPUT=ROOT/'models/dragonfly_legs_animated.glb'
LEGS=tuple(f'{i}_{s}' for i in (1,2,3) for s in ('left','right'))
JOINTS=('hip_yaw','hip_pitch','knee','ankle','tarsus_1','tarsus_2','tarsus_3')
LIMITS=np.deg2rad([20,25,35,55,15,15,15])

@lru_cache(maxsize=6)
def leg_geometry(leg):
    if leg not in LEGS:raise ValueError('Unknown leg')
    pair,side=leg.split('_');module=create_leg(int(pair),side)
    base=create_chassis().anchors['leg_'+leg]
    pivots=np.array([(base@module.anchors[j+'_pivot'])[:3,3] for j in JOINTS])
    foot=(base@module.anchors['foot'])[:3,3]
    sign=1 if side=='left' else -1
    return pivots,foot,np.array([[0,0,sign]]+[[sign,0,0]]*6,float)

def forward_kinematics(leg,angles):
    """Return joint positions, foot anchor, final orientation (body frame)."""
    q=np.asarray(angles,float)
    if q.shape!=(7,) or not np.isfinite(q).all():raise ValueError('Expected seven finite angles')
    p,foot,axes=leg_geometry(leg);r=Rotation.identity();prev=np.zeros(3);pos=np.zeros(3);out=[]
    for pivot,axis,a in zip(p,axes,q):
        pos=pos+r.apply(pivot-prev);out.append(pos.copy());r=r*Rotation.from_rotvec(axis*a);prev=pivot
    return np.array(out),pos+r.apply(foot-prev),r.as_quat()

def level_pose(q):
    a,b,c=q;return np.array([a,b,c,-b-c,0.,0.,0.])

def inverse_kinematics(leg,target,initial=None,tolerance=2e-5):
    """Analytic IK with bounded least-squares fallback; return angles, error, reachable."""
    target=np.asarray(target,float)
    if target.shape!=(3,) or not np.isfinite(target).all():raise ValueError('Expected finite 3D target')
    if not np.isfinite(tolerance) or tolerance<=0:raise ValueError('Positive tolerance required')
    q0=np.zeros(3) if initial is None else np.asarray(initial,float)[:3]
    if q0.shape!=(3,) or not np.isfinite(q0).all():raise ValueError('Invalid initial pose')
    # Solve yaw analytically, then planar two-link IK with level foot pitch.
    # Reflect right-side coordinates into the canonical left-leg frame.
    sign=1 if leg.endswith('left') else -1;mirror=np.array([1,sign,1]);p,f,_=leg_geometry(leg);p=p*mirror;f=f*mirror;goal=target*mirror
    d=goal-p[0];dx=(f-p[0])[0];rad=np.dot(d[:2],d[:2])-dx*dx
    if rad>=0:
        local_y=np.sqrt(rad);yaw=np.arctan2(d[1],d[0])-np.arctan2(local_y,dx)
        yaw=(yaw+np.pi)%(2*np.pi)-np.pi
        v1=(p[2]-p[1])[1:];v2=(p[3]-p[2])[1:];toe=(f-p[3])[1:]
        v=np.array([local_y,d[2]])-(p[1]-p[0])[1:]-toe
        l1,l2=np.linalg.norm(v1),np.linalg.norm(v2);a1=np.arctan2(v1[1],v1[0]);a2=np.arctan2(v2[1],v2[0])
        cosine=(np.dot(v,v)-l1*l1-l2*l2)/(2*l1*l2)
        if abs(cosine)<=1+1e-10:
            delta=np.sign(a2-a1)*np.arccos(np.clip(cosine,-1,1));hip=np.arctan2(v[1],v[0])-np.arctan2(l2*np.sin(delta),l1+l2*np.cos(delta))-a1;knee=delta-(a2-a1)
            q=level_pose([yaw,hip,knee]);err=float(np.linalg.norm(forward_kinematics(leg,q)[1]-target))
            if err<=tolerance and np.all(abs(q)<=LIMITS+1e-9):return q,err,True
    # Fallback: closest bounded pose for unreachable or alternate-branch targets.
    def residual(q):return (forward_kinematics(leg,level_pose(q))[1]-target)*1000
    bounds=np.deg2rad([20,23,30])
    res=least_squares(residual,np.clip(q0,-bounds,bounds),bounds=(-bounds,bounds),xtol=1e-10,ftol=1e-10,gtol=1e-10,max_nfev=60)
    q=level_pose(res.x);err=float(np.linalg.norm(forward_kinematics(leg,q)[1]-target))
    return q,err,bool(err<=tolerance and np.all(abs(q)<=LIMITS+1e-9))

def foot_target(leg,phase,mode='walk',stride=.002,lift=.0007):
    """Two alternating tripod groups; smooth stance and swing paths."""
    if not np.isfinite([phase,stride,lift]).all() or stride<0 or lift<0:raise ValueError('Invalid gait parameters')
    p=leg_geometry(leg)[1].copy()
    if mode=='idle':return p
    if mode in ('tuck','landing'):
        a=1. if mode=='tuck' else .5-.5*np.cos(2*np.pi*phase)
        return p+np.array([0.,(-1 if leg.endswith('left') else 1)*.0008*a,.0007*a])
    if mode!='walk':raise ValueError('Unknown leg mode')
    p[1]+=(-1 if leg.endswith('left') else 1)*.0007
    group=0 if leg in ('1_left','2_right','3_left') else 1;u=(phase+.5*group)%1
    if u<.5:
        s=2*u;p[0]+=stride*(.5-(3*s*s-2*s**3))
    else:
        s=2*u-1;p[0]+=stride*(-.5+3*s*s-2*s**3);p[2]+=lift*np.sin(np.pi*s)**2
    return p

def gait(leg,times,mode='walk',stride=.002,lift=.0007):
    t=np.asarray(times,float)
    if t.ndim!=1 or len(t)<2 or not np.isfinite(t).all() or np.any(np.diff(t)<=0):raise ValueError('Invalid timeline')
    poses=[];errors=[];q=None
    for u in (t-t[0])/(t[-1]-t[0]):
        q,e,ok=inverse_kinematics(leg,foot_target(leg,u,mode,stride,lift),q)
        if not ok:raise ValueError(f'Unreachable {leg} phase {u:.3f}: {e*1000:.3f} mm')
        poses.append(q);errors.append(e)
    p=np.array(poses)
    if not np.allclose(p[-1],p[0],atol=1e-5):raise ValueError('IK branch changed during cycle')
    p[-1]=p[0];return p,max(errors)

def group_for(part):
    """Assign neutral components to their nearest rigid mechanical link."""
    if part.startswith('fixed_'):return None
    if part.startswith('hip_yaw_'):return 'hip_yaw' if part.endswith('_rotor') else None
    if part.startswith('yaw_'):return 'hip_yaw'
    if part.startswith('hip_pitch_'):return 'hip_pitch' if part.endswith('_rotor') else 'hip_yaw'
    if part.startswith(('femur','spring_hook','hip_encoder')):return 'hip_pitch'
    if part.startswith('knee_'):return 'knee' if part.endswith('_rotor') else 'hip_pitch'
    if part.startswith('tibia'):return 'knee'
    if part.startswith('ankle_'):return 'ankle' if part.endswith('_rotor') else 'knee'
    if part.startswith('claw_'):return 'tarsus_3'
    if part.startswith('tarsus_'):
        i=int(part.split('_')[1]);return f'tarsus_{i}' if part.endswith('_rotor') else ('ankle' if i==1 else f'tarsus_{i-1}')
    if part=='leg_data_harness':return 'hip_pitch'
    if part.startswith('wire_clip_'):return ('hip_yaw','hip_pitch','hip_pitch','knee','knee')[int(part[-1])]
    raise ValueError('Unclassified leg part '+part)

def rig_legs(source,destination):
    doc,raw=read_glb(source);nodes=doc['nodes'];root=doc['scenes'][0]['nodes'][0];children=nodes[root]['children']
    for leg in LEGS:
        p,_,_=leg_geometry(leg);prefix='leg_'+leg+'/';parts=[i for i in children if nodes[i]['name'].startswith(prefix)]
        if len(parts)!=107:raise ValueError(f'Expected 107 parts: {leg}')
        parent=root;prev=np.zeros(3)
        for j,pivot in zip(JOINTS,p):
            selected=[i for i in parts if group_for(nodes[i]['name'][len(prefix):])==j]
            if not selected:raise ValueError('Empty joint '+j)
            for i in selected:
                old=np.asarray(nodes[i].get('matrix',np.eye(4).flatten(order='F'))).reshape(4,4,order='F');nodes[i]['matrix']=(np.linalg.inv(transform(pivot))@old).flatten(order='F').tolist();children.remove(i)
            idx=len(nodes);nodes.append(dict(name=f'rig_leg_{leg}_{j}',translation=(pivot-prev).tolist(),rotation=[0,0,0,1],children=selected));nodes[parent]['children'].append(idx);parent=idx;prev=pivot
    write_glb(destination,doc,bytearray(raw))

def copy_clips(ref,raw,doc,binary):
    names={n.get('name'):i for i,n in enumerate(doc['nodes'])};cache={};doc['animations']=[]
    for clip in ref['animations']:
        c=deepcopy(clip)
        for s in c['samplers']:
            for k in ('input','output'):
                idx=s[k]
                if idx not in cache:cache[idx]=_append_accessor(doc,binary,array(ref,raw,idx),ref['accessors'][idx]['type'])
                s[k]=cache[idx]
        for ch in c['channels']:ch['target']['node']=names[ref['nodes'][ch['target']['node']]['name']]
        doc['animations'].append(c)

def export(source=SOURCE,destination=OUTPUT,neutral=NEUTRAL,mode='walk',stride=.002,lift=.0007):
    if Path(destination).resolve() in (Path(source).resolve(),Path(neutral).resolve()):raise ValueError('Do not overwrite source')
    ref,raw=read_glb(source);reports={};gs={w:geometry(w) for w in WINGS}
    t=array(ref,raw,ref['animations'][0]['samplers'][0]['input']);poses={leg:gait(leg,t,mode,stride,lift) for leg in LEGS};print('Six leg paths solved',flush=True)
    with tempfile.TemporaryDirectory() as tmp:
        d=Path(tmp);p=[d/f'{i}.glb' for i in range(7)]
        rig_source(neutral,p[0],gs);rig_legs(p[0],p[1]);optimize(p[1],p[2],cache=False);print('Leg hierarchy rigged and batched',flush=True)
        step_17_metals.enrich(p[2],p[3],d/'metals');step_18_carbon.enrich(p[3],p[4],d/'carbon');step_19_optics.enrich(p[4],p[5]);step_20_wing_iridescence.enrich(p[5],p[6],d/'wings');doc,b=read_glb(p[6]);print('Materials reapplied',flush=True)
    binary=bytearray(b);copy_clips(ref,raw,doc,binary);names={n.get('name'):i for i,n in enumerate(doc['nodes'])}
    for clip in doc['animations']:
        ti=clip['samplers'][0]['input'];times=array(doc,binary,ti).copy()
        if not np.array_equal(t,times):raise ValueError('Expected common timeline')
        for leg,(q,e) in poses.items():
            axes=leg_geometry(leg)[2];rot=Rotation.from_rotvec((q[:,:,None]*axes).reshape(-1,3)).as_quat().reshape(len(t),7,4)
            for j,name in enumerate(JOINTS):
                clip['channels'].append({'sampler':len(clip['samplers']),'target':{'node':names[f'rig_leg_{leg}_{name}'],'path':'rotation'}})
                clip['samplers'].append({'input':ti,'output':_append_accessor(doc,binary,rot[:,j],'VEC4'),'interpolation':'LINEAR'})
        clip.setdefault('extras',{})['leg_control']={'mode':mode,'stride_m':stride,'lift_m':lift,'body_relative':True,'contact_dynamics':False};reports[clip['name']]=len(clip['channels'])
    destination=Path(destination);destination.parent.mkdir(parents=True,exist_ok=True);write_glb(destination,doc,binary)
    return {'stats':stats(doc),'channels':reports,'max_ik_error_mm':max(e for q,e in poses.values())*1000,'output':str(destination)}

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--source',type=Path,default=SOURCE);p.add_argument('--neutral',type=Path,default=NEUTRAL);p.add_argument('--output',dest='destination',type=Path,default=OUTPUT);p.add_argument('--mode',choices=['walk','landing','tuck','idle'],default='walk');p.add_argument('--stride',type=float,default=.002);p.add_argument('--lift',type=float,default=.0007);print(json.dumps(export(**vars(p.parse_args())),indent=2))
