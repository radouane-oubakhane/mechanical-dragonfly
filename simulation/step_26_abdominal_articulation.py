"""Ten-joint abdominal controller. Angles in degrees, times in seconds, positions in metres.
The existing alternating Y/Z hinges are retained. This is kinematics, not dynamics.
"""
from dataclasses import dataclass
from pathlib import Path
import argparse,json
import numpy as np
from scipy.spatial.transform import Rotation
from step_23_wing_kinematics import ROOT,read_glb,write_glb,array,_append_accessor

SOURCE=ROOT/'models/dragonfly_head_animated.glb'
OUTPUT=ROOT/'models/dragonfly_abdomen_animated.glb'
NAMES=tuple(f'rig_abdomen_{i:02d}' for i in range(1,11))
AXES=np.array([[0,1,0] if i%2==0 else [0,0,1] for i in range(10)],float)

@dataclass(frozen=True)
class AbdomenLimits:
    angles: tuple=(3.,3.,4.,4.,5.,5.,6.,6.,7.,7.)
    rates: tuple=(30.,30.,35.,35.,40.,40.,45.,45.,50.,50.)
    def __post_init__(self):
        for name in ('angles','rates'):
            a=np.asarray(getattr(self,name),float)
            if a.shape!=(10,) or not np.isfinite(a).all() or np.any(a<=0):raise ValueError('Expected ten positive finite limits')
            if name=='angles' and np.any(a>=90):raise ValueError('Angles must be less than 90 degrees')
            object.__setattr__(self,name,tuple(a))

class AbdomenController:
    """Independent bounded joint commands with exponential smoothing and slew limits."""
    def __init__(self,limits=AbdomenLimits(),response=.15,initial=None):
        self.limits=limits;self.response=float(response)
        self.pose=np.zeros(10) if initial is None else np.array(initial,float,copy=True)
        if not np.isfinite(self.response) or self.response<=0 or self.pose.shape!=(10,) or not np.isfinite(self.pose).all() or np.any(abs(self.pose)>limits.angles):raise ValueError('Invalid controller configuration')
    def update(self,desired,dt):
        a=np.asarray(desired,float)
        if a.shape!=(10,) or not np.isfinite(a).all() or not np.isfinite(dt) or dt<=0:raise ValueError('Expected ten finite commands and positive dt')
        a=np.clip(a,-np.array(self.limits.angles),self.limits.angles)
        delta=(a-self.pose)*(-np.expm1(-dt/self.response));bound=np.array(self.limits.rates)*dt
        self.pose+=np.clip(delta,-bound,bound)
        return self.pose.copy()

def curvature_pose(vertical_deg=0.,lateral_deg=0.,limits=AbdomenLimits()):
    """Distribute total requested bend over the five joints for each axis.
    Positive Y rotation lifts the rearward (-X) abdomen; positive Z moves it right.
    Totals are joint-angle sums, not exact final orientation under combined bends.
    """
    values=np.asarray([vertical_deg,lateral_deg],float)
    if not np.isfinite(values).all():raise ValueError('Curvature must be finite')
    weights=np.linspace(.7,1.3,5);weights/=weights.sum()
    pose=np.empty(10);pose[::2]=values[0]*weights;pose[1::2]=values[1]*weights
    return np.clip(pose,-np.array(limits.angles),limits.angles)

def profile(times,mode='wave',limits=AbdomenLimits(),vertical=0.,lateral=0.):
    """Periodic authored poses, with a distal phase lag and increasing amplitude."""
    t=np.asarray(times,float)
    if t.ndim!=1 or len(t)<2 or not np.isfinite(t).all() or np.any(np.diff(t)<=0):raise ValueError('Expected strictly increasing times')
    phase=2*np.pi*(t-t[0])/(t[-1]-t[0]);i=np.arange(10);ramp=np.linspace(.45,1.,10)
    wave=np.sin(phase[:,None]-.42*i)
    if mode=='wave':p=wave*ramp*np.where(i%2==0,4.,3.)
    elif mode=='balance':p=wave*ramp*np.where(i%2==0,2.8,1.1)+curvature_pose(8,0,limits)
    elif mode=='turn':p=wave*ramp*np.where(i%2==0,1.5,2.5)+curvature_pose(3,-12,limits)
    elif mode=='idle':p=wave*ramp*.8
    elif mode=='posture':p=np.broadcast_to(curvature_pose(vertical,lateral,limits),(len(t),10)).copy()
    else:raise ValueError('Unknown abdomen mode')
    bound=np.array(limits.angles);p=np.clip(p,-bound,bound);p[-1]=p[0]
    if np.any(abs(np.diff(p,axis=0))/np.diff(t)[:,None]>np.array(limits.rates)+1e-7):raise ValueError('Profile exceeds joint rate limits; increase duration')
    return p

def joint_rotations(pose):
    """Return (...,10,4) local xyzw quaternions about existing hinge axes."""
    a=np.asarray(pose,float)
    if a.shape[-1:]!=(10,) or not np.isfinite(a).all():raise ValueError('Expected ten joint angles')
    v=np.deg2rad(a[...,None])*AXES
    return Rotation.from_rotvec(v.reshape(-1,3)).as_quat().reshape(a.shape+(4,))

def rig_offsets(doc):
    nodes=doc['nodes'];ids={n.get('name'):i for i,n in enumerate(nodes)}
    offsets=[]
    for i,name in enumerate(NAMES):
        n=nodes[ids[name]]
        if i and ids[name] not in nodes[ids[NAMES[i-1]]].get('children',[]):raise ValueError('Invalid abdomen chain')
        if not np.allclose(n.get('rotation',[0,0,0,1]),[0,0,0,1]):raise ValueError('Expected neutral rig rotations')
        offsets.append(n['translation'])
    a=np.array(offsets,float)
    if a.shape!=(10,3) or not np.isfinite(a).all() or not np.allclose(a[:,1:],0) or np.any(a[:,0]>=0):raise ValueError('Invalid pivot offsets')
    return a

def forward_kinematics(pose,offsets,tail_offset=(-.012,0,0)):
    """Return 10 world/body-frame pivot positions and a nominal tail attachment point."""
    q=joint_rotations(pose);offsets=np.asarray(offsets,float);tail=np.asarray(tail_offset,float)
    if q.shape!=(10,4) or offsets.shape!=(10,3) or tail.shape!=(3,) or not np.isfinite(offsets).all() or not np.isfinite(tail).all():raise ValueError('Expected single pose and valid offsets')
    p=np.zeros(3);r=Rotation.identity();out=[]
    for off,rot in zip(offsets,q):
        p=p+r.apply(off);out.append(p.copy());r=r*Rotation.from_quat(rot)
    return np.array(out),p+r.apply(tail)

def export(source=SOURCE,destination=OUTPUT,mode='auto',vertical=0.,lateral=0.,limits=AbdomenLimits()):
    """Replace only ten abdomen rotation tracks, preserving every other accessor."""
    if Path(source).resolve()==Path(destination).resolve():raise ValueError('Do not overwrite source')
    if mode!='posture' and (vertical!=0 or lateral!=0):raise ValueError('Bend commands require posture mode')
    doc,raw=read_glb(source);binary=bytearray(raw);offsets=rig_offsets(doc);report={}
    modes={'hover':'wave','forward':'balance','turn_left':'turn'}
    for clip in doc['animations']:
        behavior=modes.get(clip['name'],'idle') if mode=='auto' else mode
        tracks={doc['nodes'][c['target']['node']].get('name'):c for c in clip['channels'] if c['target']['path']=='rotation'}
        ti=clip['samplers'][tracks['rig_wing_fore_left']['sampler']]['input'];t=array(doc,raw,ti)
        p=profile(t,behavior,limits,vertical,lateral);q=joint_rotations(p)
        for j,name in enumerate(NAMES):
            c=tracks[name];c['sampler']=len(clip['samplers']);clip['samplers'].append({'input':ti,'output':_append_accessor(doc,binary,q[:,j],'VEC4'),'interpolation':'LINEAR'})
        used=sorted({c['sampler'] for c in clip['channels']});remap={old:new for new,old in enumerate(used)}
        clip['samplers']=[clip['samplers'][i] for i in used]
        for c in clip['channels']:c['sampler']=remap[c['sampler']]
        clip.setdefault('extras',{})['abdomen_control']={'behavior':behavior,'limits':vars(limits),'vertical_deg':vertical,'lateral_deg':lateral,'physical_validation':False}
        tips=np.array([forward_kinematics(row,offsets)[1] for row in p]);report[clip['name']]={'behavior':behavior,'channels':len(clip['channels']),'keyframes':len(t),'max_joint_deg':np.max(abs(p),axis=0).tolist(),'tail_range_mm':(np.ptp(tips,axis=0)*1000).tolist()}
    destination=Path(destination);destination.parent.mkdir(parents=True,exist_ok=True);write_glb(destination,doc,binary);return report

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--source',type=Path,default=SOURCE);p.add_argument('--output',type=Path,default=OUTPUT);p.add_argument('--mode',choices=['auto','wave','balance','turn','idle','posture'],default='auto');p.add_argument('--vertical',type=float,default=0.);p.add_argument('--lateral',type=float,default=0.);a=p.parse_args();print(json.dumps(export(a.source,a.output,a.mode,a.vertical,a.lateral),indent=2))
