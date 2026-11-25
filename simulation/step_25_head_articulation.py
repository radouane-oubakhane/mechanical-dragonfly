"""Two-axis head control in degrees, seconds and metres (+X forward, +Z up).
Software limits only: no collision solver or real flight attitude estimator.
"""
from dataclasses import dataclass
from pathlib import Path
import argparse, json
import numpy as np
from scipy.spatial.transform import Rotation
from step_23_wing_kinematics import ROOT, read_glb, write_glb, array, _append_accessor

SOURCE = ROOT / 'models/dragonfly_transmission_animated.glb'
OUTPUT = ROOT / 'models/dragonfly_head_animated.glb'
NAMES = ('rig_head_yaw', 'rig_head_pitch')

@dataclass(frozen=True)
class HeadLimits:
    yaw: float = 25.
    pitch: float = 18.
    yaw_rate: float = 75.
    pitch_rate: float = 60.

    def __post_init__(self):
        if not all(np.isfinite(v) and v > 0 for v in vars(self).values()):
            raise ValueError('Limits must be positive and finite')
        if self.yaw >= 90 or self.pitch >= 90:
            raise ValueError('Limits must be less than 90 degrees')

    @property
    def angles(self): return np.array([self.yaw, self.pitch])
    @property
    def rates(self): return np.array([self.yaw_rate, self.pitch_rate])

@dataclass(frozen=True)
class HeadRig:
    yaw_origin: tuple = (.03325, 0., 0.)
    pitch_offset: float = .0039

    def __post_init__(self):
        if np.shape(self.yaw_origin) != (3,) or not np.isfinite(self.yaw_origin).all() or not np.isfinite(self.pitch_offset) or self.pitch_offset < 0:
            raise ValueError('Invalid rig pivot')

    @classmethod
    def from_document(cls, doc):
        nodes = doc['nodes']; ids = {n.get('name'): i for i, n in enumerate(nodes)}
        y, p = (nodes[ids[n]] for n in NAMES)
        offset = np.asarray(p['translation'])
        if ids[NAMES[1]] not in y['children'] or not np.allclose(offset[1:], 0):
            raise ValueError('Expected longitudinal offset and nested pitch joint')
        return cls(tuple(y['translation']), float(offset[0]))


def aim_direction(direction, limits=HeadLimits()):
    """Body-frame unit or non-unit direction -> bounded yaw/elevation degrees."""
    d = np.asarray(direction, float)
    if d.shape[-1:] != (3,) or not np.isfinite(d).all() or np.any(np.linalg.norm(d,axis=-1)<1e-12):
        raise ValueError('Expected finite nonzero 3D direction')
    yaw = np.arctan2(d[...,1], d[...,0]); pitch = np.arctan2(d[...,2], np.hypot(d[...,0],d[...,1]))
    return np.clip(np.rad2deg(np.stack([yaw,pitch],axis=-1)), -limits.angles, limits.angles)


def aim_target(target, rig=HeadRig(), limits=HeadLimits()):
    """Aim from the pitch pivot, accounting for its offset from the yaw axis.
    Target is a body-frame point. Unreachable directions saturate at software limits.
    """
    d = np.asarray(target,float) - np.asarray(rig.yaw_origin)
    if d.shape[-1:] != (3,) or not np.isfinite(d).all() or np.any(np.linalg.norm(d,axis=-1) <= rig.pitch_offset+1e-9):
        raise ValueError('Target must be outside the pivot region')
    yaw = np.clip(np.arctan2(d[...,1],d[...,0]), -np.deg2rad(limits.yaw), np.deg2rad(limits.yaw))
    forward = d[...,0]*np.cos(yaw)+d[...,1]*np.sin(yaw)-rig.pitch_offset
    pitch = np.arctan2(d[...,2],forward)
    return np.clip(np.rad2deg(np.stack([yaw,pitch],axis=-1)), -limits.angles, limits.angles)


def stabilize(world_direction, body_attitude, limits=HeadLimits()):
    """Keep a world-space gaze direction given a body-to-world SciPy Rotation.
    This corrects gaze yaw/pitch only; roll cannot be controlled by this rig.
    """
    return aim_direction(body_attitude.inv().apply(world_direction), limits)


class HeadController:
    """Stateful exponential response plus hard angular slew-rate limits."""
    def __init__(self, limits=HeadLimits(), response=.12, initial=(0.,0.)):
        self.limits=limits; self.response=float(response); self.pose=np.asarray(initial,float).copy()
        if self.response<=0 or not np.isfinite(self.response) or self.pose.shape!=(2,) or not np.isfinite(self.pose).all() or np.any(abs(self.pose)>limits.angles):
            raise ValueError('Invalid controller configuration')

    def update(self, desired, dt):
        desired=np.asarray(desired,float)
        if desired.shape!=(2,) or not np.isfinite(desired).all() or not np.isfinite(dt) or dt<=0:
            raise ValueError('Expected finite desired angles and positive dt')
        desired=np.clip(desired,-self.limits.angles,self.limits.angles)
        delta=(desired-self.pose)*(-np.expm1(-dt/self.response))
        self.pose += np.clip(delta,-self.limits.rates*dt,self.limits.rates*dt)
        return self.pose.copy()


def profile(times, behavior, rig=HeadRig(), limits=HeadLimits(), target=None):
    """Periodic authored scan/idle/track or illustrative attitude compensation.
    The stabilization example assumes a virtual body attitude; it does not move the body.
    """
    t=np.asarray(times,float)
    if t.ndim!=1 or len(t)<2 or not np.isfinite(t).all() or np.any(np.diff(t)<=0):
        raise ValueError('Expected increasing times')
    phase=2*np.pi*(t-t[0])/(t[-1]-t[0]); s=np.sin(phase); c=np.cos(phase)
    if behavior=='scan': result=np.column_stack([18*s, 7*np.sin(phase+np.pi/4)])
    elif behavior=='idle': result=np.column_stack([5*s, 3*c])
    elif behavior=='track':
        point=np.column_stack([np.full_like(t,.24), .055*s, .026*c]) if target is None else np.broadcast_to(np.asarray(target,float),(len(t),3))
        result=aim_target(point,rig,limits)
    elif behavior=='stabilize':
        attitude=Rotation.from_euler('ZYX',np.column_stack([12*s, 8*c, 5*s]),degrees=True)
        result=stabilize([1.,0.,0.],attitude,limits)
    else: raise ValueError('Unknown behavior')
    result=np.clip(result,-limits.angles,limits.angles); result[-1]=result[0]
    if np.any(abs(np.diff(result,axis=0))/np.diff(t)[:,None]>limits.rates+1e-7):
        raise ValueError('Profile exceeds rate limits; use a slower timeline or smaller angles')
    return result


def head_rotations(pose):
    """glTF xyzw rotations, yaw +Z and elevation about -Y."""
    p=np.asarray(pose,float)
    if p.shape[-1:]!=(2,) or not np.isfinite(p).all(): raise ValueError('Invalid angles')
    zeros=np.zeros_like(p[...,0]); yaw=Rotation.from_rotvec(np.deg2rad(np.stack([zeros,zeros,p[...,0]],axis=-1)))
    pitch=Rotation.from_rotvec(np.deg2rad(np.stack([zeros,-p[...,1],zeros],axis=-1)))
    return yaw.as_quat(), pitch.as_quat()


def export(source=SOURCE,destination=OUTPUT,mode='auto',target=None,limits=HeadLimits()):
    """Replace only the two head rotation samplers in each existing clip."""
    if Path(source).resolve()==Path(destination).resolve(): raise ValueError('Do not overwrite source')
    if target is not None and mode!='track': raise ValueError('Target requires track mode')
    doc,raw=read_glb(source); binary=bytearray(raw); rig=HeadRig.from_document(doc)
    modes={'hover':'scan','forward':'stabilize','turn_left':'track'}; report={}
    for clip in doc['animations']:
        behavior=modes.get(clip['name'],'idle') if mode=='auto' else mode
        tracks={doc['nodes'][c['target']['node']].get('name'):c for c in clip['channels'] if c['target']['path']=='rotation'}
        # Use the existing dense wing timeline, not the original slower head timeline.
        timeidx=next(s['input'] for c in clip['channels'] if doc['nodes'][c['target']['node']].get('name')=='rig_wing_fore_left' for s in [clip['samplers'][c['sampler']]])
        t=array(doc,raw,timeidx); pose=profile(t,behavior,rig,limits,target)
        for name,q in zip(NAMES,head_rotations(pose)):
            channel=tracks[name]; channel['sampler']=len(clip['samplers'])
            clip['samplers'].append({'input':timeidx,'output':_append_accessor(doc,binary,q,'VEC4'),'interpolation':'LINEAR'})
        # Remove orphaned samplers; leave all untouched tracks' accessor indices intact.
        used=sorted({c['sampler'] for c in clip['channels']}); remap={old:new for new,old in enumerate(used)}
        clip['samplers']=[clip['samplers'][i] for i in used]
        for channel in clip['channels']: channel['sampler']=remap[channel['sampler']]
        clip.setdefault('extras',{})['head_control']={'behavior':behavior,'limits':vars(limits),'target_body_m':None if target is None else list(target),'stabilization':'virtual body attitude; no body motion' if behavior=='stabilize' else None}
        report[clip['name']]={'behavior':behavior,'channels':len(clip['channels']),'head_keyframes':len(t),'max_rates_deg_s':np.max(abs(np.diff(pose,axis=0))/np.diff(t)[:,None],axis=0).tolist()}
    destination=Path(destination);destination.parent.mkdir(parents=True,exist_ok=True);write_glb(destination,doc,binary)
    return report

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--source',type=Path,default=SOURCE);p.add_argument('--output',type=Path,default=OUTPUT);p.add_argument('--mode',choices=['auto','scan','idle','track','stabilize'],default='auto');p.add_argument('--target',type=float,nargs=3,help='Body-frame point in metres; requires --mode track')
    a=p.parse_args();print(json.dumps(export(a.source,a.output,a.mode,a.target),indent=2))
