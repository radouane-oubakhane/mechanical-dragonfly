"""Independent four-wing kinematics and glTF clip export (rigid visual model).
All angles are degrees; all time values are seconds. No aerodynamic or collision solver.
"""
from dataclasses import dataclass, replace
from pathlib import Path
import argparse, json, sys
import numpy as np
from scipy.spatial.transform import Rotation, Slerp
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'optimization'))
from step_16_geometry import read_glb, write_glb, array

WINGS = ('fore_left', 'fore_right', 'hind_left', 'hind_right')
FLAP_LIMIT, PITCH_LIMIT = 35., 15.  # software limits, not certified clearances
SOURCE = ROOT/'models/dragonfly_animated_balanced_iridescent.glb'
OUTPUT = ROOT/'models/dragonfly_wing_kinematics.glb'

@dataclass(frozen=True)
class WingControl:
    frequency: float = 3.
    flap: float = 25.
    phase: float = 0.
    pitch: float = 10.
    flap_bias: float = 0.
    pitch_bias: float = 0.
    pitch_phase: float = 90.

    def __post_init__(self):
        if not all(np.isfinite(v) for v in vars(self).values()):
            raise ValueError('Wing parameters must be finite')
        if self.frequency <= 0 or self.flap < 0 or self.pitch < 0:
            raise ValueError('Positive frequency and nonnegative amplitudes required')
        if abs(self.flap_bias)+self.flap > FLAP_LIMIT or abs(self.pitch_bias)+self.pitch > PITCH_LIMIT:
            raise ValueError('Wing angle exceeds software limit')


def make_profile(name):
    """Return independent controls; phase lag is expressed in degrees."""
    if name not in ('hover','forward','turn_left'):
        raise ValueError('Unknown profile')
    controls = {}
    for wing in WINGS:
        hind = wing.startswith('hind')
        phase = -90. if hind else 0.
        if name == 'hover':
            c = WingControl(3., 28. if hind else 30., phase, 10.)
        elif name == 'forward':
            c = WingControl(4., 22. if hind else 25., -60. if hind else 0., 9., 4., -3.)
        else:
            left = wing.endswith('left')
            c = WingControl(3., 19. if left else 30., phase, 8. if left else 12.)
        controls[wing] = c
    return controls


def angles(control, t):
    """Return flap and feather pitch angles, with continuous periodic motion."""
    t = np.asarray(t, dtype=float)
    if not np.isfinite(t).all(): raise ValueError('Time must be finite')
    p = 2*np.pi*control.frequency*t + np.deg2rad(control.phase)
    return (control.flap_bias + control.flap*np.sin(p),
            control.pitch_bias + control.pitch*np.sin(p+np.deg2rad(control.pitch_phase)))


def wing_rotation(wing, control, t):
    """Body axes: +X head, +Y left, +Z up. Flap then local feather pitch.
    A positive flap raises either wing. Positive pitch raises the forward edge.
    The pitch axis is virtual: the existing rotor assembly moves with the wing.
    Returns glTF xyzw unit quaternions.
    """
    if wing not in WINGS: raise ValueError('Unknown wing')
    f,p = angles(control,t)
    sign = 1 if wing.endswith('left') else -1
    zeros=np.zeros_like(f)
    flap=Rotation.from_rotvec(np.deg2rad(np.stack([sign*f,zeros,zeros],axis=-1)))
    pitch=Rotation.from_rotvec(np.deg2rad(np.stack([zeros,-p,zeros],axis=-1)))
    return (flap*pitch).as_quat()


def timeline(controls, duration=2., fps=120):
    """Uniform sampling; reject undersampled profiles and incomplete cycles."""
    if set(controls)!=set(WINGS): raise ValueError('Exactly four named wing controls required')
    if not np.isfinite(duration) or duration<=0 or not isinstance(fps,int) or fps<2:
        raise ValueError('Positive duration and integer fps required')
    if not np.isclose(duration*fps,round(duration*fps),rtol=0,atol=1e-8):
        raise ValueError('Duration must span an integer number of frames')
    for c in controls.values():
        if fps < 12*c.frequency: raise ValueError('Use at least 12 samples per flap cycle')
        if not np.isclose(duration*c.frequency,round(duration*c.frequency),rtol=0,atol=1e-8):
            raise ValueError('Duration must span whole wing cycles for a seamless loop')
    if round(duration*fps)>100000: raise ValueError('Timeline too long')
    return np.linspace(0,duration,round(duration*fps)+1,dtype=np.float32)


def _append_accessor(doc, binary, values, kind):
    a=np.asarray(values,dtype='<f4'); binary.extend(b'\0'*(-len(binary)%4)); offset=len(binary)
    binary.extend(a.tobytes()); vi=len(doc['bufferViews'])
    doc['bufferViews'].append({'buffer':0,'byteOffset':offset,'byteLength':a.nbytes})
    ac={'bufferView':vi,'componentType':5126,'count':len(a),'type':kind}
    if kind=='SCALAR': ac.update(min=[float(a.min())],max=[float(a.max())])
    doc['accessors'].append(ac);return len(doc['accessors'])-1


def export_profiles(source=SOURCE,destination=OUTPUT,profiles=None,duration=2.,fps=120):
    """Export multiple named clips; all original geometry/material bytes remain.
    The 12 nonwing tracks are reused unchanged for the original two-second length;
    other durations are allowed only in whole periods of the original body clip.
    """
    if Path(source).resolve()==Path(destination).resolve(): raise ValueError('Do not overwrite source')
    if profiles is None: profiles={n:make_profile(n) for n in ('hover','forward','turn_left')}
    if not profiles: raise ValueError('At least one profile required')
    doc,raw=read_glb(source); binary=bytearray(raw)
    original=doc['animations'][0]; nodes=doc['nodes']
    by_name={nodes[c['target']['node']]['name']:c for c in original['channels']}
    if not all('rig_wing_'+w in by_name for w in WINGS): raise ValueError('Missing wing rig')
    body=[c for c in original['channels'] if not nodes[c['target']['node']]['name'].startswith('rig_wing_')]
    if len(body)!=12: raise ValueError('Expected twelve body tracks')
    clips=[]
    for name,controls in profiles.items():
        times=timeline(controls,duration,fps); tidx=_append_accessor(doc,binary,times,'SCALAR'); samplers=[];channels=[]
        def add(target,sampler):
            channels.append({'sampler':len(samplers),'target':dict(target)}); samplers.append(sampler)
        for wing in WINGS:
            q=wing_rotation(wing,controls[wing],times);q[-1]=q[0]
            add(by_name['rig_wing_'+wing]['target'],{'input':tidx,'output':_append_accessor(doc,binary,q,'VEC4'),'interpolation':'LINEAR'})
        for c in body:
            s=original['samplers'][c['sampler']]; old_t=array(doc,raw,s['input']);period=float(old_t[-1]-old_t[0])
            if np.isclose(duration,period): add(c['target'],dict(s));continue
            if not np.isclose(duration/period,round(duration/period)): raise ValueError('Duration must be whole body-motion periods')
            old_q=array(doc,raw,s['output']);q=Slerp(old_t,Rotation.from_quat(old_q))(np.mod(times,period)+old_t[0]).as_quat();q[-1]=q[0]
            add(c['target'],{'input':tidx,'output':_append_accessor(doc,binary,q,'VEC4'),'interpolation':'LINEAR'})
        clips.append({'name':name,'samplers':samplers,'channels':channels,'extras':{'wing_controls':{w:vars(c) for w,c in controls.items()},'software_limits_deg':[FLAP_LIMIT,PITCH_LIMIT]}})
    doc['animations']=clips
    destination=Path(destination);destination.parent.mkdir(parents=True,exist_ok=True);write_glb(destination,doc,binary)
    return {'clips':len(clips),'channels_per_clip':len(clips[0]['channels']),'duration':duration,'fps':fps,'output':str(destination)}


def pose_glb(source,destination,clip='hover',time=0.):
    """Bake one animation pose into a separate static GLB for the studio renderer."""
    if Path(source).resolve()==Path(destination).resolve(): raise ValueError('Do not overwrite source')
    doc,raw=read_glb(source);a=next((a for a in doc['animations'] if a['name']==clip),None)
    if a is None or not np.isfinite(time): raise ValueError('Invalid clip or time')
    for c in a['channels']:
        s=a['samplers'][c['sampler']];t=array(doc,raw,s['input']);q=array(doc,raw,s['output']);value=float(np.clip(time,t[0],t[-1]))
        node=doc['nodes'][c['target']['node']];node['rotation']=Slerp(t,Rotation.from_quat(q))(value).as_quat().tolist()
    doc.pop('animations');write_glb(destination,doc,bytearray(raw))

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--source',type=Path,default=SOURCE);p.add_argument('--output',type=Path,default=OUTPUT)
    p.add_argument('--profile',choices=['all','hover','forward','turn_left'],default='all');p.add_argument('--frequency',type=float);p.add_argument('--duration',type=float,default=2.);p.add_argument('--fps',type=int,default=120)
    a=p.parse_args();names=('hover','forward','turn_left') if a.profile=='all' else (a.profile,); profiles={n:make_profile(n) for n in names}
    if a.frequency is not None:profiles={n:{w:replace(c,frequency=a.frequency) for w,c in controls.items()} for n,controls in profiles.items()}
    print(json.dumps(export_profiles(a.source,a.output,profiles,a.duration,a.fps),indent=2))
