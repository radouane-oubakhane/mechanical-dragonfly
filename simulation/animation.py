"""Build a hierarchical glTF animation from the final neutral-pose GLB.

Rigid-part demonstration: no physics, cable deformation or collision solver.
The input model is unchanged. The exported clip can be played in glTF viewers.
"""
from pathlib import Path
import argparse
import json
import struct
import sys
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'components'))
from step_01_chassis import create_chassis
from step_04_transmission import create_transmission
from step_05_actuators import create_wing_actuator, CHANNELS
from step_06_wing_joints import create_wing_joint
from step_07_head import create_head
from step_10_abdominal_spine import create_abdominal_segment


def read_glb(path):
    raw=Path(path).read_bytes(); magic,version,size=struct.unpack_from('<4sII',raw)
    if magic!=b'glTF' or version!=2 or size!=len(raw): raise ValueError('expected GLB 2.0')
    n,kind=struct.unpack_from('<I4s',raw,12)
    if kind!=b'JSON': raise ValueError('missing JSON chunk')
    doc=json.loads(raw[20:20+n]); off=20+n; length,kind=struct.unpack_from('<I4s',raw,off)
    if kind!=b'BIN\x00': raise ValueError('missing BIN chunk')
    return doc,bytearray(raw[off+8:off+8+length])


def write_glb(path,doc,binary):
    while len(binary)%4: binary.append(0)
    doc['buffers'][0]['byteLength']=len(binary)
    data=json.dumps(doc,separators=(',',':')).encode();data+=b' '*((-len(data))%4)
    raw=struct.pack('<4sII',b'glTF',2,12+8+len(data)+8+len(binary))+struct.pack('<I4s',len(data),b'JSON')+data+struct.pack('<I4s',len(binary),b'BIN\x00')+binary
    Path(path).write_bytes(raw)


def rig_specs(scale=1.0):
    """Ordered (name,parent,pivot,axis,selector,amplitude,phase,frequency) records.
    Pivot and axis are expressed in the original neutral world frame.
    """
    chassis=create_chassis(scale); trans=create_transmission(scale); t=chassis.anchors['transmission']
    specs=[]
    for side,pair in CHANNELS:
        a=create_wing_actuator(side,pair,scale);j=create_wing_joint(side,pair,scale)
        p=t@trans.anchors[a.name]@a.anchors['output']@j.anchors['flap_pivot']
        k=f'{pair}_{side}'; prefixes=(f'wing_skeleton_{k}/',f'wing_membrane_{k}/',f'wing_joint_{k}/rotor_')
        specs.append((f'rig_wing_{k}',None,p[:3,3],p[:3,1],lambda n,pr=prefixes:n.startswith(pr), 12,0 if pair=='fore' else np.pi/3,2.0))
    head=create_head(scale); hp=chassis.anchors['neck']
    yaw=hp@head.anchors['yaw_pivot']; pitch=hp@head.anchors['pitch_pivot']
    specs.append(('rig_head_yaw',None,yaw[:3,3],yaw[:3,0],lambda n:n.startswith('head/yaw_'),5,0,.5))
    specs.append(('rig_head_pitch','rig_head_yaw',pitch[:3,3],pitch[:3,0],lambda n:n.startswith(('head/pitch_','eye_left/','eye_right/','facial_armor/')),3,0,.5))
    placement=chassis.anchors['abdomen']; parent=None
    for i in range(1,11):
        seg=create_abdominal_segment(i,scale); p=placement@seg.anchors['pivot']; name=f'rig_abdomen_{i:02d}'
        prefix=f'abdominal_segment_{i:02d}/'; armor=f'abdominal_armor_{i:02d}/'
        def select(n,prefix=prefix,armor=armor,i=i):
            return n.startswith(armor) or (n.startswith(prefix) and not n.startswith(prefix+'chassis_mount') and not n.startswith(prefix+'chassis_adapter')) or (i==10 and n.startswith('tail/'))
        specs.append((name,parent,p[:3,3],p[:3,2],select,1.5,0.25*(i-1),.5))
        parent=name;placement=placement@seg.anchors['next']
    return specs


def export_animation(source,destination,scale=1.,duration=2.,fps=60):
    """Export a glTF clip with 16 independently animated rigid joint nodes."""
    if not np.isfinite(duration) or duration<=0 or not isinstance(fps,int) or fps<2: raise ValueError('positive duration and integer fps >= 2 required')
    doc,binary=read_glb(source);nodes=doc['nodes'];root=doc['scenes'][doc.get('scene',0)]['nodes'][0]
    original=list(nodes[root]['children']); remaining=set(original);rig={};pivots={}; channels=[];samplers=[]
    def accessor(a,typ):
        a=np.asarray(a,dtype='<f4');offset=len(binary);binary.extend(a.tobytes());view=len(doc['bufferViews']);doc['bufferViews'].append({'buffer':0,'byteOffset':offset,'byteLength':a.nbytes})
        acc={'bufferView':view,'componentType':5126,'count':len(a),'type':typ}
        if typ=='SCALAR':acc.update(min=[float(a.min())],max=[float(a.max())])
        doc['accessors'].append(acc);return len(doc['accessors'])-1
    times=np.linspace(0,duration,round(duration*fps)+1,dtype=np.float32);tidx=accessor(times,'SCALAR')
    specs=rig_specs(scale)
    for name,parent,p,axis,select,amp,phase,hz in specs:
        selected=[i for i in original if i in remaining and select(nodes[i]['name'])]
        if not selected: raise ValueError(f'empty rig group {name}')
        # Each mesh moves beneath a translated pivot; its rest-world pose is preserved.
        for i in selected:
            n=nodes[i];m=np.asarray(n.get('matrix',np.eye(4).flatten(order='F')),float).reshape((4,4),order='F');m[:3,3]-=p;n['matrix']=m.flatten(order='F').tolist();remaining.remove(i)
        pid=root if parent is None else rig[parent];delta=p-(np.zeros(3) if parent is None else pivots[parent]);idx=len(nodes)
        nodes.append({'name':name,'translation':delta.tolist(),'rotation':[0,0,0,1],'children':selected})
        if parent is not None:nodes[pid]['children'].append(idx)
        rig[name]=idx;pivots[name]=p
        # Periodic demonstration, begins at zero displacement even with phase lag.
        theta=np.deg2rad(amp)*(np.sin(2*np.pi*hz*times+phase)-np.sin(phase))*.5
        axis=axis/np.linalg.norm(axis);q=np.column_stack((np.sin(theta/2)[:,None]*axis,np.cos(theta/2)))
        samplers.append({'input':tidx,'output':accessor(q,'VEC4'),'interpolation':'LINEAR'});channels.append({'sampler':len(samplers)-1,'target':{'node':idx,'path':'rotation'}})
    nodes[root]['children']=[i for i in original if i in remaining]+[rig[n] for n,parent,*_ in specs if parent is None]
    doc['animations']=[{'name':'Mechanical motion demonstration','samplers':samplers,'channels':channels}]
    write_glb(destination,doc,binary);return len(channels)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source',type=Path,default=ROOT/'models'/'step_15_assembled.glb')
    parser.add_argument('--output',type=Path,default=ROOT/'models'/'dragonfly_animated.glb')
    parser.add_argument('--scale',type=float,default=1.0,help='must match input GLB scale')
    parser.add_argument('--duration',type=float,default=2.0)
    args=parser.parse_args();count=export_animation(args.source,args.output,args.scale,args.duration)
    print(f'{count} animated joint nodes | {args.output}')
