"""Check constrained IK, mirrored gait, and preservation of all earlier clips."""
from pathlib import Path
import sys,hashlib
import numpy as np
from scipy.spatial.transform import Rotation
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'simulation'))
from step_27_leg_kinematics import *

def tracks(d,b,clip):
    return {(d['nodes'][c['target']['node']]['name'],c['target']['path']):tuple(array(d,b,clip['samplers'][c['sampler']][k]) for k in ('input','output')) for c in clip['channels']}

def test():
    times=np.linspace(0,2,25);max_error=0.
    for leg in LEGS:
        p,f,_=leg_geometry(leg);pos,foot,_=forward_kinematics(leg,np.zeros(7));assert np.allclose(p,pos) and np.allclose(f,foot)
        for mode in ('walk','landing','tuck','idle'):
            q,e=gait(leg,times,mode);max_error=max(e,max_error);assert e<2e-5;assert np.all(abs(q)<=LIMITS+1e-9);assert np.array_equal(q[0],q[-1])
            for i,row in enumerate(q):
                pos,foot,orientation=forward_kinematics(leg,row);assert np.allclose(np.linalg.norm(np.diff(pos,axis=0),axis=1),np.linalg.norm(np.diff(p,axis=0),axis=1));assert np.linalg.norm(foot-foot_target(leg,times[i]/2,mode))<2e-5
                assert np.allclose(Rotation.from_quat(orientation).apply([0,0,1]),[0,0,1],atol=1e-9)
        _,err,ok=inverse_kinematics(leg,[1,1,1]);assert not ok and err>.1
    for i in (1,2,3):
        a=foot_target(f'{i}_left',.25);b=foot_target(f'{i}_right',.75);assert np.allclose(a,b*[1,-1,1])
    a,ab=read_glb(SOURCE);d,b=read_glb(OUTPUT);assert stats(a)['triangles']==stats(d)['triangles']==504756;assert len(a['images'])==len(d['images'])==35;assert a['materials']==d['materials']
    def imhash(doc,raw):
        out=[]
        for im in doc['images']:
            v=doc['bufferViews'][im['bufferView']];off=v.get('byteOffset',0);out.append(hashlib.sha256(raw[off:off+v['byteLength']]).hexdigest())
        return sorted(out)
    assert imhash(a,ab)==imhash(d,b)
    names={n.get('name'):i for i,n in enumerate(d['nodes'])};nodes=d['nodes']
    for leg in LEGS:
        p,_,_=leg_geometry(leg);prev=np.zeros(3)
        for j,pivot in zip(JOINTS,p):
            name=f'rig_leg_{leg}_{j}';n=nodes[names[name]];assert np.allclose(n['translation'],pivot-prev);prev=pivot
        for j,k in zip(JOINTS,JOINTS[1:]):assert names[f'rig_leg_{leg}_{k}'] in nodes[names[f'rig_leg_{leg}_{j}']]['children']
    for old,new in zip(a['animations'],d['animations']):
        x,y=tracks(a,ab,old),tracks(d,b,new);assert len(x)==56 and len(y)==98 and x.keys()<=y.keys()
        for k in x:assert all(np.array_equal(u,v) for u,v in zip(x[k],y[k]))
        for leg in LEGS:
            _,_,axes=leg_geometry(leg);rows=[]
            for j,axis in zip(JOINTS,axes):
                t,v=y[(f'rig_leg_{leg}_{j}','rotation')];assert len(t)==241 and np.allclose(np.linalg.norm(v,axis=1),1,atol=1e-6);assert np.array_equal(v[0],v[-1]);rows.append(Rotation.from_quat(v).as_rotvec()@axis)
            q=np.array(rows).T;assert np.all(abs(q)<=LIMITS+1e-6)
            for i in range(0,len(t),12):assert np.linalg.norm(forward_kinematics(leg,q[i])[1]-foot_target(leg,float(t[i]/t[-1])))<2e-5
    print(f'PASS: six legs, four modes, IK reachability, mirrored paths, joint limits, foot pitch, rigid link lengths, 42 rig nodes, 56 exact prior tracks per clip, 98 tracks per clip, 504756 triangles, 35 texture hashes. Max sampled IK error: {max_error*1000:.8f} mm')
if __name__=='__main__':test()
