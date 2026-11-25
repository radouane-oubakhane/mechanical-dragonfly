"""Test bounded abdomen motion, rigid chain FK, and exact prior-asset preservation."""
from pathlib import Path
import sys,tempfile
import numpy as np
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'simulation'))
from step_26_abdominal_articulation import *
from step_16_geometry import stats
from step_24_transmission import pose_glb

def tracks(doc,raw,clip):
    return {(doc['nodes'][c['target']['node']]['name'],c['target']['path']):tuple(array(doc,raw,clip['samplers'][c['sampler']][k]) for k in ('input','output')) for c in clip['channels']}

def test():
    lim=AbdomenLimits();ctrl=AbdomenController();old=ctrl.pose.copy()
    for _ in range(300):
        new=ctrl.update(np.full(10,90.),.01);assert np.all(abs(new-old)<=np.array(lim.rates)*.01+1e-9);assert np.all(abs(new)<=lim.angles);old=new
    assert np.allclose(old,lim.angles,atol=1e-6)
    p=curvature_pose(10,-8);assert np.allclose([sum(p[::2]),sum(p[1::2])],[10,-8])
    for f in [lambda:AbdomenLimits(angles=(1,)),lambda:ctrl.update([1]*10,0),lambda:profile([0,0]),lambda:profile([0,.01,.02,.03],'wave'),lambda:joint_rotations([1]),lambda:curvature_pose(float('nan'))]:
        try:f()
        except ValueError:pass
        else:raise AssertionError('Invalid input accepted')
    a,ab=read_glb(SOURCE);b,bb=read_glb(OUTPUT);assert bb[:len(ab)]==ab
    for key in ('nodes','meshes','materials','images','textures','scenes'):assert a[key]==b[key],key
    assert stats(a)['triangles']==stats(b)['triangles']==504756 and len(b['images'])==35
    off=rig_offsets(b);pos,tip=forward_kinematics(np.zeros(10),off);assert np.allclose(pos,np.cumsum(off,axis=0));assert np.allclose(tip,pos[-1]+[-.012,0,0])
    for v,l,axis,sign in [(10,0,2,1),(0,10,1,-1)]:
        pos,tip=forward_kinematics(curvature_pose(v,l),off);assert sign*tip[axis]>0
        assert np.allclose(np.linalg.norm(np.diff(pos,axis=0),axis=1),np.linalg.norm(off[1:],axis=1))
    for old,new in zip(a['animations'],b['animations']):
        x,y=tracks(a,ab,old),tracks(b,bb,new);assert len(x)==len(y)==56 and x.keys()==y.keys()
        for key,(t,v) in x.items():
            if key[0] not in NAMES:assert np.array_equal(t,y[key][0]) and np.array_equal(v,y[key][1])
        ts=y[(NAMES[0],'rotation')][0];assert len(ts)==241
        angles=profile(ts,new['extras']['abdomen_control']['behavior']);q=joint_rotations(angles)
        for j,name in enumerate(NAMES):
            t,v=y[(name,'rotation')];assert np.array_equal(ts,t) and np.allclose(v,q[:,j],atol=1e-7);assert np.allclose(np.linalg.norm(v,axis=1),1,atol=1e-6) and np.array_equal(v[0],v[-1]);r=Rotation.from_quat(v);assert np.allclose(r.as_rotvec(),np.deg2rad(angles[:,j,None])*AXES[j],atol=1e-7);assert max(np.rad2deg((r[:-1].inv()*r[1:]).magnitude())/np.diff(t))<=lim.rates[j]+.001
        for p in angles[::20]:
            pos,tip=forward_kinematics(p,off);assert np.allclose(np.linalg.norm(np.diff(pos,axis=0),axis=1),np.linalg.norm(off[1:],axis=1))
    with tempfile.TemporaryDirectory() as td:
        out=Path(td)/'posture.glb';export(SOURCE,out,'posture',12,-10);d,r=read_glb(out);assert all(c['extras']['abdomen_control']['behavior']=='posture' for c in d['animations']);p=Path(td)/'pose.glb';pose_glb(out,p,'hover',.5);d,r=read_glb(p);assert 'animations' not in d
    print('PASS: 10 bounded joints, alternating axes, slew limits, rigid segment distances, posture and periodic profiles, 46 unchanged tracks per clip, exact source BIN prefix, geometry/material preservation, custom posture and pose export')
if __name__=='__main__':test()
