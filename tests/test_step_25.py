"""Step 25: controller math and preservation of the Stage 24 asset."""
import sys,tempfile
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'simulation'))
from step_25_head_articulation import *
from step_16_geometry import stats
from step_24_transmission import pose_glb

def test():
    rig=HeadRig(); lim=HeadLimits()
    targets=np.array([[.25,.05,.02],[.3,-.06,-.04],[.25,0,0]])
    pose=aim_target(targets,rig); y,p=head_rotations(pose); yr=Rotation.from_quat(y); pr=Rotation.from_quat(p)
    origins=np.array(rig.yaw_origin)+yr.apply([rig.pitch_offset,0,0]); directions=targets-origins;directions/=np.linalg.norm(directions,axis=1)[:,None]
    assert np.allclose((yr*pr).apply([1,0,0]),directions)
    assert np.allclose(aim_direction([0,1,1]),[25,18]);assert np.allclose(aim_direction([0,-1,-1]),[-25,-18])
    attitude=Rotation.from_euler('ZYX',[12,-8,5],degrees=True);q=stabilize([1,0,0],attitude);y,p=head_rotations(q)
    assert np.allclose((attitude*Rotation.from_quat(y)*Rotation.from_quat(p)).apply([1,0,0]),[1,0,0])
    ctrl=HeadController();old=ctrl.pose.copy()
    for i in range(300):
        new=ctrl.update([90,-90],.01);assert np.all(abs(new-old)<=lim.rates*.01+1e-10);assert np.all(abs(new)<=lim.angles);old=new
    assert np.allclose(old,[25,-18],atol=1e-6)
    for f in [lambda:HeadLimits(yaw=-1),lambda:aim_direction([0,0,0]),lambda:aim_target(rig.yaw_origin),lambda:ctrl.update([1,2],0),lambda:profile(np.linspace(0,.1,10),'scan',limits=HeadLimits(yaw_rate=.1))]:
        try:f()
        except ValueError:pass
        else:raise AssertionError('Invalid input accepted')
    a,ab=read_glb(SOURCE);b,bb=read_glb(OUTPUT)
    assert bb[:len(ab)]==ab
    for key in ('nodes','meshes','materials','images','textures','scenes'):
        assert a[key]==b[key],key
    assert stats(a)['triangles']==stats(b)['triangles']==504756
    assert len(b['images'])==35
    def tracks(doc,raw,clip):
        return {(doc['nodes'][c['target']['node']]['name'],c['target']['path']):(array(doc,raw,clip['samplers'][c['sampler']]['input']),array(doc,raw,clip['samplers'][c['sampler']]['output'])) for c in clip['channels']}
    for old,new in zip(a['animations'],b['animations']):
        x,y=tracks(a,ab,old),tracks(b,bb,new);assert len(x)==len(y)==56 and x.keys()==y.keys()
        for key,(t,v) in x.items():
            if key[0] not in NAMES:assert np.array_equal(t,y[key][0]) and np.array_equal(v,y[key][1])
        ty,qy=y[(NAMES[0],'rotation')];tp,qp=y[(NAMES[1],'rotation')];assert np.array_equal(ty,tp) and len(ty)==241
        expected=profile(ty,new['extras']['head_control']['behavior'],HeadRig.from_document(b));yy,pp=head_rotations(expected)
        assert np.allclose(qy,yy,atol=1e-7) and np.allclose(qp,pp,atol=1e-7)
        for j,q in enumerate((qy,qp)):
            assert np.allclose(np.linalg.norm(q,axis=1),1,atol=1e-6) and np.allclose(q[0],q[-1]);rate=np.rad2deg((Rotation.from_quat(q[:-1]).inv()*Rotation.from_quat(q[1:])).magnitude())/np.diff(ty);assert max(rate)<=lim.rates[j]+.001
    with tempfile.TemporaryDirectory() as td:
        path=Path(td)/'target.glb';export(SOURCE,path,'track',[.25,.04,.02]);d,r=read_glb(path);assert all(x['extras']['head_control']['behavior']=='track' for x in d['animations'])
        p=Path(td)/'pose.glb';pose_glb(OUTPUT,p,'hover',.5);d,r=read_glb(p);assert 'animations' not in d
    print('PASS: target aim, saturation, world gaze stabilization, rate-limited controller, invalid inputs, exact source BIN prefix and mesh/material/node preservation, 54 unchanged tracks per clip, 2 bounded periodic head tracks, custom target and pose export')
if __name__=='__main__':test()
