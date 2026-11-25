"""Validate transmission animation and retained Stage 23 clips."""
from pathlib import Path
import sys,tempfile
import numpy as np
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'simulation'))
from step_24_transmission import *

def test():
    old,ob=read_glb(PROFILES);new,nb=read_glb(OUTPUT)
    assert stats(old)['triangles']==stats(new)['triangles']==504756
    assert len(new['images'])==len(old['images'])==35
    assert new['materials']==old['materials']
    for d,b in [(old,ob),(new,nb)]:
        assert len(d['animations'])==3
    for i,a in enumerate(new['animations']):
        assert a['name']==old['animations'][i]['name'];assert len(a['channels'])==56
        def tracks(d,b,clip):
            return {(d['nodes'][c['target']['node']]['name'],c['target']['path']):(array(d,b,clip['samplers'][c['sampler']]['input']),array(d,b,clip['samplers'][c['sampler']]['output'])) for c in clip['channels']}
        x=tracks(old,ob,old['animations'][i]);y=tracks(new,nb,a);assert len(y)==56
        for k,(t,v) in x.items():
            assert np.array_equal(t,y[k][0]);assert np.allclose(v,y[k][1],atol=1e-7)
        for (name,path),(t,v) in y.items():
            assert np.isfinite(v).all() and np.all(np.diff(t)>0)
            if path=='rotation':
                assert np.allclose(np.linalg.norm(v,axis=1),1,atol=1e-6);assert abs(v[0]@v[-1])>1-1e-6
            else:assert np.allclose(v[0],v[-1]);assert path!='scale' or np.all(v>0)
        for w in WINGS:
            g=geometry(w);c=WingControl(**a['extras']['wing_controls'][w]);t=y[('rig_drive_'+w+'_output','rotation')][0];p=drive_pose(w,c,t,g)
            driver=Rotation.from_quat(p['driver']['rotation']);output=Rotation.from_quat(p['output']['rotation']);assert np.max((driver*output).magnitude())<1e-8
            rock=Rotation.from_quat(p['rocker']['rotation']);lower=g['rocker']+rock.apply(g['lower']-g['rocker']);upper=g['rocker']+rock.apply(g['upper']-g['rocker']);crank=g['output']+output.apply(g['crank']-g['output'])
            assert np.allclose(p['clevis']['translation'],lower)
            for role,start,end,L in [('piston',g['rod_start'],lower,np.linalg.norm(g['lower']-g['rod_start'])),('link',crank,upper,np.linalg.norm(g['upper']-g['crank']))]:
                v=p[role];tip=v['translation']+Rotation.from_quat(v['rotation']).apply(np.c_[L*v['scale'][:,0],np.zeros((len(t),2))]);assert np.allclose(tip,end,atol=1e-9);assert np.allclose(v['translation'],start)
                assert np.allclose(y[('rig_drive_'+w+'_'+role,'scale')][1],v['scale'],atol=1e-6)
    with tempfile.TemporaryDirectory() as temp:
        path=Path(temp)/'pose.glb';pose_glb(OUTPUT,path,time=.12);d,b=read_glb(path);assert 'animations' not in d and stats(d)['triangles']==504756
    print('PASS: geometry count, 35 images, material definitions, all original tracks, 56 channels per clip, periodicity, gear counter-rotation, rocker and link endpoints, pose export')
if __name__=='__main__':test()
