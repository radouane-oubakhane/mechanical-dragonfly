import sys, unittest, tempfile
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'simulation'))
from step_23_wing_kinematics import *

class WingTests(unittest.TestCase):
    def test_limits_and_sampling(self):
        for name in ('hover','forward','turn_left'):
            c=make_profile(name);t=timeline(c)
            self.assertEqual(len(t),241)
            for w,k in c.items():
                f,p=angles(k,t);self.assertLessEqual(abs(f).max(),FLAP_LIMIT+1e-8);self.assertLessEqual(abs(p).max(),PITCH_LIMIT+1e-8)
                q=wing_rotation(w,k,t);np.testing.assert_allclose(np.linalg.norm(q,axis=1),1,atol=1e-10);np.testing.assert_allclose(q[0],q[-1],atol=1e-10)
        with self.assertRaises(ValueError): WingControl(flap=36)
        with self.assertRaises(ValueError): WingControl(pitch=16)
        with self.assertRaises(ValueError): timeline(make_profile('hover'),fps=30)
        with self.assertRaises(ValueError): timeline(make_profile('hover'),duration=1.5)
        with self.assertRaises(ValueError): WingControl(frequency=float('nan'))
    def test_phase_symmetry_and_controls(self):
        c=make_profile('hover');self.assertEqual(c['hind_left'].phase-c['fore_left'].phase,-90)
        t=np.linspace(0,2,241);reflection=np.diag([1,-1,1])
        for pair in ('fore','hind'):
            l,r=pair+'_left',pair+'_right';ql=Rotation.from_quat(wing_rotation(l,c[l],t)).as_matrix();qr=Rotation.from_quat(wing_rotation(r,c[r],t)).as_matrix()
            np.testing.assert_allclose(qr,reflection@ql@reflection,atol=1e-12)
        turn=make_profile('turn_left');self.assertLess(turn['fore_left'].flap,turn['fore_right'].flap)
        custom=dict(c);custom['fore_left']=WingControl(5,20,30,5);t=timeline(custom);self.assertFalse(np.allclose(wing_rotation('fore_left',custom['fore_left'],t),wing_rotation('fore_left',c['fore_left'],t)))
    def test_export_preservation(self):
        d,b=read_glb(SOURCE);e,bb=read_glb(OUTPUT)
        self.assertEqual(bb[:len(b)],b)
        for k in ('nodes','meshes','materials','textures','images','scenes'):
            self.assertEqual(d[k],e[k])
        self.assertEqual(e['accessors'][:len(d['accessors'])],d['accessors']);self.assertEqual(e['bufferViews'][:len(d['bufferViews'])],d['bufferViews'])
        self.assertEqual([a['name'] for a in e['animations']],['hover','forward','turn_left'])
        old=d['animations'][0]; old_body={c['target']['node']:old['samplers'][c['sampler']] for c in old['channels'] if not d['nodes'][c['target']['node']]['name'].startswith('rig_wing_')}
        for a in e['animations']:
            self.assertEqual(len(a['channels']),16);self.assertEqual(len(set(c['target']['node'] for c in a['channels'])),16)
            for c in a['channels']:
                s=a['samplers'][c['sampler']]; n=c['target']['node'];t=array(e,bb,s['input']);q=array(e,bb,s['output']);self.assertTrue(np.all(np.diff(t)>0));np.testing.assert_allclose(np.linalg.norm(q,axis=1),1,atol=1e-6)
                np.testing.assert_allclose(q[0],q[-1],atol=1e-6)
                if n in old_body:self.assertEqual(s,old_body[n])
                else:
                    w=e['nodes'][n]['name'].removeprefix('rig_wing_');expected=wing_rotation(w,make_profile(a['name'])[w],t);np.testing.assert_allclose(q,expected,atol=2e-7)
        # Count triangles without decoding mesh buffers.
        tris=sum(e['accessors'][p['indices']]['count']//3 for n in e['nodes'] if 'mesh' in n for p in e['meshes'][n['mesh']]['primitives'])
        self.assertEqual(tris,504756)
    def test_pose_export(self):
        with tempfile.TemporaryDirectory() as tmp:
            out=Path(tmp)/'pose.glb';pose_glb(OUTPUT,out,'hover',.0833333333);d,b=read_glb(out);self.assertNotIn('animations',d)
            for n in d['nodes']:
                if n.get('name') in ['rig_wing_'+w for w in WINGS]:
                    w=n['name'].removeprefix('rig_wing_');np.testing.assert_allclose(n['rotation'],wing_rotation(w,make_profile('hover')[w],.0833333333),atol=2e-6)
        with self.assertRaises(ValueError):export_profiles(SOURCE,SOURCE)
    def test_preview(self):
        from PIL import Image
        with Image.open(ROOT/'renders/dragonfly_wing_motion.gif') as im:
            self.assertGreaterEqual(im.n_frames,4);self.assertEqual(im.size,(960,640));a=np.array(im.convert('RGB'));im.seek(2);b=np.array(im.convert('RGB'));self.assertGreater(np.mean(np.abs(a.astype(float)-b)),.1)
if __name__=='__main__':unittest.main(verbosity=2)
