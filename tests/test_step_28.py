"""Numerical and integration regression tests for Step 28."""
import sys, unittest
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'simulation'))
from step_28_flight import *

class FlightTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.cfg=FlightConfig();cls.geo=aero_geometry(cls.cfg);cls.hover=simulate('hover',config=cls.cfg,geo=cls.geo);cls.forward=simulate('forward',config=cls.cfg,geo=cls.geo)
    def test_geometry(self):
        g=self.geo;self.assertTrue(np.all(g.area>0));np.testing.assert_allclose(g.area[::2],g.area[1::2]);np.testing.assert_allclose(g.centers[::2,1],-g.centers[1::2,1]);self.assertEqual(np.linalg.matrix_rank(g.allocation),3)
    def test_gravity_and_trim(self):
        c,g=self.cfg,self.geo;force,torque,lift=forces(c,g,Rotation.identity(),np.zeros(3),np.zeros(4));np.testing.assert_allclose(force,[0,0,-c.mass*c.gravity]);np.testing.assert_allclose(torque,0)
        f,sat=allocate(g,c.mass*c.gravity,[0,0],c.max_frequency);self.assertFalse(sat);force,torque,_=forces(c,g,Rotation.identity(),np.zeros(3),np.sqrt(f/g.gain));np.testing.assert_allclose(force,0,atol=1e-12);np.testing.assert_allclose(torque,0,atol=1e-12)
    def test_hover_and_forward(self):
        for d in [self.hover,self.forward]:
            self.assertTrue(all(np.isfinite(v).all() for v in d.values()));self.assertLess(abs(d['position'][-1,2]-1),.005);self.assertGreater(d['position'][:,2].min(),0);self.assertFalse(d['saturated'].any());self.assertTrue(np.all((d['frequency']>=0)&(d['frequency']<=self.cfg.max_frequency)));np.testing.assert_allclose(np.linalg.norm(d['rotation'],axis=1),1,atol=1e-12);np.testing.assert_allclose(np.diff(d['phase_cycles'],axis=0),d['frequency'][:-1]*np.diff(d['time'])[:,None],atol=1e-11)
        self.assertLess(np.linalg.norm(self.hover['velocity'][-1]),.005);self.assertGreater(self.forward['position'][-1,0],1);self.assertGreater(self.forward['velocity'][-1,0],.5)
    def test_convergence(self):
        coarse=simulate('forward',dt=1/240,config=self.cfg,geo=self.geo);self.assertLess(np.linalg.norm(coarse['position'][-1]-self.forward['position'][-1]),.004)
    def test_saturation_and_validation(self):
        f,sat=allocate(self.geo,100,[0,0],self.cfg.max_frequency);self.assertTrue(sat);self.assertTrue(np.all(f<=self.geo.gain*self.cfg.max_frequency**2));self.assertRaises(ValueError,FlightConfig,mass=-1);self.assertRaises(ValueError,simulate,'invalid')
    def test_export_preservation_and_tracks(self):
        old,ob=read_glb(SOURCE);new,nb=read_glb(OUTPUT);self.assertEqual(nb[:len(ob)],ob);self.assertEqual(new['animations'][:3],old['animations']);
        for key in ('meshes','materials','textures','images'):self.assertEqual(new[key],old[key])
        self.assertEqual(new['nodes'][:len(old['nodes'])],old['nodes']);self.assertEqual(stats(new)['triangles'],504756)
        for clip,data in zip(new['animations'][3:],[self.hover,self.forward]):
            self.assertEqual(len(clip['channels']),100);self.assertEqual(len({(c['target']['node'],c['target']['path']) for c in clip['channels']}),100)
            for c in clip['channels']:
                s=clip['samplers'][c['sampler']];v=array(new,nb,s['output']);self.assertEqual(len(v),1921);self.assertTrue(np.isfinite(v).all());name=new['nodes'][c['target']['node']]['name'];path=c['target']['path']
                if path=='rotation':np.testing.assert_allclose(np.linalg.norm(v,axis=1),1,atol=1e-6)
                if name=='rig_flight_body' and path=='translation':np.testing.assert_allclose(v,data['position'],atol=1e-7)
                if name=='rig_wing_fore_left':
                    ref=wing_rotation('fore_left',WingControl(frequency=1.,flap=30.,pitch=10.),data['phase_cycles'][:,0]);np.testing.assert_allclose(abs(np.sum(v*ref,axis=1)),1,atol=1e-6)
if __name__=='__main__':unittest.main(verbosity=2)
