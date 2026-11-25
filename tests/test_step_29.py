"""Local application API tests, independent of browser GPU availability."""
import sys,unittest,tempfile,threading,json,urllib.request
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'app'))
from server import pose, flight, export_pose, Handler, ThreadingHTTPServer, MODELS, read_glb
class ControlTests(unittest.TestCase):
 def test_models(self):
  self.assertEqual(len(MODELS),18)
  for f in MODELS.values(): self.assertTrue((ROOT/'models'/f).is_file())
 def test_head_abdomen_leg(self):
  h=pose({'kind':'head','target':[.25,.04,.02]});self.assertEqual(len(h['rotations']),2);self.assertLessEqual(abs(h['angles'][0]),25)
  a=pose({'kind':'abdomen','vertical':12,'lateral':-10});self.assertEqual(len(a['rotations']),10)
  for leg in [f'{i}_{s}' for i in (1,2,3) for s in ('left','right')]:
   r=pose({'kind':'leg','leg':leg,'offset_mm':[0,0,.5]});self.assertTrue(r['reachable']);self.assertLess(r['error_mm'],.02);self.assertEqual(len(r['rotations']),7)
  for r in [h,a]:
   for q in r['rotations'].values():self.assertAlmostEqual(np.linalg.norm(q),1)
 def test_flight(self):
  r=flight({'mode':'hover','duration':2});self.assertEqual(len(r['time']),121);self.assertGreater(r['position'][-1][2],.9);self.assertEqual(len(r['frequency'][0]),4)
  with self.assertRaises(ValueError):flight({'mass_g':0})
 def test_export(self):
  d,b=read_glb(ROOT/'models'/MODELS['flight']);i=next(i for i,n in enumerate(d['nodes']) if n.get('name')=='rig_head_yaw')
  with tempfile.TemporaryDirectory() as tmp:
   p=Path(tmp)/'pose.glb';export_pose({'model':'flight','transforms':{str(i):{'rotation':[0,0,0,1]}}},p);o,raw=read_glb(p);self.assertEqual(b,raw);self.assertNotIn('animations',o);self.assertEqual(o['nodes'][i]['rotation'],[0,0,0,1]);self.assertEqual(d['meshes'],o['meshes'])
 def test_http(self):
  server=ThreadingHTTPServer(('127.0.0.1',0),Handler);t=threading.Thread(target=server.serve_forever,daemon=True);t.start();base=f'http://127.0.0.1:{server.server_port}'
  try:
   with urllib.request.urlopen(base+'/') as r:self.assertIn(b'CONTROL STUDIO',r.read())
   with urllib.request.urlopen(base+'/api/models') as r:self.assertEqual(json.load(r),MODELS)
   with urllib.request.urlopen(base+'/model/1') as r:self.assertEqual(r.read(4),b'glTF')
   req=urllib.request.Request(base+'/api/pose',data=json.dumps({'kind':'head','target':[.25,.04,.02]}).encode(),headers={'Content-Type':'application/json'})
   with urllib.request.urlopen(req) as r:self.assertEqual(len(json.load(r)['rotations']),2)
  finally:server.shutdown();server.server_close();t.join()
if __name__=='__main__':unittest.main(verbosity=2)
