"""Cinematic planner, fixed framing and media smoke checks."""
from pathlib import Path
import sys, tempfile, unittest, json, shutil, subprocess
import numpy as np
from PIL import Image
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'rendering'))
from step_30_cinematic import plan,encode,STAGES,STATIC
from camera import camera
class CinematicTests(unittest.TestCase):
    def test_all_assembly_stages(self):
        shots=plan('assembly',24,3)
        self.assertEqual(list(dict.fromkeys(s.source for s in shots)),list(map(str,STAGES)))
        self.assertTrue(all(p.is_file() for p in STAGES))
        self.assertEqual(len(shots),75)
    def test_turntable_and_macro(self):
        shots=plan('turntable',12,2)
        self.assertEqual(len(shots),24)
        np.testing.assert_allclose(np.diff([s.azimuth for s in shots]),15)
        self.assertEqual(shots[0].azimuth,35)
        macro=plan('macro',12,2)
        self.assertEqual(macro[0].azimuth,45);self.assertEqual(macro[-1].azimuth,80)
    def test_slow_motion(self):
        s=plan('motion',24,2,clip='forward',motion_speed=.25)
        np.testing.assert_allclose(np.diff([v.seconds for v in s]),.25/24)
        self.assertTrue(all(v.clip=='forward' for v in s))
    def test_validation(self):
        for args in [('bad',24,1),('motion',0,1),('motion',24,float('nan'))]:
            with self.assertRaises(ValueError):plan(*args)
    def test_fixed_camera_extent(self):
        b=np.array([[-.2,-.2,-.02],[.05,.2,.04]])
        for a in (0,90,180):
            vp,_=camera(b,16/9,a,45,extent_override=.25)
            self.assertAlmostEqual(np.linalg.norm(vp[1,:3]),4.)
    @unittest.skipUnless(shutil.which('ffmpeg') and shutil.which('ffprobe'),'FFmpeg unavailable')
    def test_video_encoding(self):
        with tempfile.TemporaryDirectory() as td:
            p=Path(td)
            for i in range(3):Image.new('RGB',(64,64),(i*60,20,40)).save(p/f'frame_{i:06d}.png')
            encode(p,p/'test.mp4',6)
            r=json.loads(subprocess.check_output(['ffprobe','-v','error','-show_streams','-of','json',str(p/'test.mp4')]))['streams'][0]
            self.assertEqual((r['codec_name'],r['width'],r['height'],int(r['nb_frames'])),('h264',64,64,3))
if __name__=='__main__':unittest.main(verbosity=2)
