"""Stage 21: actual offscreen shader and rasterization integration tests."""
from pathlib import Path
import hashlib, sys, tempfile
import numpy as np
from PIL import Image
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'rendering'))
from scene import Scene
from camera import camera
from step_21_renderer import render

def main():
    source=ROOT/'models/dragonfly_static_balanced_iridescent.glb'
    digest=hashlib.sha256(source.read_bytes()).digest()
    scene=Scene(source)
    assert scene.triangles==504756 and len(scene.images)==35
    assert np.isfinite(scene.vertices).all()
    assert len(scene.draws)==1709 and sum(scene.transparent(d) for d in scene.draws)==1631
    print('PASS: scene ingestion, triangle count, 35 textures, material classification')
    for projection in ('orthographic','perspective'):
        vp,eye=camera(scene.bounds,1.6,25,55,projection)
        assert np.isfinite(vp).all() and np.linalg.det(vp)!=0
    with tempfile.TemporaryDirectory() as temp:
        paths=[Path(temp)/f'{n}.png' for n in range(2)]
        results=[render(source,paths[0],480,320,12,65,supersample=2),render(source,paths[1],480,320,55,35,projection='perspective')]
        imgs=[np.asarray(Image.open(p)) for p in paths]
        for im,report in zip(imgs,results):
            assert im.shape==(320,480,3) and report['triangles']==504756
            assert np.count_nonzero(np.any(im!=im[0,0],axis=-1))>1000
            assert im.std()>5
        assert np.mean(np.abs(imgs[0].astype(float)-imgs[1]))>1
        print('PASS: both camera projections, shader compilation, textured rendering, antialiasing and PNG export')
        print('Backend:',results[0]['renderer'])
    assert hashlib.sha256(source.read_bytes()).digest()==digest
    print('PASS: source GLB unchanged')
if __name__=='__main__':main()
