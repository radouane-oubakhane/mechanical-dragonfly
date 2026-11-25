"""Test actual studio rendering, HDR light data and projected shadows."""
from pathlib import Path
import hashlib,sys,tempfile
import numpy as np
from PIL import Image
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'rendering'))
from studio import PRESETS,environment,shadow_atlas
from scene import Scene
from step_22_studio import render

def main():
    source=ROOT/'models/dragonfly_static_balanced_iridescent.glb'
    digest=hashlib.sha256(source.read_bytes()).digest()
    scene=Scene(source)
    maps=[environment(s) for s in PRESETS.values()]
    for s,m in zip(PRESETS.values(),maps):
        assert m.shape==(256,512,3) and m.dtype==np.float32
        assert np.isfinite(m).all() and m.min()>=0 and m.max()>1
        assert np.array_equal(m,environment(s))
    assert not np.array_equal(maps[0],maps[1])
    print('PASS: three deterministic linear HDR environments')
    a,lo,hi,z=shadow_atlas(scene,PRESETS['graphite'])
    assert a.shape==(768,768,2) and np.all(hi>lo) and z<scene.bounds[0,2]
    assert a[:,:,0].max()>50 and a[:,:,1].max()>20
    assert np.any((a[:,:,0]>0)&(a[:,:,0]<255))
    print('PASS: floor placement, projected shadow and contact masks, soft boundaries')
    with tempfile.TemporaryDirectory() as d:
        paths=[Path(d)/f'{i}.png' for i in range(3)]
        reports=[render(source,paths[0],480,320,30,50,preset='graphite'),render(source,paths[1],480,320,30,50,preset='graphite',ground=False),render(source,paths[2],480,320,30,50,preset='gallery',projection='perspective',supersample=2)]
        imgs=[np.asarray(Image.open(p)) for p in paths]
        for im,r in zip(imgs,reports):
            assert im.shape==(320,480,3) and im.std()>5
            assert r['triangles']==504756 and r['textures']==35 and r['hdr_peak']>1
        assert np.mean(np.abs(imgs[0].astype(float)-imgs[1]))>5
        assert np.mean(np.abs(imgs[0].astype(float)-imgs[2]))>5
    print('PASS: actual OpenGL rendering, floor toggle, perspective, lighting differences, supersampling and PNG export')
    assert hashlib.sha256(source.read_bytes()).digest()==digest
    print('PASS: original model unchanged')
if __name__=='__main__':main()
