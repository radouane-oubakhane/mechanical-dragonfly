"""Stage 09: preserve geometry, validate head interfaces and GLB exports."""
from pathlib import Path
import numpy as np
import trimesh
from step_01_chassis import create_chassis
from step_07_head import create_head
from step_08_eyes import build_stage_08
from step_09_facial_armor import create_facial_armor,build_stage_09

def check():
    _,prior=build_stage_08(); face,scene=build_stage_09()
    assert len(face.scene.geometry)==74 and len(scene.geometry)==1730
    for n in prior.graph.nodes_geometry:
        a,g=prior.graph[n]; b,h=scene.graph[n]
        assert np.array_equal(a,b) and np.array_equal(prior.geometry[g].vertices,scene.geometry[h].vertices) and np.array_equal(prior.geometry[g].faces,scene.geometry[h].faces),n
    print('PASS: all 1656 previous mesh vertices, faces and transforms preserved')
    for n,m in scene.geometry.items():
        assert np.isfinite(m.vertices).all() and m.is_watertight and m.is_winding_consistent and m.volume>0,n
    print('PASS: all 1730 meshes finite, watertight, consistently wound and positive volume')
    head=create_head(); neck=create_chassis().anchors['neck']; placement=neck@head.anchors['facial_armor']
    for n in face.scene.graph.nodes_geometry:
        a,g=face.scene.graph[n]; b,h=scene.graph['facial_armor/'+n]
        assert np.allclose(b,placement@a)
    for side in ('left','right'):
        assert np.allclose(placement@face.anchors['antenna_'+side],neck@head.anchors['antenna_'+side])
    for n,a in face.anchors.items():
        assert a.shape==(4,4) and np.allclose(a[:3,:3].T@a[:3,:3],np.eye(3)) and np.isclose(np.linalg.det(a[:3,:3]),1),n
    assert len([n for n in face.scene.geometry if n.startswith('ocellus_lens_')])==3
    assert len([n for n in face.scene.geometry if n.startswith('antenna_tip_')])==2
    assert np.isclose(face.scene.geometry['mount_backplate'].bounds[0,0],0)
    print('PASS: face mount, both antenna socket frames, three ocelli, two antennae and nine rigid anchors')
    def bounds(prefix):
        points=[]
        for n in scene.graph.nodes_geometry:
            if n.startswith(prefix):
                t,g=scene.graph[n]; points.append(trimesh.transform_points(scene.geometry[g].vertices,t))
        p=np.vstack(points); return np.array([p.min(axis=0),p.max(axis=0)])
    f=bounds('facial_armor/'); l=bounds('eye_left/'); r=bounds('eye_right/')
    assert f[1,1]<l[0,1] and f[0,1]>r[1,1]
    print('PASS: complete face and antenna bounding envelope separated from both compound eyes along Y')
    large=create_facial_armor(2)
    assert np.allclose(large.scene.bounds,2*face.scene.bounds)
    for n,a in face.anchors.items():
        assert np.allclose(large.anchors[n][:3,3],2*a[:3,3])
    for scale in (0,-1,np.nan,np.inf):
        try: create_facial_armor(scale)
        except ValueError: pass
        else: raise AssertionError('invalid scale accepted')
    print('PASS: scaling and invalid input rejection')
    for filename,expected in [('step_09_facial_armor.glb',face.scene),('step_09_assembled.glb',scene)]:
        loaded=trimesh.load_scene(Path(__file__).resolve().parents[1]/'models'/filename)
        assert len(loaded.geometry)==len(expected.geometry) and np.allclose(loaded.bounds,expected.bounds,atol=1e-7)
        lenses=[m for n,m in loaded.geometry.items() if 'ocellus_lens_' in n]
        assert len(lenses)==3 and all(m.visual.material.alphaMode=='BLEND' for m in lenses)
    print('PASS: both GLB round trips and auxiliary lens transparency metadata')
if __name__=='__main__': check()
