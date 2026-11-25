"""Validate optical geometry, mounting, preservation and exported models."""
from pathlib import Path
import numpy as np
import trimesh
from step_01_chassis import create_chassis
from step_07_head import build_stage_07
from step_08_eyes import build_stage_08, create_compound_eye, facet_centers, PITCH, PARAM_RADIUS


def check():
    head,prior=build_stage_07(); eyes,scene=build_stage_08()
    assert len(facet_centers())==241
    assert len(scene.geometry)==1656
    for node in prior.graph.nodes_geometry:
        a,g=prior.graph[node]; b,h=scene.graph[node]
        assert np.array_equal(a,b) and np.array_equal(prior.geometry[g].vertices,scene.geometry[h].vertices) and np.array_equal(prior.geometry[g].faces,scene.geometry[h].faces),node
    print('PASS: all 634 previous mesh vertices, faces and transforms preserved')
    for name,m in scene.geometry.items():
        assert np.isfinite(m.vertices).all() and m.is_watertight and m.is_winding_consistent and m.volume>0,name
    print('PASS: all 1656 meshes finite, watertight, consistently wound and positive volume')
    points=facet_centers(); d=np.linalg.norm(points[:,None]-points[None,:],axis=-1); np.fill_diagonal(d,np.inf)
    assert d.min()>=PITCH-1e-10
    assert np.max(np.linalg.norm(points,axis=1))+PITCH/np.sqrt(3)<=PARAM_RADIUS+1e-10
    neck=create_chassis().anchors['neck']; mirror=np.diag([1,-1,1])
    for side,eye in eyes.items():
        assert len(eye.scene.geometry)==511
        assert len([n for n in eye.scene.geometry if n.endswith('_lens')])==241
        assert len([n for n in eye.scene.geometry if n.endswith('_bezel')])==241
        assert np.allclose(eye.scene.bounds*1000,[[0,-7.05,-7.05],[6.57,7.05,7.05]])
        f=head.anchors['eye_'+side]; placement=neck@f
        for node in eye.scene.graph.nodes_geometry:
            a,g=eye.scene.graph[node]; b,h=scene.graph[f'eye_{side}/{node}']; assert np.allclose(b,placement@a)
        # Test head vertices within the radial footprint of the installed eye.
        # This is not full triangle-level intersection testing.
        for name,m in head.scene.geometry.items():
            local=(m.vertices-f[:3,3])@f[:3,:3]
            assert not np.any((local[:,0]>1e-8)&(np.linalg.norm(local[:,1:],axis=1)<.00705)),name
        flange=eye.scene.geometry['mount_flange']; radius=np.linalg.norm(flange.vertices[:,1:],axis=1)
        assert np.isclose(radius.min(),.0062) and np.isclose(radius.max(),.00705)
        for name,anchor in eye.anchors.items():
            assert anchor.shape==(4,4) and np.allclose(anchor[:3,:3].T@anchor[:3,:3],np.eye(3)) and np.isclose(np.linalg.det(anchor[:3,:3]),1),name
    assert np.allclose(head.anchors['eye_left'][:3,3],mirror@head.anchors['eye_right'][:3,3])
    assert np.allclose(head.anchors['eye_left'][:3,0],mirror@head.anchors['eye_right'][:3,0])
    print('PASS: 482 distinct lens cells and 482 bezels; lattice pitch, aperture, flange, mirrored axes, transforms and targeted head vertex clearance')
    large=create_compound_eye('left',2)
    assert np.allclose(large.scene.bounds,2*eyes['left'].scene.bounds)
    for side,scale in [('invalid',1),('left',0),('left',-1),('left',np.nan),('left',np.inf)]:
        try: create_compound_eye(side,scale)
        except ValueError: pass
        else: raise AssertionError('invalid arguments accepted')
    print('PASS: eye scaling and invalid argument rejection')
    for filename,expected in [('step_08_eye.glb',eyes['left'].scene),('step_08_assembled.glb',scene)]:
        loaded=trimesh.load_scene(Path(__file__).resolve().parents[1]/'models'/filename)
        assert len(loaded.geometry)==len(expected.geometry) and np.allclose(loaded.bounds,expected.bounds,atol=1e-7)
        lens=next(m for n,m in loaded.geometry.items() if n.endswith('_lens'))
        assert lens.visual.material.alphaMode=='BLEND' and lens.visual.material.baseColorFactor[3]<255
    print('PASS: both GLB round trips and lens transparency metadata')

if __name__=='__main__': check()
