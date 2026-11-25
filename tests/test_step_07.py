"""Head mesh, mounting, frame and cumulative regression tests."""
from pathlib import Path
import numpy as np
import trimesh
from step_01_chassis import create_chassis
from step_06_wing_joints import build_stage_06
from step_07_head import create_head, build_stage_07


def check():
    _,prior=build_stage_06()
    head,assembled=build_stage_07()
    assert len(head.scene.geometry)==86 and len(assembled.geometry)==634
    for node in prior.graph.nodes_geometry:
        a,g=prior.graph[node]; b,h=assembled.graph[node]
        assert np.array_equal(a,b) and np.array_equal(prior.geometry[g].vertices,assembled.geometry[h].vertices) and np.array_equal(prior.geometry[g].faces,assembled.geometry[h].faces),node
    print('PASS: all 548 previous mesh vertices, faces and transforms preserved')
    for name,m in assembled.geometry.items():
        assert np.isfinite(m.vertices).all() and m.is_watertight and m.is_winding_consistent and m.volume>0,name
    print('PASS: 634 meshes finite, watertight, consistently wound and positive volume')
    root=create_chassis(); placement=root.anchors['neck']
    for name,f in head.anchors.items():
        assert f.shape==(4,4) and np.allclose(f[3],[0,0,0,1]),name
        assert np.allclose(f[:3,:3].T@f[:3,:3],np.eye(3)) and np.isclose(np.linalg.det(f[:3,:3]),1),name
    for name,m in head.scene.geometry.items():
        assert m.bounds[0,0]>=-1e-10,name
    assert np.isclose(prior.bounds[1,0],placement[0,3])
    for y in (-8,8):
        for z in (-6,6):
            pad=head.scene.geometry[f'fixed_rail_pad_{y}_{z}']
            rail=root.scene.geometry[f'{"left" if y>0 else "right"}_{"upper" if z>0 else "lower"}_rail']
            assert np.allclose(pad.bounds[:,1:],rail.bounds[:,1:])
            assert np.isclose(pad.bounds[0,0]+placement[0,3],rail.bounds[1,0])
    assert np.allclose(head.anchors['yaw_pivot'][:3,0],[0,0,1])
    assert np.allclose(head.anchors['pitch_pivot'][:3,0],[0,1,0])
    left=head.anchors['eye_left']; right=head.anchors['eye_right']; mirror=np.diag([1,-1,1])
    assert np.allclose(left[:3,3],mirror@right[:3,3])
    assert np.allclose(left[:3,0],mirror@right[:3,0]) and left[1,0]>.9 and left[0,0]>0
    for side in ('left','right'):
        f=head.anchors[f'eye_{side}']; m=head.scene.geometry[f'pitch_eye_socket_{side}']
        assert np.isclose(np.max((m.vertices-f[:3,3])@f[:3,0]),0,atol=1e-9)
    print('PASS: chassis mounting pads, prior-stage front-plane clearance, orthogonal neck axes and mirrored eye sockets')
    _,large=build_stage_07(2)
    assert np.allclose(large.bounds,2*assembled.bounds)
    for scale in (0,-1,np.nan,np.inf):
        try: create_head(scale)
        except ValueError: pass
        else: raise AssertionError('invalid scale accepted')
    print('PASS: scaling and invalid input rejection')
    for name,scene in [('step_07_head.glb',head.scene),('step_07_assembled.glb',assembled)]:
        loaded=trimesh.load_scene(Path(__file__).resolve().parents[1]/'models'/name)
        assert len(loaded.geometry)==len(scene.geometry) and np.allclose(loaded.bounds,scene.bounds,atol=1e-7)
    print('PASS: head and cumulative GLB round trips')

if __name__=='__main__': check()
