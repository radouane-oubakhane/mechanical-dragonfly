"""Stage 06 geometry, attachment and regression checks."""
from pathlib import Path
import numpy as np
import trimesh
from step_01_chassis import create_chassis
from step_04_transmission import create_transmission
from step_05_actuators import build_stage_05
from step_06_wing_joints import create_wing_joint, build_stage_06


def check():
    actuators, prior=build_stage_05()
    joints, assembled=build_stage_06()
    assert len(joints)==4 and len(assembled.geometry)==len(prior.geometry)+120
    for node in prior.graph.nodes_geometry:
        a,g=prior.graph[node]; b,h=assembled.graph[node]
        assert np.array_equal(a,b) and np.array_equal(prior.geometry[g].vertices,assembled.geometry[h].vertices) and np.array_equal(prior.geometry[g].faces,assembled.geometry[h].faces),node
    print('PASS: all 428 prior mesh vertices, faces and transforms preserved')
    for name,m in assembled.geometry.items():
        assert np.isfinite(m.vertices).all() and m.is_watertight and m.is_winding_consistent and m.volume>0,name
    print('PASS: all 548 meshes finite, watertight, consistently wound and positive volume')
    root=create_chassis().anchors['transmission']; transmission=create_transmission()
    for name,joint in joints.items():
        channel=name.removeprefix('wing_joint_'); pair,side=channel.split('_'); sign=1 if side=='left' else -1
        key='actuator_'+channel; placement=root@transmission.anchors[key]@actuators[key].anchors['output']
        for a in joint.anchors.values():
            assert a.shape==(4,4) and np.allclose(a[3],[0,0,0,1])
            assert np.allclose(a[:3,:3].T@a[:3,:3],np.eye(3)) and np.isclose(np.linalg.det(a[:3,:3]),1)
        wing=placement@joint.anchors['wing']; pivot=placement@joint.anchors['flap_pivot']
        assert np.allclose(wing[:3,0],[0,sign,0])
        assert np.allclose(wing[:3,3]*1000,[13 if pair=='fore' else -13,sign*25.525,17.7])
        assert np.allclose(abs(pivot[:3,1]),[1,0,0])
        assert np.isclose(joint.scene.geometry['fixed_mount_tongue'].bounds[0,0],0)
        assert np.isclose(joint.scene.geometry['rotor_socket_jaw_1'].bounds[1,0],joint.anchors['wing'][0,3])
        # A conservative global clearance proof: all prior meshes lie inside
        # |Y| <= 20.6 mm; new meshes lie outside it, except touching mounts.
        for node in joint.scene.graph.nodes_geometry:
            tf,g=joint.scene.graph[node]; v=trimesh.transform_points(joint.scene.geometry[g].vertices,placement@tf)
            assert np.min(sign*v[:,1])>=prior.bounds[1,1]-1e-9,(name,node)
    print('PASS: four aligned actuator mounts, outward wing sockets, longitudinal flap axes, prior-stage envelope clearance')
    _,large=build_stage_06(2)
    assert np.allclose(large.bounds,2*assembled.bounds)
    for args in [('bad','fore',1),('left','bad',1),('left','fore',0),('left','fore',np.nan)]:
        try: create_wing_joint(*args)
        except ValueError: pass
        else: raise AssertionError('invalid arguments accepted')
    print('PASS: scaling and invalid argument rejection')
    for name,scene in [('step_06_wing_joint.glb',joints['wing_joint_fore_left'].scene),('step_06_assembled.glb',assembled)]:
        loaded=trimesh.load_scene((Path(__file__).resolve().parents[1]/'models')/name)
        assert len(loaded.geometry)==len(scene.geometry) and np.allclose(loaded.bounds,scene.bounds,atol=1e-7)
    print('PASS: both GLB round trips')

if __name__=='__main__': check()
