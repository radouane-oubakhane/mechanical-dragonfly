"""Static geometry, socket alignment and cumulative regression tests."""
from pathlib import Path
import numpy as np
import trimesh
from step_01_chassis import create_chassis
from step_04_transmission import build_stage_04
from step_05_actuators import build_stage_05, create_wing_actuator


def check():
    transmission, prior=build_stage_04()
    modules, assembled=build_stage_05()
    assert len(modules)==4
    assert len(assembled.geometry)==len(prior.geometry)+sum(len(m.scene.geometry) for m in modules.values())
    for node in prior.graph.nodes_geometry:
        a,g=prior.graph[node]; b,h=assembled.graph[node]
        assert np.array_equal(a,b),node
        assert np.array_equal(prior.geometry[g].vertices,assembled.geometry[h].vertices),node
        assert np.array_equal(prior.geometry[g].faces,assembled.geometry[h].faces),node
    print('PASS: all Stage 04 meshes and transforms preserved')
    for name,mesh in assembled.geometry.items():
        assert np.isfinite(mesh.vertices).all(),name
        assert mesh.is_watertight and mesh.is_winding_consistent and mesh.volume>0,name
    print('PASS: finite, watertight, consistently wound positive-volume meshes')
    t=create_chassis().anchors['transmission']
    for key,m in modules.items():
        channel=key.removeprefix('actuator_')
        pair,side=channel.split('_'); sy=1 if side=='left' else -1
        placement=t@transmission.anchors[key]
        assert len(m.anchors)==6
        for anchor in m.anchors.values():
            assert anchor.shape==(4,4) and np.allclose(anchor[3],[0,0,0,1])
            assert np.allclose(anchor[:3,:3].T@anchor[:3,:3],np.eye(3))
            assert np.isclose(np.linalg.det(anchor[:3,:3]),1)
        assert np.allclose((placement@m.anchors['gear_input'])[:3,3],(t@transmission.anchors['output_shaft_'+channel])[:3,3])
        output=placement@m.anchors['output']
        assert np.allclose(output[:3,0],[0,sy,0])
        assert np.allclose(output[:3,3]*1000,[13 if pair=='fore' else -13,sy*20.6,17.7])
        assert np.isclose(m.anchors['output'][0,3],m.scene.geometry['output_mount_lug'].bounds[1,0])
        assert np.isclose(m.scene.geometry['mounting_tongue'].bounds[0,0],0)
        # Inspect the static clearance against prior meshes using their world
        # bounding boxes, allowing only the two intended touching interfaces.
        for node in m.scene.graph.nodes_geometry:
            tf,g=m.scene.graph[node]; mesh=m.scene.geometry[g].copy(); mesh.apply_transform(placement@tf)
            low,high=mesh.bounds
            for old in prior.graph.nodes_geometry:
                old_tf,old_g=prior.graph[old]; old_mesh=prior.geometry[old_g]
                corners=trimesh.bounds.corners(old_mesh.bounds)
                corners=trimesh.transform_points(corners,old_tf)
                overlap=np.minimum(high,corners.max(axis=0))-np.maximum(low,corners.min(axis=0))
                assert not np.all(overlap>1e-9),(key,node,old,overlap)
    print('PASS: four socket and crank center alignments; outward wing anchors; no positive AABB overlap with previous stage')
    _,large=build_stage_05(2)
    assert np.allclose(large.bounds,2*assembled.bounds)
    for args in [('bad','fore',1),('left','bad',1),('left','fore',0),('left','fore',np.nan)]:
        try: create_wing_actuator(*args)
        except ValueError: pass
        else: raise AssertionError('invalid arguments accepted')
    print('PASS: scale and argument validation')
    for name,scene in [('step_05_actuator.glb',modules['actuator_fore_left'].scene),('step_05_assembled.glb',assembled)]:
        loaded=trimesh.load_scene((Path(__file__).resolve().parents[1]/'models')/name)
        assert len(loaded.geometry)==len(scene.geometry)
        assert np.allclose(loaded.bounds,scene.bounds,atol=1e-7)
    print('PASS: both GLB exports reimport with matching counts and bounds')

if __name__=='__main__': check()
