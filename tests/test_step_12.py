"""Regression: preserved geometry, leg geometry, mounts, mirror, and GLB exports."""
from pathlib import Path
import numpy as np
import trimesh
from step_01_chassis import create_chassis
from step_11_abdominal_armor import build_stage_11
from step_12_legs import create_leg, build_stage_12


def check():
    _,prior=build_stage_11(); modules,scene=build_stage_12()
    assert len(modules)==6 and len(scene.geometry)==3299
    for n in prior.graph.nodes_geometry:
        a,g=prior.graph[n]; b,h=scene.graph[n]
        assert np.array_equal(a,b) and np.array_equal(prior.geometry[g].vertices,scene.geometry[h].vertices) and np.array_equal(prior.geometry[g].faces,scene.geometry[h].faces),n
    print('PASS: all 2657 prior meshes, vertices, faces and transforms preserved',flush=True)
    for n,m in scene.geometry.items():
        assert np.isfinite(m.vertices).all() and m.is_watertight and m.is_winding_consistent and m.volume>0,n
    print('PASS: 3299 finite, watertight, consistently wound positive-volume meshes',flush=True)
    chassis=create_chassis(); feet=[]; bounds={}; mirror=np.array([1,-1,1])
    for pair in (1,2,3):
        left=modules[f'leg_{pair}_left']; right=modules[f'leg_{pair}_right']
        for name,mesh in left.scene.geometry.items():
            assert np.allclose(mesh.vertices*mirror,right.scene.geometry[name].vertices),name
        for module in (left,right):
            p=chassis.anchors[module.name]; assert len(module.anchors)==12
            for name,a in module.anchors.items():
                assert np.allclose(a[:3,:3].T@a[:3,:3],np.eye(3)) and np.isclose(np.linalg.det(a[:3,:3]),1),name
            for node in module.scene.graph.nodes_geometry:
                local,g=module.scene.graph[node]; actual,h=scene.graph[module.name+'/'+node]
                assert np.allclose(actual,p@local),node
            feet.append((p@module.anchors['foot'])[2,3]); bounds[module.name]=module.scene.bounds+p[:3,3]
            for name in ('hip_yaw','hip_pitch','knee','ankle','tarsus_1','tarsus_2','tarsus_3'):
                assert name+'_pivot' in module.anchors
            for name in ('femur_tension_spring','leg_data_harness'):
                assert len(module.scene.geometry[name].split())==1
    assert np.allclose(feet,-.039)
    # Entire leg bounding boxes must be disjoint from each other; no claim
    # about contact or internal collisions inside any single leg.
    for i,a in enumerate(bounds.values()):
        for b in list(bounds.values())[i+1:]:
            assert np.any(a[1]<b[0]) or np.any(b[1]<a[0]),(a,b)
    print('PASS: six mount transforms, mirrored geometry, 72 right-handed anchor frames, seven joint axes per leg, coplanar foot anchors, disjoint inter-leg AABBs',flush=True)
    for pair in (1,2,3):
        a=create_leg(pair,'left'); b=create_leg(pair,'left',2)
        assert np.allclose(b.scene.bounds,2*a.scene.bounds)
        for n,f in a.anchors.items(): assert np.allclose(b.anchors[n][:3,3],2*f[:3,3])
    for pair,side,s in [(0,'left',1),(4,'left',1),(True,'left',1),(1,'bad',1),(1,'right',0),(1,'right',np.nan),(1,'left',-1)]:
        try: create_leg(pair,side,s)
        except ValueError: pass
        else: raise AssertionError('invalid input accepted')
    print('PASS: scaling and invalid input rejection',flush=True)
    root=Path(__file__).resolve().parents[1]/'models'
    for f,count in [('step_12_leg.glb',107),('step_12_legs.glb',642),('step_12_assembled.glb',3299)]:
        loaded=trimesh.load_scene(root/f); assert len(loaded.geometry)==count
        if count==3299: assert np.allclose(loaded.bounds,scene.bounds,atol=1e-7)
    print('PASS: three GLB round trips and assembly bounds',flush=True)

if __name__=='__main__': check()
