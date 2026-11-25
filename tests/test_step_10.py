"""Stage 10 regression: preserved geometry, chain interfaces and GLB exports."""
from pathlib import Path
import numpy as np
import trimesh
from step_01_chassis import create_chassis
from step_09_facial_armor import build_stage_09
from step_10_abdominal_spine import (PITCHES,RADII,create_abdominal_segment,abdominal_placements,build_stage_10)


def check():
    _,prior=build_stage_09(); modules,scene=build_stage_10()
    assert len(modules)==10 and len(scene.geometry)==2178
    for n in prior.graph.nodes_geometry:
        a,g=prior.graph[n]; b,h=scene.graph[n]
        assert np.array_equal(a,b) and np.array_equal(prior.geometry[g].vertices,scene.geometry[h].vertices) and np.array_equal(prior.geometry[g].faces,scene.geometry[h].faces),n
    print('PASS: all 1730 previous vertices, faces and transforms preserved',flush=True)
    for n,m in scene.geometry.items():
        assert np.isfinite(m.vertices).all() and m.is_watertight and m.is_winding_consistent and m.volume>0,n
    print('PASS: all 2178 meshes finite, watertight, consistently wound and positive volume',flush=True)
    frames=abdominal_placements(); origin=create_chassis().anchors['abdomen']; t=origin.copy()
    previous=None; prev_t=None
    for i,(name,m) in enumerate(modules.items(),1):
        f=frames[name]; assert np.allclose(f@ m.anchors['mount'],t)
        assert len(m.anchors)==8
        for a in m.anchors.values():
            assert np.allclose(a[:3,:3].T@a[:3,:3],np.eye(3)) and np.isclose(np.linalg.det(a[:3,:3]),1)
        assert np.allclose((f@m.anchors['pivot'])[:3,2],[0,1,0] if i%2 else [0,0,1])
        for node in m.scene.graph.nodes_geometry:
            local,g=m.scene.graph[node]; actual,h=scene.graph[name+'/'+node]
            assert np.allclose(actual,f@local)
        if previous:
            assert np.allclose((prev_t@previous.anchors['next'])[:3,3],(f@m.anchors['pivot'])[:3,3])
            for side in ('left','right'):
                assert np.allclose(prev_t@previous.anchors['wire_out_'+side],f@m.anchors['wire_in_'+side])
            # Fork and hub are centered on the same joint axis and have axial gaps.
            r=RADII[i-1]; axis=m.anchors['pivot'][:3,2]; p=(f@m.anchors['pivot'])[:3,3]
            for s in (-1,1):
                fork=previous.scene.geometry[f'fixed_distal_fork_{s}']; hub=m.scene.geometry['moving_proximal_hub']
                fv=(trimesh.transform_points(fork.vertices,prev_t)-p)@axis*s
                hv=(trimesh.transform_points(hub.vertices,f)-p)@axis*s
                assert fv.min()>hv.max() and np.isclose(fv.min(),.34*r*.001)
        # Only the initial adapter bridges and fasteners may extend forward of the rear mount.
        for g,mesh in m.scene.geometry.items():
            if i==1 and not g.startswith('chassis_adapter_'):
                assert mesh.bounds[1,0] <= .00002, g
        previous,prev_t=m,f; t=f@m.anchors['next']
    assert np.isclose(np.linalg.norm(t[:3,3]-origin[:3,3]),sum(PITCHES)*.001)
    assert all(a>b for a,b in zip(RADII,RADII[1:]))
    assert abs(scene.extents[0]-.25)<.001
    print('PASS: ten chained mounts, alternating joint axes, nine fork/hub gaps, matched harness endpoints, taper and 165 mm pitch length',flush=True)
    for i in (1,5,10):
        a=create_abdominal_segment(i); b=create_abdominal_segment(i,2)
        assert np.allclose(b.scene.bounds,2*a.scene.bounds)
        for name,f in a.anchors.items(): assert np.allclose(b.anchors[name][:3,3],2*f[:3,3])
    for i,s in [(0,1),(11,1),(True,1),(1.5,1),(1,0),(1,-1),(1,np.inf),(1,np.nan)]:
        try: create_abdominal_segment(i,s)
        except ValueError: pass
        else: raise AssertionError('invalid input accepted')
    print('PASS: scaling and invalid input rejection',flush=True)
    root=Path(__file__).resolve().parents[1]/'models'
    for filename,count in [('step_10_abdominal_segment.glb',len(modules['abdominal_segment_01'].scene.geometry)),('step_10_abdomen.glb',448),('step_10_assembled.glb',2178)]:
        loaded=trimesh.load_scene(root/filename); assert len(loaded.geometry)==count
        if count==2178: assert np.allclose(loaded.bounds,scene.bounds,atol=1e-7)
    print('PASS: three GLB round trips and full assembly bounds',flush=True)

if __name__=='__main__': check()
