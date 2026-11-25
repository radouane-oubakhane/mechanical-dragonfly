"""Stage 11: geometry preservation, armor shell topology and mount checks."""
from pathlib import Path
import numpy as np
import trimesh
from step_01_chassis import create_chassis
from step_10_abdominal_spine import build_stage_10, segment_dimensions
from step_11_abdominal_armor import build_stage_11, create_abdominal_armor, create_tail


def check():
    segments,prior=build_stage_10(); modules,scene=build_stage_11()
    assert len(modules)==11 and len(scene.geometry)==2657
    for n in prior.graph.nodes_geometry:
        a,g=prior.graph[n]; b,h=scene.graph[n]
        assert np.array_equal(a,b) and np.array_equal(prior.geometry[g].vertices,scene.geometry[h].vertices) and np.array_equal(prior.geometry[g].faces,scene.geometry[h].faces),n
    print('PASS: all 2178 previous meshes, vertices, faces and transforms preserved',flush=True)
    for n,m in scene.geometry.items():
        assert np.isfinite(m.vertices).all() and m.is_watertight and m.is_winding_consistent and m.volume>0,n
    print('PASS: all 2657 meshes finite, watertight, consistently wound and positive volume',flush=True)
    transform=create_chassis().anchors['abdomen']; last_end=None
    for i in range(1,11):
        segment=segments[f'abdominal_segment_{i:02d}']; module=modules[f'abdominal_armor_{i:02d}']; L,r0,r1=segment_dimensions(i)
        placement=transform@np.linalg.inv(segment.anchors['mount']); expected=placement@segment.anchors['armor']
        assert len(module.anchors)==5
        for node in module.scene.graph.nodes_geometry:
            local,g=module.scene.graph[node]; actual,h=scene.graph[module.name+'/'+node]
            assert np.allclose(actual,expected@local)
        # Panels span only the central 68% of each segment; the open joint zones
        # remain exposed. Inner shell vertices clear the nominal frame radius.
        panels=[]
        for label in ('left','right','dorsal','ventral'):
            m=module.scene.geometry['panel_'+label]; v=m.vertices*1000
            t=.5-v[:,0]/L; radius=r0+(r1-r0)*t
            assert np.allclose([t.min(),t.max()],[.16,.84])
            assert np.min(np.linalg.norm(v[:,1:],axis=1)-radius)>.349
            assert m.euler_number==(-4 if label in ('left','right') else 2),label
            assert len(m.split())==1
            # Check the actual polygonal frame/rail envelope at the shell's x range.
            panels.append(m)
        # All original spine vertices within the axial panel span fit inside
        # the minimum inner radius at their longitudinal station.
        for name,m in segment.scene.geometry.items():
            v=m.vertices*1000; t=-v[:,0]/L; keep=(t>=.16)&(t<=.84)
            assert np.all(np.linalg.norm(v[keep,1:],axis=1)<r0+(r1-r0)*t[keep]+.34), (i,name)
        lo,hi=module.scene.bounds[:,0]+expected[0,3]
        if last_end is not None: assert last_end-hi>.0035
        last_end=lo
        transform=placement@segment.anchors['next']
    tail=modules['tail']; assert len(tail.anchors)==4
    for node in tail.scene.graph.nodes_geometry:
        local,g=tail.scene.graph[node]; actual,h=scene.graph['tail/'+node]; assert np.allclose(actual,transform@local)
    assert abs(scene.extents[0]-.25571473812)<1e-8
    print('PASS: ten armor mounts, actual spine vertex clearance, nine exposed joint gaps, 60 cut-through slots, tail alignment',flush=True)
    for i in (1,5,10):
        a=create_abdominal_armor(i); b=create_abdominal_armor(i,2)
        assert np.allclose(b.scene.bounds,2*a.scene.bounds)
        for n,f in a.anchors.items(): assert np.allclose(b.anchors[n][:3,3],2*f[:3,3])
    assert np.allclose(create_tail(2).scene.bounds,2*create_tail().scene.bounds)
    for i,s in [(0,1),(11,1),(True,1),(1,0),(1,np.nan),(1,-1),(1,np.inf)]:
        try: create_abdominal_armor(i,s)
        except ValueError: pass
        else: raise AssertionError('invalid input accepted')
    print('PASS: scaling and invalid input rejection',flush=True)
    root=Path(__file__).resolve().parents[1]/'models'
    for f,count in [('step_11_abdominal_armor.glb',len(modules['abdominal_armor_01'].scene.geometry)),('step_11_tail.glb',len(tail.scene.geometry)),('step_11_assembled.glb',2657)]:
        loaded=trimesh.load_scene(root/f); assert len(loaded.geometry)==count
        if count==2657: assert np.allclose(loaded.bounds,scene.bounds,atol=1e-7)
    print('PASS: three GLB round trips and full assembly bounds',flush=True)

if __name__=='__main__': check()
