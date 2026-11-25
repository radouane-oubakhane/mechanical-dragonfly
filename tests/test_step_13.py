"""Stage 13: verify wing topology, attachments, symmetry and cumulative exports."""
from pathlib import Path
import numpy as np
import trimesh
from shapely.geometry import Polygon
from shapely.ops import unary_union
from step_12_legs import build_stage_12
from step_13_wing_skeleton import (make_wing_plan,create_wing_skeleton,wing_placements,build_stage_13,CHANNELS)


def check():
    _,prior=build_stage_12(); modules,scene=build_stage_13()
    assert len(modules)==4 and len(scene.geometry)==4527
    for n in prior.graph.nodes_geometry:
        a,g=prior.graph[n]; b,h=scene.graph[n]
        assert np.array_equal(a,b) and np.array_equal(prior.geometry[g].vertices,scene.geometry[h].vertices) and np.array_equal(prior.geometry[g].faces,scene.geometry[h].faces),n
    print('PASS: all 3299 previous meshes, vertices, faces and transforms preserved',flush=True)
    for n,m in scene.geometry.items():
        assert np.isfinite(m.vertices).all() and m.is_watertight and m.is_winding_consistent and m.volume>0,n
    print('PASS: 4527 finite, watertight, consistently wound positive-volume meshes',flush=True)
    placements=wing_placements(); shapes={}; tips={}
    for side,pair in CHANNELS:
        plan=make_wing_plan(side,pair); key=f'wing_skeleton_{pair}_{side}'; module=modules[key]; p=plan.points_mm
        assert len(plan.cells)==286 and len(plan.crossveins)==286 and len(plan.longitudinal)==8
        assert len(module.scene.geometry)==307 and len(module.anchors)==3
        edges={tuple(sorted((a,b))) for path in plan.longitudinal for a,b in zip(path,path[1:])}|set(plan.crossveins)
        assert all(tuple(sorted((a,b))) in edges for cell in plan.cells for a,b in zip(cell,cell[1:]+cell[:1]))
        outline=Polygon(p[list(plan.outline),:2]); cells=[Polygon(p[list(c),:2]) for c in plan.cells]
        assert outline.is_valid and all(c.is_valid and c.area>0 for c in cells)
        assert outline.symmetric_difference(unary_union(cells)).area<1e-8 and np.isclose(sum(c.area for c in cells),outline.area)
        for node in module.scene.graph.nodes_geometry:
            local,g=module.scene.graph[node]; actual,h=scene.graph[key+'/'+node]
            assert np.allclose(actual,placements[key]@local),node
        for anchor in module.anchors.values():
            assert np.allclose(anchor[:3,:3].T@anchor[:3,:3],np.eye(3)) and np.isclose(np.linalg.det(anchor[:3,:3]),1)
        world=trimesh.transform_points(p[list(plan.outline)]*.001,placements[key]); shapes[key]=Polygon(world[:,:2]); tips[key]=(placements[key]@module.anchors['tip'])[:3,3]
        # Wings start outward of their hinge mounting face, apart from tube radii.
        assert module.scene.bounds[0,0]>=-.0003
        assert np.array_equal(p,make_wing_plan(side,pair).points_mm)
        larger=create_wing_skeleton(make_wing_plan(side,pair,2)); assert np.allclose(larger.scene.bounds,module.scene.bounds*2)
    for pair in ('fore','hind'):
        a=make_wing_plan('left',pair); b=make_wing_plan('right',pair)
        assert np.array_equal(a.points_mm*[1,-1,1],b.points_mm)
        assert np.allclose(tips[f'wing_skeleton_{pair}_left']*[1,-1,1],tips[f'wing_skeleton_{pair}_right'])
    for side in ('left','right'):
        a=shapes['wing_skeleton_fore_'+side]; b=shapes['wing_skeleton_hind_'+side]
        assert a.intersection(b).area==0 and a.distance(b)>.008
    assert np.isclose(tips['wing_skeleton_fore_left'][1]-tips['wing_skeleton_fore_right'][1],.4)
    assert abs(scene.extents[1]-.4)<.0003
    print('PASS: 4 aligned wing mounts, 12 rigid anchors, 1144 nonoverlapping cells bounded by veins, deterministic mirrored plans, neutral fore/hind outline separation, nominal 400 mm tip span, proportional scaling',flush=True)
    for side,pair,scale in [('bad','fore',1),('left','bad',1),('left','fore',0),('right','hind',np.nan),('right','hind',-1)]:
        try: make_wing_plan(side,pair,scale)
        except ValueError: pass
        else: raise AssertionError('invalid parameters accepted')
    print('PASS: invalid input rejection',flush=True)
    root=Path(__file__).resolve().parents[1]/'models'
    for f,count in [('step_13_wing.glb',307),('step_13_wings.glb',1228),('step_13_assembled.glb',4527)]:
        loaded=trimesh.load_scene(root/f); assert len(loaded.geometry)==count
        if count==4527: assert np.allclose(loaded.bounds,scene.bounds,atol=1e-7)
    print('PASS: three GLB round trips and cumulative assembly bounds',flush=True)

if __name__=='__main__': check()
