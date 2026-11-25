"""Regression and targeted clearance tests for Stage 03 (no CAD collision solver)."""
from pathlib import Path
import numpy as np
import trimesh
from step_01_chassis import create_chassis
from step_02_power_core import create_power_core, attach_module
from step_03_electronics import create_electronics, create_power_harness, build_stage_03


def check():
    chassis, power = create_chassis(), create_power_core()
    electronics, harness, assembled = build_stage_03()
    old = attach_module(chassis, 'power', power)
    assert len(assembled.geometry) == len(old.geometry)+len(electronics.scene.geometry)+len(harness.scene.geometry)
    for node in old.graph.nodes_geometry:
        mat, geom = old.graph[node]
        mat2, geom2 = assembled.graph[node]
        assert np.allclose(mat, mat2)
        assert np.array_equal(old.geometry[geom].vertices, assembled.geometry[geom2].vertices)
        assert np.array_equal(old.geometry[geom].faces, assembled.geometry[geom2].faces)
    print('PASS: earlier geometry and transforms preserved')
    for mesh in assembled.geometry.values():
        assert mesh.is_watertight and mesh.is_winding_consistent and mesh.volume > 0
        assert np.isfinite(mesh.vertices).all()
    print('PASS: finite, watertight, consistently wound positive-volume meshes')
    for module in [chassis,power,electronics,harness]:
        for a in module.anchors.values():
            assert a.shape == (4,4) and np.allclose(a[3], [0,0,0,1])
            assert np.allclose(a[:3,:3].T@a[:3,:3], np.eye(3))
            assert np.isclose(np.linalg.det(a[:3,:3]), 1)
    for label in ['positive','negative']:
        s = chassis.anchors['power'] @ power.anchors[label]
        e = chassis.anchors['electronics'] @ electronics.anchors['power_'+label]
        assert np.allclose(s, harness.anchors[label+'_start'])
        assert np.allclose(e, harness.anchors[label+'_end'])
        # Tube cap center vertices coincide with the actual terminal and connector.
        for name, point in [(label+'_terminal_lead',s[:3,3]),(label+'_pcb_lead',e[:3,3])]:
            assert np.min(np.linalg.norm(harness.scene.geometry[name].vertices-point,axis=1)) < 1e-9
    print('PASS: rigid anchors and wire endpoint alignment')
    # For each new triangle, intersect its edges with both planes of every
    # frame slab. All vertices of the clipped triangle must fit inside the
    # octagonal aperture; convexity then guarantees clearance in the slab.
    clearance = float('inf')
    normals = np.column_stack([np.cos(np.arange(8)*np.pi/4+np.pi/8),np.sin(np.arange(8)*np.pi/4+np.pi/8)])
    for node in assembled.graph.nodes_geometry:
        if not node.startswith(('electronics/','power_harness/')): continue
        mat, g = assembled.graph[node]
        mesh = assembled.geometry[g].copy(); mesh.apply_transform(mat)
        tris = mesh.triangles*1000
        for x, radius in [(26,8),(20,10.5),(0,10.5),(-20,10.5),(-26,8)]:
            lo,hi = x-1.5,x+1.5
            mask = (tris[:,:,0].min(1)<=hi)&(tris[:,:,0].max(1)>=lo)
            t=tris[mask]; v=t.reshape(-1,3)
            pts=[v[(v[:,0]>=lo)&(v[:,0]<=hi)]]
            for i,j in [(0,1),(1,2),(2,0)]:
                a,b=t[:,i],t[:,j]
                for plane in (lo,hi):
                    cross=(a[:,0]-plane)*(b[:,0]-plane)<0
                    aa,bb=a[cross],b[cross]
                    q=(plane-aa[:,0])/(bb[:,0]-aa[:,0])
                    pts.append(aa+q[:,None]*(bb-aa))
            pts=np.concatenate(pts)
            if len(pts):
                yz=pts[:,1:3].copy(); yz[:,1]/=.85
                gap=radius*np.cos(np.pi/8)-(yz@normals.T).max()
                assert gap>1e-6, (node,x,gap)
                clearance=min(clearance,gap)
    print(f'PASS: all new mesh triangles clear the five frame apertures (min normalized margin {clearance:.3f} mm)')
    assert np.allclose(assembled.extents, old.extents)
    # Check separation from the power core using conservative bounding boxes,
    # with the deliberate copper terminal contact excepted.
    pbound=power.scene.bounds+chassis.anchors['power'][:3,3]
    assert electronics.scene.bounds[0,2]+chassis.anchors['electronics'][2,3]>pbound[1,2]
    for name,mesh in harness.scene.geometry.items():
        sign=1 if name.startswith('positive') else -1
        if name.endswith('terminal_lead'): continue
        # Harness stays beyond the entire power module or above it.
        v=mesh.vertices
        assert np.all((sign*v[:,0]>pbound[1,0]+1e-7)|(v[:,2]>pbound[1,2]+1e-7))
    print('PASS: board and harness clear power-core envelope except intended terminal contacts')
    _,_,larger=build_stage_03(2)
    assert np.allclose(larger.extents,2*assembled.extents)
    for invalid in [0,-1,np.nan,np.inf]:
        try: create_electronics(invalid)
        except ValueError: pass
        else: raise AssertionError('invalid scale accepted')
    print('PASS: scale and invalid input checks')
    loaded=trimesh.load_scene((Path(__file__).resolve().parents[1]/'models')/'step_03_assembled.glb')
    assert len(loaded.geometry)==len(assembled.geometry)
    assert np.allclose(loaded.extents,assembled.extents,atol=1e-7)
    print('PASS: GLB round trip')


if __name__=='__main__': check()
