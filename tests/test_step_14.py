"""Stage 14: validate cellular film geometry, fit, materials and exports."""
from pathlib import Path
import json
import struct
import numpy as np
import trimesh
from shapely.geometry import Polygon
from step_13_wing_skeleton import make_wing_plan, wing_placements, create_wing_skeleton, CHANNELS
from step_14_wing_membranes import (create_wing_membrane, create_membrane_cell, membrane_thickness, build_stage_14, INSET)


def check():
    root=Path(__file__).resolve().parents[1]/'models'
    prior=trimesh.load_scene(root/'step_13_assembled.glb'); scene=trimesh.load_scene(root/'step_14_assembled.glb')
    plans=[make_wing_plan(side,pair) for side,pair in CHANNELS]
    skeletons={f'wing_skeleton_{p.pair}_{p.side}':create_wing_skeleton(p) for p in plans}
    modules={f'wing_membrane_{p.pair}_{p.side}':create_wing_membrane(p) for p in plans}
    assert len(modules)==4 and len(scene.geometry)==5671
    for n in prior.graph.nodes_geometry:
        a,g=prior.graph[n]; b,h=scene.graph[n]
        assert np.allclose(a,b,atol=1e-10) and np.array_equal(prior.geometry[g].vertices,scene.geometry[h].vertices) and np.array_equal(prior.geometry[g].faces,scene.geometry[h].faces),n
    assert np.allclose(prior.bounds,scene.bounds)
    print('PASS: all 4527 previous meshes, vertices, faces, transforms and assembly bounds preserved',flush=True)
    for n,m in scene.geometry.items():
        assert np.isfinite(m.vertices).all() and m.is_watertight and m.is_winding_consistent and m.volume>0,n
    print('PASS: 5671 finite watertight consistently wound positive-volume meshes',flush=True)
    placements=wing_placements(); max_gap=0.; thickness=[]
    for side,pair in CHANNELS:
        plan=make_wing_plan(side,pair); p=plan.points_mm; span=p[-1,0]
        sk=f'wing_skeleton_{pair}_{side}'; key=f'wing_membrane_{pair}_{side}'; mod=modules[key]
        assert len(mod.scene.geometry)==len(plan.cells)==286
        assert np.array_equal(mod.anchors['tip'],skeletons[sk].anchors['tip'])
        assert np.array_equal(mod.anchors['mount'],skeletons[sk].anchors['membrane'])
        for k,c in enumerate(plan.cells):
            name=f'cell_{k:03d}'; m=mod.scene.geometry[name]; n=len(c)
            a,b=m.vertices[:n]*1000,m.vertices[n:]*1000; mid=(a+b)/2
            t=a[:,2]-b[:,2]; thickness.extend(t)
            assert np.allclose(a[:,:2],b[:,:2]) and np.allclose(t,membrane_thickness(mid[:,0]/span))
            orig=p[list(c)]; expected=orig+INSET*(orig.mean(axis=0)-orig)
            assert np.allclose(mid,expected) or np.allclose(mid,expected[::-1])
            # Convex cell inset preserves footprint, and its full edge lies
            # within a conservative 0.08 mm minimum crossvein radius.
            outer=Polygon(orig[:,:2]); inner=Polygon(mid[:,:2]); assert outer.covers(inner)
            assert np.isclose(inner.area/outer.area,(1-INSET)**2)
            # 3D maximum endpoint displacement + half-thickness bounds the
            # distance between corresponding panel edges and vein centerlines.
            gap=np.linalg.norm(expected-orig,axis=1).max()+t.max()/2
            max_gap=max(max_gap,gap)
            assert gap<.08, (key,k,gap)
            material=m.visual.material
            assert material.alphaMode=='BLEND' and 0<material.baseColorFactor[3]<255
            transform,_=scene.graph[key+'/'+name]
            assert np.allclose(transform,placements[sk])
        bigger=create_wing_membrane(make_wing_plan(side,pair,2))
        assert np.allclose(bigger.scene.bounds,mod.scene.bounds*2)
    for pair in ('fore','hind'):
        left=modules[f'wing_membrane_{pair}_left']; right=modules[f'wing_membrane_{pair}_right']
        for n,m in left.scene.geometry.items():
            r=right.scene.geometry[n]; k=len(m.vertices)//2
            assert np.allclose(m.vertices[:k]*[1,-1,1],r.vertices[:k][::-1])
            assert np.allclose(m.vertices[k:]*[1,-1,1],r.vertices[k:][::-1])
            assert np.array_equal(m.visual.material.baseColorFactor,r.visual.material.baseColorFactor)
    print(f'PASS: 1144 aligned nonoverlapping cell footprints, {max_gap:.5f} mm conservative maximum edge displacement including half-thickness, thickness {min(thickness):.5f}–{max(thickness):.5f} mm, mirrored tints and geometry, proportional scaling',flush=True)
    for pts,span,scale in [(np.zeros((3,3)),1,1),(np.zeros((2,3)),1,1),(np.eye(3),0,1),(np.eye(3),1,-1)]:
        try: create_membrane_cell(pts,span,scale)
        except ValueError: pass
        else: raise AssertionError('invalid input accepted')
    print('PASS: invalid input rejection',flush=True)
    root=Path(__file__).resolve().parents[1]/'models'
    for f,count,films in [('step_14_membrane.glb',286,286),('step_14_membranes.glb',1144,1144),('step_14_assembled.glb',5671,1144)]:
        path=root/f; loaded=trimesh.load_scene(path); assert len(loaded.geometry)==count
        if count==5671: assert np.allclose(loaded.bounds,scene.bounds,atol=1e-7)
        # Inspect actual GLB JSON rather than trusting in-memory materials.
        with path.open('rb') as fp:
            assert fp.read(4)==b'glTF'; fp.read(8); size,kind=struct.unpack('<I4s',fp.read(8)); assert kind==b'JSON'; data=json.loads(fp.read(size))
        mats=data['materials']; film_ids={i for i,m in enumerate(mats) if 'clear film' in m.get('name','')}
        assert len(film_ids)==4
        for i in film_ids:
            m=mats[i]; assert m['alphaMode']=='BLEND' and 0<m['pbrMetallicRoughness']['baseColorFactor'][3]<1
        assert sum(p.get('material') in film_ids for m in data['meshes'] for p in m['primitives'])==films
    print('PASS: three GLB round trips and all exported membrane alpha-blending metadata',flush=True)

if __name__=='__main__': check()
