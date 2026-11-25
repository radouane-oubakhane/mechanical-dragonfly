"""Regression, placement and sampled gear-clearance tests (Shapely for 2D)."""
from pathlib import Path
import numpy as np
import trimesh
from shapely.geometry import Polygon
from shapely.affinity import translate
from step_01_chassis import create_chassis
from step_03_electronics import build_stage_03
from step_04_transmission import build_stage_04, create_transmission, gear_outline


def check():
    _,_,prior=build_stage_03()
    transmission,assembled=build_stage_04()
    assert len(assembled.geometry)==len(prior.geometry)+len(transmission.scene.geometry)
    for node in prior.graph.nodes_geometry:
        a,g=prior.graph[node]; b,h=assembled.graph[node]
        assert np.array_equal(a,b)
        assert np.array_equal(prior.geometry[g].vertices,assembled.geometry[h].vertices)
        assert np.array_equal(prior.geometry[g].faces,assembled.geometry[h].faces)
    print('PASS: stages 01–03 geometry and transforms preserved')
    for name,mesh in assembled.geometry.items():
        assert np.isfinite(mesh.vertices).all(),name
        assert mesh.is_watertight and mesh.is_winding_consistent and mesh.volume>0,name
    print('PASS: finite, watertight, consistently wound positive-volume meshes')
    t=create_chassis().anchors['transmission']
    assert len(transmission.anchors)==13
    for name,a in transmission.anchors.items():
        assert a.shape==(4,4) and np.allclose(a[3],[0,0,0,1])
        assert np.allclose(a[:3,:3].T@a[:3,:3],np.eye(3))
        assert np.isclose(np.linalg.det(a[:3,:3]),1)
    for pair,x in [('fore',13),('hind',-13)]:
        for side,sy in [('left',1),('right',-1)]:
            name=pair+'_'+side
            a=transmission.anchors['actuator_'+name]
            assert np.allclose(a[:3,0],[0,sy,0])
            assert np.allclose((t@a)[:3,3]*1000,[x,sy*10.5,14.5])
            plate=transmission.scene.geometry[name+'_actuator_plate']
            assert np.isclose(a[1,3],plate.bounds[1 if sy==1 else 0,1])
            s=transmission.anchors['output_shaft_'+name]
            d=transmission.anchors['driver_shaft_'+name]
            assert np.isclose(np.linalg.norm((s-d)[:2,3]),.006)
            coupling=transmission.scene.geometry[name+'_output_coupling']
            assert np.isclose(s[2,3],coupling.bounds[1,2])
    print('PASS: rigid frames, four actuator plate interfaces and shaft center distances')
    # All new parts except intentional chassis mounting feet sit above the old scene.
    for n in transmission.scene.graph.nodes_geometry:
        m,g=transmission.scene.graph[n]; mesh=transmission.scene.geometry[g].copy()
        mesh.apply_transform(t@m)
        if n.startswith('chassis_foot_'):
            assert mesh.bounds[0,2]<prior.bounds[1,2]<mesh.bounds[1,2]
        else:
            assert mesh.bounds[0,2]>prior.bounds[1,2]-1e-10,n
    print('PASS: dorsal clearance above prior geometry (except mounting feet)')
    # The input and output gears counter-rotate at the nominal 1:1 ratio.
    # Check both the forward and aft layouts across one complete tooth pitch.
    min_gap=float('inf'); samples=0
    for direction in [1,-1]:
        for phase in np.linspace(0,2*np.pi/12,121):
            a=Polygon(gear_outline(phase=phase))
            b=translate(Polygon(gear_outline(phase=np.pi/12-phase)),xoff=6*direction)
            assert a.is_valid and b.is_valid
            assert a.intersection(b).area<1e-10
            min_gap=min(min_gap,a.distance(b)); samples+=1
    print(f'PASS: {samples} sampled gear poses, minimum planar clearance {min_gap:.3f} mm')
    _,large=build_stage_04(2)
    assert np.allclose(large.extents,2*assembled.extents)
    for invalid in [0,-1,np.nan,np.inf]:
        try: create_transmission(invalid)
        except ValueError: pass
        else: raise AssertionError('invalid scale accepted')
    print('PASS: proportional scale and invalid scale rejection')
    for file,scene in [('step_04_transmission.glb',transmission.scene),('step_04_assembled.glb',assembled)]:
        loaded=trimesh.load_scene((Path(__file__).resolve().parents[1]/'models')/file)
        assert len(loaded.geometry)==len(scene.geometry)
        assert np.allclose(loaded.bounds,scene.bounds,atol=1e-7)
    print('PASS: both GLB exports reimport with matching geometry counts and bounds')

if __name__=='__main__': check()
