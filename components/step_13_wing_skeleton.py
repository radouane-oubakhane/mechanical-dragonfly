"""Stage 13: four biomimetic mechanical wing frameworks.

WingPlan is the shared geometric contract for Step 14: vertices and cells
are in wing-local nominal millimeters; scale converts to exported meters.
Local X is outward, Z is dorsal; local Y is mirrored between sides so the
assembled wing pair is symmetric in the body coordinate system.
"""
from dataclasses import dataclass
from pathlib import Path
import numpy as np
import trimesh
from step_01_chassis import Module, create_chassis
from step_04_transmission import create_transmission
from step_05_actuators import CHANNELS, create_wing_actuator
from step_06_wing_joints import create_wing_joint
from step_12_legs import build_stage_12, swept_wire


@dataclass(frozen=True)
class WingPlan:
    """Shared geometry; each cell is an ordered tuple of vertex indices.

    `outline` follows the perimeter; `longitudinal` holds continuous vein
    paths; `crossveins` contains undirected pairs of vertex indices.
    Use `points_mm * (.001 * scale)` to get physical local coordinates.
    """
    side: str
    pair: str
    scale: float
    points_mm: np.ndarray
    outline: tuple[int, ...]
    cells: tuple[tuple[int, ...], ...]
    longitudinal: tuple[tuple[int, ...], ...]
    crossveins: tuple[tuple[int, int], ...]


def make_wing_plan(side: str, pair: str, scale: float=1.0) -> WingPlan:
    """Construct a deterministic cellular wing plan with a cambered surface."""
    if side not in ('left','right') or pair not in ('fore','hind'):
        raise ValueError('side must be left/right and pair must be fore/hind')
    if not np.isfinite(scale) or scale <= 0:
        raise ValueError('scale must be positive and finite')
    # The existing sockets sit 25.525 mm from the centerline. The forewing
    # tip is 174.475 mm farther out, giving a nominal 400 mm wingspan.
    span=174.475 if pair=='fore' else 169.0
    knots=np.array([0,.025,.065,.16,.30,.47,.62,.78,.90,1.])
    if pair=='fore':
        leading=[-.5,-1,-5,-12,-19,-23,-24,-22,-16,-10]
        trailing=[.5,1,5,9,9,6,3,0,-4,-10]
    else:
        leading=[-.5,-1,-3,-8,-9,-7,-5,-1,4,10]
        trailing=[.5,1,9,19,26,28,26,22,17,10]
    # A dense, slightly irregular cellular lattice. The root collar uses
    # just the first few millimeters; the distal tip is a shared vertex.
    stations=np.r_[0.,.025,np.linspace(.065,.96,33)]
    bands=np.array([0,.11,.24,.39,.55,.70,.85,1.])
    n=len(bands); points=[]; sign=1 if side=='left' else -1
    rng=np.random.default_rng(1301 if pair=='fore' else 1302)
    for i,t in enumerate(stations):
        a,b=np.interp(t,knots,leading),np.interp(t,knots,trailing)
        for j,f in enumerate(bands):
            # Preserve exact outline; perturb only internal chord fractions.
            q=f+(rng.uniform(-.025,.025) if i>1 and 0<j<n-1 else 0)
            camber=.95*np.sin(np.pi*t)*np.sin(np.pi*q)+.25*np.sin(np.pi*t)
            points.append((span*t,sign*(a+(b-a)*q),camber))
    tip=len(points); points.append((span,sign*leading[-1],0.))
    points=np.asarray(points); points.setflags(write=False)
    longitudinal=tuple(tuple(i*n+j for i in range(len(stations)))+(tip,) for j in range(n))
    outline=tuple(i*n for i in range(len(stations)))+(tip,)+tuple(i*n+n-1 for i in reversed(range(len(stations))))+tuple(range(n-2,0,-1))
    cells=[]; cross=set()
    def edge(a,b): cross.add(tuple(sorted((a,b))))
    for i in range(len(stations)):
        for j in range(n-1):
            a=i*n+j; b=a+1; edge(a,b)
            if i==len(stations)-1:
                cells.append((a,tip,b)); continue
            c=(i+1)*n+j; d=c+1
            # Branching diagonal braces in selected distal cells, leaving
            # other cells quadrilateral. The same cells define membrane faces.
            if i>=5 and (i+2*j)%5==0:
                if (i+j)%2:
                    edge(a,d); cells.extend(((a,c,d),(a,d,b)))
                else:
                    edge(b,c); cells.extend(((a,c,b),(b,c,d)))
            else:
                cells.append((a,c,d,b))
    return WingPlan(side,pair,float(scale),points,outline,tuple(cells),longitudinal,tuple(sorted(cross)))


def create_wing_skeleton(plan: WingPlan) -> Module:
    """Generate continuous spars, individual crossveins and root hardware."""
    if not isinstance(plan,WingPlan): raise TypeError('expected WingPlan')
    mm=.001*plan.scale; p=plan.points_mm; scene=trimesh.Scene()
    def mat(name,rgb,metal,rough):
        return trimesh.visual.material.PBRMaterial(name=name,baseColorFactor=[*rgb,1],metallicFactor=metal,roughnessFactor=rough)
    ti=mat('Wing satin titanium',[.52,.56,.60],.92,.32)
    steel=mat('Wing polished steel',[.72,.75,.78],.96,.23)
    carbon=mat('Wing carbon composite',[.055,.065,.075],.08,.43)
    bronze=mat('Wing flexible bronze alloy',[.50,.31,.15],.78,.35)
    cyan=mat('Wing root sensor index',[.08,.56,.65],.25,.29)
    def add(name,mesh,material):
        mesh.apply_scale(mm); mesh.visual=trimesh.visual.TextureVisuals(material=material)
        scene.add_geometry(mesh,node_name=name,geom_name=name)
    def box(name,size,center,material):
        mesh=trimesh.creation.box(extents=size); mesh.apply_translation(center); add(name,mesh,material)
    def tube(name,a,b,r,material):
        add(name,swept_wire(np.array([a,b]),r,sides=10),material)
    # Root tongue mates to the Stage 06 socket's outward face.
    box('root_titanium_tongue',[3.4,1.1,.44],[1.7,0,0],ti)
    for s in (-1,1):
        box(f'root_composite_cheek_{s}',[2.6,.20,.65],[1.5,s*.65,0],carbon)
        for x in (.8,2.2):
            tube(f'root_fastener_{s}_{x}',[x,s*.40,.22],[x,s*.40,.36],.115,steel)
    box('root_encoder_index',[.65,.28,.08],[2.75,0,.27],cyan)
    # Strong leading spar and lighter trailing edge; internal veins are
    # continuous paths terminating at the shared tip.
    for j,path in enumerate(plan.longitudinal):
        radius=[.28,.21,.15,.14,.17,.12,.12,.14][j]
        material=ti if j in (0,4,7) else carbon
        add(f'longitudinal_vein_{j:02d}',swept_wire(p[list(path)],radius,sides=12),material)
    for k,(a,b) in enumerate(plan.crossveins):
        t=(p[a,0]+p[b,0])/(2*p[-1,0]); r=.08 if t>.5 else .11
        tube(f'crossvein_{k:03d}',p[a],p[b],r,bronze if k%7==0 else ti)
    # Reinforce the root-to-wing transition; braces follow the first station.
    for j in (0,2,5,7):
        a=p[j]; b=p[8+j]; tube(f'root_reinforcement_{j}',a,b,.20,ti)
    # Darkened pterostigma-like structural reinforcement near the leading
    # edge of the tip, deliberately kept narrow so it does not fill cells.
    path=plan.longitudinal[0]; a,b=p[path[-5]],p[path[-3]]
    tube('distal_leading_reinforcement',a,b,.36,carbon)
    anchors={'mount':np.eye(4),'membrane':np.eye(4)}
    tip=np.eye(4); tip[:3,3]=p[-1]*mm; anchors['tip']=tip
    return Module(f'wing_skeleton_{plan.pair}_{plan.side}',scene,anchors)


def wing_placements(scale: float=1.0) -> dict[str,np.ndarray]:
    """Resolve existing attachment chain from the chassis to each wing."""
    t=create_chassis(scale).anchors['transmission']; transmission=create_transmission(scale)
    out={}
    for side,pair in CHANNELS:
        actuator=create_wing_actuator(side,pair,scale); joint=create_wing_joint(side,pair,scale)
        out[f'wing_skeleton_{pair}_{side}']=t@transmission.anchors[actuator.name]@actuator.anchors['output']@joint.anchors['wing']
    return out


def build_stage_13(scale: float=1.0) -> tuple[dict[str,Module],trimesh.Scene]:
    """Attach all four frameworks without changing any previous geometry."""
    _,scene=build_stage_12(scale); modules={}; placements=wing_placements(scale)
    for side,pair in CHANNELS:
        module=create_wing_skeleton(make_wing_plan(side,pair,scale)); modules[module.name]=module
        placement=placements[module.name]@np.linalg.inv(module.anchors['mount'])
        for node in module.scene.graph.nodes_geometry:
            local,g=module.scene.graph[node]; name=f'{module.name}/{node}'
            scene.add_geometry(module.scene.geometry[g].copy(),node_name=name,geom_name=name,transform=placement@local)
    return modules,scene


if __name__=='__main__':
    out=Path(__file__).resolve().parents[1]/'models'; out.mkdir(exist_ok=True)
    modules,scene=build_stage_13(); wings=trimesh.Scene()
    for n in scene.graph.nodes_geometry:
        if n.startswith('wing_skeleton_'):
            t,g=scene.graph[n]; wings.add_geometry(scene.geometry[g].copy(),node_name=n,geom_name=n,transform=t)
    modules['wing_skeleton_fore_left'].scene.export(str(out/'step_13_wing.glb'))
    wings.export(str(out/'step_13_wings.glb')); scene.export(str(out/'step_13_assembled.glb'))
    print('Meshes per wing:',len(modules['wing_skeleton_fore_left'].scene.geometry))
    print('Cells per wing:',len(make_wing_plan('left','fore').cells))
    print('Assembly meshes:',len(scene.geometry),'dimensions (mm):',scene.extents*1000)
