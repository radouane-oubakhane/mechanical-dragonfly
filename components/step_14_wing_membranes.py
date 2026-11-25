"""Stage 14: thin transparent cellular membranes fitted to Stage 13 veins."""
from pathlib import Path
import numpy as np
import trimesh
from step_01_chassis import Module
from step_13_wing_skeleton import WingPlan, make_wing_plan, wing_placements, build_stage_13, CHANNELS

INSET = 0.006
THICKNESS_ROOT_MM = 0.045
THICKNESS_TIP_MM = 0.025


def membrane_thickness(span_fraction):
    """Nominal thickness in mm, tapering from root to tip."""
    return THICKNESS_ROOT_MM + (THICKNESS_TIP_MM - THICKNESS_ROOT_MM) * np.clip(span_fraction, 0, 1)


def create_membrane_cell(points_mm, span_mm, scale=1.0):
    """Convert three or four ordered cambered points (mm) to a closed mesh (m)."""
    p = np.asarray(points_mm, dtype=float)
    if p.shape not in ((3, 3), (4, 3)) or not np.isfinite(p).all():
        raise ValueError('expected three or four finite 3D points')
    if not np.isfinite(span_mm) or span_mm <= 0 or not np.isfinite(scale) or scale <= 0:
        raise ValueError('span and scale must be positive and finite')
    # Small inset avoids coincident side faces; structural veins cover seams.
    p = p + INSET * (p.mean(axis=0) - p)
    area = np.sum(p[:, 0] * np.roll(p[:, 1], -1) - p[:, 1] * np.roll(p[:, 0], -1))
    if abs(area) < 1e-12:
        raise ValueError('degenerate cell')
    # Both mirrored wings require upward-facing dorsal surfaces.
    if area < 0:
        p = p[::-1].copy()
    h = membrane_thickness(p[:, 0] / span_mm) / 2
    top = p.copy(); top[:, 2] += h
    bottom = p.copy(); bottom[:, 2] -= h
    n = len(p); faces = []
    for i in range(1, n - 1):
        faces.extend([(0, i, i+1), (n, n+i+1, n+i)])
    for i in range(n):
        j = (i+1) % n
        faces.extend([(i, n+i, n+j), (i, n+j, j)])
    return trimesh.Trimesh(vertices=np.vstack([top,bottom]) * (.001 * scale),
                           faces=np.asarray(faces), process=False)


def create_wing_membrane(plan: WingPlan) -> Module:
    """Create one transparent membrane module from the exact wing lattice."""
    if not isinstance(plan, WingPlan):
        raise TypeError('expected WingPlan')
    if not np.isfinite(plan.scale) or plan.scale <= 0:
        raise ValueError('scale must be positive and finite')
    scene = trimesh.Scene()
    # Fixed subtle tints evoke optical film, not physical interference.
    palette = [
        ('Neutral clear film', [.78,.88,.91,.19]),
        ('Cool clear film', [.70,.84,.91,.21]),
        ('Teal clear film', [.73,.90,.88,.20]),
        ('Violet clear film', [.82,.83,.94,.18]),
    ]
    mats = [trimesh.visual.material.PBRMaterial(
        name=name, baseColorFactor=rgba, metallicFactor=0.0,
        roughnessFactor=.16, alphaMode='BLEND', doubleSided=False)
        for name,rgba in palette]
    p=plan.points_mm; span=p[-1,0]
    for k,cell in enumerate(plan.cells):
        mesh=create_membrane_cell(p[list(cell)],span,plan.scale)
        tint=(k % 3)+1 if k % 7==0 else 0
        mesh.visual=trimesh.visual.TextureVisuals(material=mats[tint])
        name=f'cell_{k:03d}'
        scene.add_geometry(mesh,node_name=name,geom_name=name)
    tip=np.eye(4); tip[:3,3]=p[-1]*(.001*plan.scale)
    return Module(f'wing_membrane_{plan.pair}_{plan.side}',scene,
                  {'mount':np.eye(4),'tip':tip})


def build_stage_14(scale: float=1.0) -> tuple[dict[str,Module],trimesh.Scene]:
    """Attach all membrane modules to their Stage 13 skeleton anchors."""
    skeletons,scene=build_stage_13(scale)
    placements=wing_placements(scale); modules={}
    for side,pair in CHANNELS:
        key=f'wing_skeleton_{pair}_{side}'
        module=create_wing_membrane(make_wing_plan(side,pair,scale))
        modules[module.name]=module
        placement=placements[key] @ skeletons[key].anchors['membrane'] @ np.linalg.inv(module.anchors['mount'])
        for node in module.scene.graph.nodes_geometry:
            local,g=module.scene.graph[node]; name=f'{module.name}/{node}'
            scene.add_geometry(module.scene.geometry[g].copy(),node_name=name,geom_name=name,transform=placement@local)
    return modules,scene


if __name__=='__main__':
    out=Path(__file__).resolve().parents[1]/'models'; out.mkdir(exist_ok=True)
    modules,scene=build_stage_14(); membranes=trimesh.Scene()
    for n in scene.graph.nodes_geometry:
        if n.startswith('wing_membrane_'):
            t,g=scene.graph[n]
            membranes.add_geometry(scene.geometry[g].copy(),node_name=n,geom_name=n,transform=t)
    modules['wing_membrane_fore_left'].scene.export(str(out/'step_14_membrane.glb'))
    membranes.export(str(out/'step_14_membranes.glb'))
    scene.export(str(out/'step_14_assembled.glb'))
    print('Cells per wing:',len(modules['wing_membrane_fore_left'].scene.geometry))
    print('Assembly meshes:',len(scene.geometry),'dimensions (mm):',scene.extents*1000)
