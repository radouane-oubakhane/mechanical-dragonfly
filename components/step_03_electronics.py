"""Stage 03: modular circuit board and power wiring for the dragonfly.

Run with Python, numpy and trimesh. Keep stages 01 and 02 beside this file.
All public anchors and geometry use meters; design dimensions are in mm.
Visual concept only: no circuit, electrical safety or flight validation.
"""
from pathlib import Path
import numpy as np
import trimesh
from step_01_chassis import Module, create_chassis
from step_02_power_core import create_power_core, attach_module


def _material(name, color, metallic=0.0, roughness=0.4):
    return trimesh.visual.material.PBRMaterial(name=name, baseColorFactor=color,
                                               metallicFactor=metallic,
                                               roughnessFactor=roughness)


def _add(scene, name, mesh, material):
    mesh.visual = trimesh.visual.TextureVisuals(material=material)
    scene.add_geometry(mesh, node_name=name, geom_name=name)


def _frame(position, direction=1):
    """A coordinate frame whose local +X points toward the chosen end."""
    result = np.eye(4)
    if direction == -1:
        result[:3, :3] = np.diag([-1, -1, 1])
    result[:3, 3] = position
    return result


def _check_scale(scale):
    if not np.isfinite(scale) or scale <= 0:
        raise ValueError('scale must be positive and finite')
    return .001 * scale


def create_electronics(scale: float = 1.0) -> Module:
    """Create the electronics module, with mount at the local origin.

    Inputs: scale (positive float).
    Output: Module with PCB geometry and named 4x4 connection frames.
    Mount to chassis.anchors['electronics']; no other modules are modified.
    """
    mm = _check_scale(scale)
    scene = trimesh.Scene()
    pcb = _material('Dark green FR4 solder mask', [.035, .17, .12, 1], .05, .48)
    gold = _material('Gold plated contacts', [.85, .61, .23, 1], .85, .3)
    black = _material('Epoxy IC package', [.035, .04, .05, 1], .03, .45)
    silver = _material('Tin solder', [.65, .68, .71, 1], .8, .32)
    ceramic = _material('Ceramic capacitors', [.58, .42, .26, 1], 0, .52)
    titanium = _material('Titanium PCB brackets', [.55, .58, .62, 1], .9, .35)
    ribbon = _material('Amber flex circuit', [.55, .23, .055, 1], .15, .42)

    def box(name, dims, pos, mat):
        mesh = trimesh.creation.box(extents=np.asarray(dims) * mm)
        mesh.apply_translation(np.asarray(pos) * mm)
        _add(scene, name, mesh, mat)

    def cylinder(name, radius, height, pos, mat):
        mesh = trimesh.creation.cylinder(radius=radius, height=height, sections=24)
        mesh.apply_scale(mm)
        mesh.apply_translation(np.asarray(pos)*mm)
        _add(scene, name, mesh, mat)

    # 1. The board slides through the central frame's opening.
    box('pcb_substrate', [35, 9.5, .65], [0, 0, 0], pcb)

    # Brackets bridge the PCB edge and the upper chassis rails in open bays.
    for i, x in enumerate([-10, 10]):
        for side, sign in [('left', 1), ('right', -1)]:
            box(f'bracket_{i}_{side}', [3.5, 1.75, .65], [x, sign*5.625, 0], titanium)
            # Small fastener heads sit on top of the bracket.
            cylinder(f'bracket_screw_{i}_{side}', .46, .18, [x, sign*5.6, .415], silver)

    # 2. Main processor and power-management ICs, with visible contact pins.
    for label, x in [('controller', 7), ('power_management', -7)]:
        box(label, [5, 4, .85], [x, 0, .75], black)
        for sign in [-1, 1]:
            for j, dx in enumerate(np.linspace(-2.0, 2.0, 8)):
                box(f'{label}_pin_{sign}_{j}', [.21, .75, .14],
                    [x+dx, sign*2.27, .395], silver)
        # Gold orientation marker on the package top.
        cylinder(f'{label}_pin1', .22, .04, [x-1.8, 1.4, 1.195], gold)

    # 3. Discrete surface-mount components along each board bay.
    for i, x in enumerate([-14, -12, -4, 4, 12, 14]):
        for sign in [-1, 1]:
            y = sign*3.55
            box(f'capacitor_{i}_{sign}', [1.1, .65, .48], [x, y, .565], ceramic)
            for end in [-1, 1]:
                box(f'capacitor_{i}_{sign}_contact_{end}', [.16, .65, .09],
                    [x+end*.47, y, .37], silver)

    # 4. Decorative exposed conductor traces in the vacant center of the board.
    # They are not a routed or electrically validated PCB netlist.
    for i, y in enumerate([-3.6, -2.7, -1.8, -.9, .9, 1.8, 2.7, 3.6]):
        box(f'center_trace_{i}', [2.5, .12, .025], [0, y, .3375], gold)
    for sign in [-1, 1]:
        for i, y in enumerate([-2.8, -1.4, 1.4, 2.8]):
            cylinder(f'via_{sign}_{i}', .22, .045, [sign*11, y, .3475], gold)

    # 5. Flex headers for later head, actuator and abdomen modules.
    anchors = {'mount': np.eye(4)}
    for label, x, sign in [('head_data', 12.5, 1), ('abdomen_data', -12.5, -1)]:
        box(label+'_header', [2.2, 2.4, .55], [x, 0, .60], black)
        box(label+'_flex', [1.2, 2.1, .12], [x+sign*1.4, 0, .73], ribbon)
        for j, y in enumerate(np.linspace(-.85, .85, 6)):
            box(f'{label}_contact_{j}', [.7, .14, .04], [x+sign*1.55, y, .81], gold)
        anchors[label] = _frame(np.array([x+sign*2, 0, .79])*mm, sign)

    for label, x, sign in [('actuator_fore_bus', 3, 1), ('actuator_hind_bus', -3, -1)]:
        box(label+'_header', [.75, 2.2, .5], [x, 0, .575], black)
        anchors[label] = _frame(np.array([x, 0, .825])*mm, sign)

    # 6. Power input connectors at opposite PCB ends. Their outer faces
    # align with the two harness endpoint frames.
    for label, sign in [('positive', 1), ('negative', -1)]:
        y = sign*1.5
        box(f'power_{label}_connector', [1.8, 2.1, 1.05], [sign*16.6, y, .85], black)
        anchors['power_'+label] = _frame(np.array([sign*17.5, y, .85])*mm, sign)

    return Module('electronics', scene, anchors)


def _round_path(points, radius, subdivisions=10):
    """Round polyline corners with quadratic Bezier bends."""
    points = np.asarray(points, dtype=float)
    out = [points[0]]
    for a, b, c in zip(points[:-2], points[1:-1], points[2:]):
        u, v = a-b, c-b
        d = min(radius, .45*np.linalg.norm(u), .45*np.linalg.norm(v))
        start, end = b+d*u/np.linalg.norm(u), b+d*v/np.linalg.norm(v)
        for t in np.linspace(0, 1, subdivisions):
            out.append((1-t)**2*start + 2*(1-t)*t*b + t*t*end)
    out.append(points[-1])
    return np.array(out)


def _tube(points, radius, sections=16):
    """Sweep a closed circular tube along a non-degenerate 3D path."""
    points = np.asarray(points)
    tangents = np.gradient(points, axis=0)
    tangents /= np.linalg.norm(tangents, axis=1)[:, None]
    # These harness routes are mostly X/Z, so Y is a stable reference.
    ref = np.array([0., 1., 0.])
    normals = np.cross(tangents, ref)
    normals /= np.linalg.norm(normals, axis=1)[:, None]
    binormals = np.cross(tangents, normals)
    angles = np.linspace(0, 2*np.pi, sections, endpoint=False)
    rings = points[:, None, :] + radius*(np.cos(angles)[None,:,None]*normals[:,None,:]
                                         + np.sin(angles)[None,:,None]*binormals[:,None,:])
    vertices = np.vstack([rings.reshape(-1, 3), points[0], points[-1]])
    faces = []
    for i in range(len(points)-1):
        for j in range(sections):
            a, b = i*sections+j, i*sections+(j+1)%sections
            c, d = b+sections, a+sections
            faces.extend([(a,b,c), (a,c,d)])
    end = (len(points)-1)*sections
    for j in range(sections):
        k = (j+1)%sections
        faces.extend([(len(vertices)-2,k,j), (len(vertices)-1,end+j,end+k)])
    mesh = trimesh.Trimesh(vertices=vertices, faces=faces, process=True)
    mesh.fix_normals()
    return mesh


def create_power_harness(chassis: Module, power: Module, electronics: Module,
                         scale: float = 1.0) -> Module:
    """Connect actual terminal and PCB anchor positions in chassis space.

    Returns a root-local Module. Modules must use the same scale and the
    stage-01 mounting layout. Cables turn in the gaps between end frames.
    """
    mm = _check_scale(scale)
    scene = trimesh.Scene()
    copper = _material('Exposed copper lead', [.82,.42,.18,1], .9,.3)
    red = _material('Positive cable insulation', [.4,.025,.02,1], 0,.42)
    black = _material('Negative cable insulation', [.025,.028,.035,1], 0,.42)
    p = chassis.anchors['power'] @ np.linalg.inv(power.anchors['mount'])
    e = chassis.anchors['electronics'] @ np.linalg.inv(electronics.anchors['mount'])
    anchors = {'mount': np.eye(4)}
    for label, sign, material in [('positive', 1, red), ('negative', -1, black)]:
        start_frame = p @ power.anchors[label]
        end_frame = e @ electronics.anchors['power_'+label]
        start, end = start_frame[:3,3], end_frame[:3,3]
        a = start + start_frame[:3,0]*.55*mm
        b = end + end_frame[:3,0]*.4*mm
        # Route outward, up in the open end bay, then inward to the board.
        turn_x = sign*22.8*mm
        path = _round_path([a, [turn_x, start[1], start[2]],
                           [turn_x, end[1], end[2]], b], .85*mm)
        _add(scene, label+'_terminal_lead', _tube([start,a], .21*mm), copper)
        _add(scene, label+'_insulated_wire', _tube(path, .30*mm), material)
        _add(scene, label+'_pcb_lead', _tube([b,end], .21*mm), copper)
        anchors[label+'_start'] = start_frame
        anchors[label+'_end'] = end_frame
    return Module('power_harness', scene, anchors)


def build_stage_03(scale: float = 1.0) -> tuple[Module, Module, trimesh.Scene]:
    """Build the new modules and a cumulative scene without mutating prior stages."""
    chassis, power = create_chassis(scale), create_power_core(scale)
    electronics = create_electronics(scale)
    harness = create_power_harness(chassis, power, electronics, scale)
    prior = attach_module(chassis, 'power', power)
    # Wrap the cumulative scene while retaining the original root anchors.
    combined = attach_module(Module('stage_02', prior, chassis.anchors),
                             'electronics', electronics)
    combined = attach_module(Module('stage_03', combined, chassis.anchors), 'mount', harness)
    return electronics, harness, combined


if __name__ == '__main__':
    out = Path(__file__).resolve().parent
    electronics, harness, assembled = build_stage_03()
    electronics.scene.export(str(out/'step_03_electronics.glb'))
    harness.scene.export(str(out/'step_03_power_harness.glb'))
    assembled.export(str(out/'step_03_assembled.glb'))
    print('Electronics meshes:', len(electronics.scene.geometry))
    print('Harness meshes:', len(harness.scene.geometry))
    print('Assembled meshes:', len(assembled.geometry))
    print('Assembly dimensions (mm):', np.round(assembled.extents*1000, 3))
