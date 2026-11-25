"""Stage 02: a removable power-core concept for the dragonfly.

Requires step_01_chassis.py in the same directory, numpy and trimesh.
All coordinates are meters; construction dimensions below are in millimeters.
This is visualization geometry, not a validated electrical battery design.
"""
from pathlib import Path

import numpy as np
import trimesh

from step_01_chassis import Module, create_chassis


def create_power_core(scale: float = 1.0) -> Module:
    """Return a power module whose local mount frame is at its origin.

    +X faces the head, +Y left, +Z up. Terminal anchor X axes point outward.
    The module fits the nominal chassis when both use the same scale.
    """
    if not np.isfinite(scale) or scale <= 0:
        raise ValueError("scale must be positive and finite")
    mm = 0.001 * scale
    scene = trimesh.Scene()

    # Physically based visual materials (no actual electrical simulation).
    def material(name, rgba, metallic, roughness, **kwargs):
        return trimesh.visual.material.PBRMaterial(
            name=name, baseColorFactor=rgba, metallicFactor=metallic,
            roughnessFactor=roughness, **kwargs)

    titanium = material("Power core titanium", [0.57, 0.60, 0.64, 1], .92, .32)
    copper = material("Copper terminals", [.83, .43, .19, 1], .9, .28)
    ceramic = material("Dark insulating ceramic", [.045, .055, .065, 1], 0, .42)
    cell = material("Cell wrapper", [.12, .20, .25, 1], .35, .36)
    glass = material("Inspection window polymer", [.45, .75, .85, .27], 0, .13,
                     alphaMode="BLEND", doubleSided=True)
    indicator = material("Cyan status strip", [.04, .55, .75, 1], .1, .28,
                         emissiveFactor=[.025, .38, .55])

    def add(name, mesh, mat):
        mesh.visual = trimesh.visual.TextureVisuals(material=mat)
        scene.add_geometry(mesh, node_name=name, geom_name=name)

    def cylinder(name, radius, length, x, mat):
        """Create an X-aligned cylinder centered at (x, 0, 0)."""
        mesh = trimesh.creation.cylinder(radius=radius * mm,
                                         height=length * mm, sections=64)
        mesh.apply_transform(trimesh.transformations.rotation_matrix(
            np.pi / 2, [0, 1, 0]))
        mesh.apply_translation([x * mm, 0, 0])
        add(name, mesh, mat)

    def annular_sector(name, inner, outer, length, start, end, mat, x=0):
        """Closed annular-sector mesh along X, including both cut faces.

        Angles are measured in the YZ plane from +Y toward +Z. The casing
        uses a large sector, leaving a real opening for the window insert.
        """
        angles = np.linspace(start, end, 65)
        vertices = []
        for theta in angles:
            for xx, rr in [(-length/2, inner), (-length/2, outer),
                           (length/2, outer), (length/2, inner)]:
                vertices.append([(xx+x)*mm, rr*np.cos(theta)*mm,
                                 rr*np.sin(theta)*mm])
        faces = []
        for i in range(len(angles)-1):
            for j in range(4):
                a, b = 4*i+j, 4*i+(j+1)%4
                c, d = b+4, a+4
                faces.extend([(a,b,c), (a,c,d)])
        last = 4*(len(angles)-1)
        faces.extend([(0,2,1),(0,3,2),
                      (last,last+1,last+2),(last,last+2,last+3)])
        mesh = trimesh.Trimesh(vertices=vertices, faces=faces, process=True)
        mesh.fix_normals()
        add(name, mesh, mat)

    def box(name, dims, pos, mat):
        mesh = trimesh.creation.box(extents=np.asarray(dims)*mm)
        mesh.apply_translation(np.asarray(pos)*mm)
        add(name, mesh, mat)

    # 1. Inner cell and protective casing, aligned with the chassis spine.
    cylinder("cell", 3.15, 30, 0, cell)
    # The opening spans 70 degrees around the top of the cylinder.
    lo, hi = np.deg2rad([55, 125])
    annular_sector("titanium_shell", 3.55, 4.0, 32, hi, lo+2*np.pi, titanium)
    # Slight angular clearances avoid coincident shell/window faces.
    annular_sector("inspection_window", 3.60, 3.96, 31.8,
                   lo+0.005, hi-0.005, glass)

    # 2. End collars, insulators and electrical contacts.
    for label, sign in [("positive", 1), ("negative", -1)]:
        # Collars close the casing without occupying the main cell interior.
        cylinder(f"{label}_end_cap", 4.0, 1.0, sign*16.5, titanium)
        cylinder(f"{label}_insulator", 2.45, 0.8, sign*17.4, ceramic)
        cylinder(f"{label}_terminal", 1.15, 1.4, sign*18.5, copper)

    # 3. A thin status strip inside the window, and two lower mounting pads.
    box("status_strip", [19, .65, .22], [0, 0, 3.26], indicator)
    for label, x in [("front", 10), ("rear", -10)]:
        # Bottom face at local Z=-4 mm, touching the chassis spine after mount.
        box(f"{label}_mount_pad", [3, 2.8, .6], [x, 0, -3.7], titanium)

    # 4. Logical connection frames for assembly and later wiring.
    anchors = {"mount": np.eye(4)}
    for label, sign in [("positive", 1), ("negative", -1)]:
        frame = np.eye(4) if sign == 1 else trimesh.transformations.rotation_matrix(
            np.pi, [0, 0, 1])
        frame[:3, 3] = [sign*19.2*mm, 0, 0]
        anchors[label] = frame
    return Module("power_core", scene, anchors)


def attach_module(parent: Module, parent_anchor: str, child: Module,
                  child_anchor: str = "mount") -> trimesh.Scene:
    """Return a NEW scene with the child attached; never mutate the inputs.

    Parent and child anchor matrices are expressed in their module's local
    coordinate frame. This helper supports this two-module build stage.
    """
    placement = parent.anchors[parent_anchor] @ np.linalg.inv(child.anchors[child_anchor])
    result = parent.scene.copy()
    for node in child.scene.graph.nodes_geometry:
        transform, geometry = child.scene.graph[node]
        name = f"{child.name}/{node}"
        if name in result.graph.nodes:
            raise ValueError(f"Duplicate node: {name}")
        result.add_geometry(child.scene.geometry[geometry].copy(),
                            node_name=name, geom_name=name,
                            transform=placement @ transform)
    return result


if __name__ == "__main__":
    output = Path(__file__).resolve().parent
    chassis = create_chassis()
    power = create_power_core()
    assembled = attach_module(chassis, "power", power)
    power.scene.export(str(output / "step_02_power_core.glb"))
    assembled.export(str(output / "step_02_assembled.glb"))
    print("Power meshes:", len(power.scene.geometry))
    print("Assembled meshes:", len(assembled.geometry))
    print("Power dimensions (mm):", np.round(power.scene.extents * 1000, 3))
    print("Assembly dimensions (mm):", np.round(assembled.extents * 1000, 3))
