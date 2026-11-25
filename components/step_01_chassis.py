from dataclasses import dataclass
from pathlib import Path

import numpy as np
import trimesh


@dataclass
class Module:
    name: str
    scene: trimesh.Scene
    anchors: dict[str, np.ndarray]


def create_chassis(scale: float = 1.0) -> Module:
    """Build a bare thorax chassis; scale=1 uses the nominal design size.

    All mesh coordinates and anchor translations are in meters.
    +X points toward the head, +Y left, and +Z up.
    """
    if not np.isfinite(scale) or scale <= 0:
        raise ValueError("scale must be positive and finite")
    mm = 0.001 * scale
    scene = trimesh.Scene()
    anchors = {}

    # Physically based material parameters (no textures yet).
    titanium = trimesh.visual.material.PBRMaterial(
        name="Titanium", baseColorFactor=[0.58, 0.60, 0.63, 1.0],
        metallicFactor=0.95, roughnessFactor=0.34,
    )
    carbon = trimesh.visual.material.PBRMaterial(
        name="Carbon fiber placeholder", baseColorFactor=[0.08, 0.09, 0.10, 1.0],
        metallicFactor=0.0, roughnessFactor=0.45,
    )

    # Local helper: apply a material, then insert a uniquely named mesh.
    def add(name, mesh, material):
        mesh.visual = trimesh.visual.TextureVisuals(material=material)
        scene.add_geometry(mesh, node_name=name, geom_name=name)

    # Three hollow octagonal frames, plus two smaller end adapters.
    # Annuli start on the Z axis; rotate them onto the body (X) axis.
    for name, x, outer, inner in [
        ("front_adapter", 26, 12, 8),
        ("front_frame", 20, 14, 10.5),
        ("middle_frame", 0, 14, 10.5),
        ("rear_frame", -20, 14, 10.5),
        ("rear_adapter", -26, 12, 8),
    ]:
        ring = trimesh.creation.annulus(
            r_min=inner * mm, r_max=outer * mm,
            height=3 * mm, sections=8,
        )
        ring.apply_transform(trimesh.transformations.rotation_matrix(
            np.pi / 2, [0, 1, 0]
        ))
        ring.apply_scale([1, 1, 0.85])  # Slightly flatten the cross-section.
        ring.apply_translation([x * mm, 0, 0])
        add(name, ring, titanium)

    # Four longitudinal rails connect the frames; the lower pair is carbon.
    for side, y in [("left", 8), ("right", -8)]:
        for level, z in [("upper", 6), ("lower", -6)]:
            rail = trimesh.creation.box(extents=np.array([56, 3, 3]) * mm)
            rail.apply_translation(np.array([0, y, z]) * mm)
            add(f"{side}_{level}_rail", rail,
                titanium if level == "upper" else carbon)

    # A lower titanium spine reinforces the open chassis.
    spine = trimesh.creation.box(extents=np.array([56, 3, 3]) * mm)
    spine.apply_translation([0, 0, -9.5 * mm])
    add("central_spine", spine, titanium)

    # Named attachment frames: identity orientation for this first stage.
    # These are logical interfaces, not yet physical drilled mounting holes.
    def anchor(name, x, y, z):
        frame = np.eye(4)
        frame[:3, 3] = np.array([x, y, z]) * mm
        anchors[name] = frame

    anchor("mount", 0, 0, 0)
    anchor("power", 0, 0, -4)
    anchor("electronics", 0, 0, 5)
    anchor("transmission", 0, 0, 12)
    anchor("neck", 28, 0, 0)
    anchor("abdomen", -28, 0, 0)
    anchor("thorax_armor", 0, 0, 0)
    for pair, x in enumerate([20, 0, -20], start=1):
        for side, y in [("left", 11), ("right", -11)]:
            anchor(f"leg_{pair}_{side}", x, y, -6)

    return Module("chassis", scene, anchors)


if __name__ == "__main__":
    chassis = create_chassis()
    output = Path(__file__).resolve().parent / "step_01_chassis.glb"
    chassis.scene.export(str(output))
    print(f"Exported: {output}")
    print(f"Meshes: {len(chassis.scene.geometry)}")
    print(f"Anchors: {len(chassis.anchors)}")
    print("Dimensions (mm):", chassis.scene.extents * 1000)
