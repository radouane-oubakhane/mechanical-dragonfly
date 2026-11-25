"""Step 19: dielectric transmission, volume absorption and thin-film optics.
Material-only upgrade of the Step 18 GLBs. No geometry or buffer edits.
All lengths are meters except thin-film coating thicknesses (nanometers).
"""
from dataclasses import dataclass
from pathlib import Path
import argparse
import json
from collections import Counter
from step_17_metals import ROOT, read_glb, write_glb, stats

@dataclass(frozen=True)
class OpticalFinish:
    ior: float
    transmission: float
    roughness: float
    thickness_m: float
    attenuation_distance_m: float
    coating_strength: float = 0.0
    coating_min_nm: float = 100.0
    coating_max_nm: float = 180.0

FINISHES = {
    'compound': OpticalFinish(1.52, .94, .09, .00022, .00075, .24, 105, 165),
    'ocellus': OpticalFinish(1.52, .96, .075, .00050, .0014, .20, 95, 155),
    'power_window': OpticalFinish(1.49, .97, .12, .00036, .004),
    'thorax_window': OpticalFinish(1.52, .98, .10, .00040, .006),
}
EXTENSIONS = ('KHR_materials_ior', 'KHR_materials_transmission', 'KHR_materials_volume', 'KHR_materials_iridescence')

def optical_family(material):
    """Explicit allow-list: excludes wing membranes and opaque sensor backs."""
    name = material.get('name', '').lower()
    if name.startswith('tinted optical glass ') and name[-1:] in '012345' and name[:-1] == 'tinted optical glass ': return 'compound'
    return {'ocellus tinted optical glass': 'ocellus', 'inspection window polymer': 'power_window', 'thorax inspection glazing': 'thorax_window'}.get(name)

def upgrade_material(material, family, coatings=True):
    """Mutate only the selected material. Fresnel is handled by the renderer.
    The absorption tint is derived from the original palette; values are artistic,
    not measured optical constants. Volume thickness is a representative path.
    """
    f = FINISHES[family]
    if material.get('alphaMode') != 'BLEND': raise ValueError('Expected Stage 18 alpha-blended optical material')
    p = material['pbrMetallicRoughness']
    rgb = p['baseColorFactor'][:3]
    # Use attenuation for depth tint instead of using a dark surface as glass.
    # Keep a weak surface tint, with a stronger transmission tint in the volume.
    p.update(baseColorFactor=[.86+.14*c for c in rgb]+[1.0], metallicFactor=0., roughnessFactor=f.roughness)
    material.update(alphaMode='OPAQUE', doubleSided=False)
    e = material.setdefault('extensions', {})
    e['KHR_materials_ior'] = {'ior': f.ior}
    e['KHR_materials_transmission'] = {'transmissionFactor': f.transmission}
    e['KHR_materials_volume'] = {'thicknessFactor': f.thickness_m,
        'attenuationDistance': f.attenuation_distance_m,
        'attenuationColor': [.20+.80*c for c in rgb]}
    if coatings and f.coating_strength:
        e['KHR_materials_iridescence'] = {'iridescenceFactor': f.coating_strength,
            'iridescenceIor': 1.38, 'iridescenceThicknessMinimum': f.coating_min_nm,
            'iridescenceThicknessMaximum': f.coating_max_nm}
    return material

def enrich(source, destination, coatings=True):
    """Write an optical variant without changing any vertex, texture or keyframe."""
    source, destination = Path(source), Path(destination)
    if source.resolve() == destination.resolve(): raise ValueError('Never overwrite source')
    d, raw = read_glb(source)
    if not d.get('textures'): raise ValueError('Expected Step 18 textured model')
    families = {i:f for i,m in enumerate(d['materials']) if (f:=optical_family(m))}
    if Counter(families.values()) != Counter(compound=6,ocellus=1,power_window=1,thorax_window=1):
        raise ValueError('Expected nine original optical materials')
    before=stats(d)
    for i,f in families.items(): upgrade_material(d['materials'][i], f, coatings)
    used=d.setdefault('extensionsUsed',[])
    for ext in EXTENSIONS[:3]+(EXTENSIONS[3:] if coatings else ()):
        if ext not in used: used.append(ext)
    instances=Counter(families[p['material']] for n in d['nodes'] if 'mesh' in n
                      for p in d['meshes'][n['mesh']]['primitives'] if p.get('material') in families)
    destination.parent.mkdir(parents=True, exist_ok=True)
    write_glb(destination,d,bytearray(raw))
    return dict(before=before,after=stats(d),optical_materials=len(families),optical_instances=dict(instances),
                coatings=coatings,texture_images=len(d['images']),source_bytes=source.stat().st_size,output_bytes=destination.stat().st_size)

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--animated',action='store_true');p.add_argument('--source',type=Path);p.add_argument('--output',type=Path)
    p.add_argument('--no-coatings',action='store_true');a=p.parse_args()
    kind='animated' if a.animated else 'static'
    source=a.source or ROOT/'models'/f'dragonfly_{kind}_balanced_carbon.glb'
    suffix='optics_uncoated' if a.no_coatings else 'optics'
    output=a.output or ROOT/'models'/f'dragonfly_{kind}_balanced_{suffix}.glb'
    print(json.dumps(enrich(source,output,not a.no_coatings),indent=2))
