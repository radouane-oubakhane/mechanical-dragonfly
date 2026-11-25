"""Step 20: thin-film interference textures for the four wing materials.
The optical coating thickness (nm) is distinct from the structural membrane
thickness (25–45 micrometers). glTF iridescence is evaluated by the renderer.
"""
from pathlib import Path
import argparse
import json
from collections import Counter
import numpy as np
from scipy.ndimage import gaussian_filter
from step_17_metals import ROOT, read_glb, write_glb, array, stats, face_charts, png_bytes

# Transmission, interference strength, min/max coating thickness (nm).
FINISHES = {
    'Neutral clear film': (.97, .30, 105., 205.),
    'Cool clear film': (.96, .38, 125., 235.),
    'Teal clear film': (.96, .38, 145., 255.),
    'Violet clear film': (.95, .42, 165., 280.),
}
TILE_MM = 18.0
EXTENSIONS = ('KHR_materials_ior', 'KHR_materials_transmission', 'KHR_materials_iridescence')


def texture_maps(size=512, seed=20):
    """Periodic linear-data maps: thickness G, roughness G, tangent normal RGB."""
    if not isinstance(size, int) or size < 64 or size > 2048 or size & (size-1):
        raise ValueError('Use a power of two from 64 to 2048')
    rng = np.random.default_rng(seed)
    v,u = np.mgrid[:size,:size] * (2*np.pi/size)
    smooth = gaussian_filter(rng.standard_normal((size,size)), size/35, mode='wrap')
    smooth /= max(smooth.std(), 1e-12)
    fine = gaussian_filter(rng.standard_normal((size,size)), max(.7,size/250), mode='wrap')
    fine /= max(fine.std(), 1e-12)
    # Broad thickness waves, weak manufacturing variation, no baked rainbow color.
    thickness = np.clip(.5 + .19*np.sin(2*u+v) + .14*np.cos(u-3*v) + .045*smooth, .02,.98)
    roughness = np.clip(.13 + .025*np.sin(3*u-2*v) + .012*fine, .075,.20)
    height = .16*np.sin(6*u+4*v) + .025*fine
    dx = (np.roll(height,-1,1)-np.roll(height,1,1))*.5
    dy = (np.roll(height,-1,0)-np.roll(height,1,0))*.5
    normal = np.stack([-dx*.32,-dy*.32,np.ones_like(dx)],axis=-1)
    normal /= np.linalg.norm(normal,axis=-1,keepdims=True)
    maps = {'thickness':np.stack([np.ones_like(thickness),thickness,np.ones_like(thickness)],axis=-1),
            'metalrough':np.stack([np.ones_like(roughness),roughness,np.zeros_like(roughness)],axis=-1),
            'normal':normal*.5+.5}
    return {key:np.round(value.clip(0,1)*255).astype(np.uint8) for key,value in maps.items()}


def generate_library(directory, size=512, seed=20):
    directory=Path(directory);directory.mkdir(parents=True,exist_ok=True)
    maps={key:png_bytes(value) for key,value in texture_maps(size,seed).items()}
    for key,data in maps.items(): (directory/f'wing_{key}.png').write_bytes(data)
    return maps


def upgrade_material(material, textures, coatings=True):
    """Use thin-walled transmission; do not conflate coating with bulk volume."""
    transmission,strength,lo,hi=FINISHES[material['name']]
    if material.get('alphaMode')!='BLEND': raise ValueError('Expected original wing film')
    p=material['pbrMetallicRoughness'];rgb=p['baseColorFactor'][:3]
    p.update(baseColorFactor=[.94+.06*c for c in rgb]+[1.],metallicFactor=0.,roughnessFactor=1.,
             metallicRoughnessTexture={'index':textures['metalrough']})
    material.update(alphaMode='OPAQUE',doubleSided=False,normalTexture={'index':textures['normal'],'scale':.25})
    e=material.setdefault('extensions',{})
    e['KHR_materials_ior']={'ior':1.46}
    e['KHR_materials_transmission']={'transmissionFactor':transmission}
    if coatings:
        e['KHR_materials_iridescence']={'iridescenceFactor':strength,'iridescenceIor':1.38,
            'iridescenceThicknessMinimum':lo,'iridescenceThicknessMaximum':hi,
            'iridescenceThicknessTexture':{'index':textures['thickness']}}
    return material


def enrich(source,destination,texture_dir=ROOT/'textures/wings',size=512,seed=20,coatings=True):
    """Append only wing UVs, normals, tangents and texture data; keep rig intact."""
    source,destination=Path(source),Path(destination)
    if source.resolve()==destination.resolve(): raise ValueError('Never overwrite source')
    doc,raw=read_glb(source); before=stats(doc); binary=bytearray(raw)
    ids={i for i,m in enumerate(doc['materials']) if m.get('name') in FINISHES}
    if len(ids)!=4 or any(doc['materials'][i].get('alphaMode')!='BLEND' for i in ids):
        raise ValueError('Expected four Stage 19 wing materials')
    if not doc.get('textures') or 'KHR_materials_volume' not in doc.get('extensionsUsed',[]):
        raise ValueError('Expected Stage 19 optics model')
    def view(data,target=None):
        binary.extend(b'\0'*(-len(binary)%4));offset=len(binary);binary.extend(data)
        entry=dict(buffer=0,byteOffset=offset,byteLength=len(data))
        if target:entry['target']=target
        doc['bufferViews'].append(entry);return len(doc['bufferViews'])-1
    def accessor(data,typ,component,target,bounds=False):
        data=np.ascontiguousarray(data)
        entry=dict(bufferView=view(data.tobytes(),target),componentType=component,count=len(data),type=typ)
        if bounds:entry.update(min=data.min(axis=0).tolist(),max=data.max(axis=0).tolist())
        doc['accessors'].append(entry);return len(doc['accessors'])-1
    sampler=len(doc['samplers']);doc['samplers'].append(dict(wrapS=10497,wrapT=10497,magFilter=9729,minFilter=9987))
    textures={}
    for role,data in generate_library(texture_dir,size,seed).items():
        if role=='thickness' and not coatings: continue
        name=f'wing_{role}';doc['images'].append(dict(name=name,bufferView=view(data),mimeType='image/png'))
        doc['textures'].append(dict(name=name,source=len(doc['images'])-1,sampler=sampler));textures[role]=len(doc['textures'])-1
    for i in ids: upgrade_material(doc['materials'][i],textures,coatings)
    for ext in EXTENSIONS[:2]+(EXTENSIONS[2:] if coatings else ()):
        if ext not in doc['extensionsUsed']: doc['extensionsUsed'].append(ext)
    count=0
    for mesh in doc['meshes']:
        for prim in mesh['primitives']:
            if prim.get('material') not in ids:continue
            if set(prim['attributes'])!={'POSITION'} or prim.get('mode',4)!=4: raise ValueError('Unsupported wing primitive')
            verts=array(doc,raw,prim['attributes']['POSITION']);indices=array(doc,raw,prim['indices'])
            p,f,uv,n,t=face_charts(verts,indices,TILE_MM)
            prim['attributes']={'POSITION':accessor(p,'VEC3',5126,34962,True),
                                'TEXCOORD_0':accessor(uv,'VEC2',5126,34962),
                                'NORMAL':accessor(n,'VEC3',5126,34962),
                                'TANGENT':accessor(t,'VEC4',5126,34962)}
            prim['indices']=accessor(f,'SCALAR',5125,34963);count+=1
    instances=Counter(doc['materials'][p['material']]['name'] for n in doc['nodes'] if 'mesh' in n
                      for p in doc['meshes'][n['mesh']]['primitives'] if p.get('material') in ids)
    if sum(instances.values())!=1144:raise ValueError('Expected 1,144 wing panel instances')
    destination.parent.mkdir(parents=True,exist_ok=True);write_glb(destination,doc,binary)
    return dict(before=before,after=stats(doc),wing_materials=len(ids),wing_primitives=count,
                wing_instances=dict(instances),texture_images=len(doc['images']),coatings=coatings,
                output_bytes=destination.stat().st_size)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--animated',action='store_true');p.add_argument('--source',type=Path);p.add_argument('--output',type=Path)
    p.add_argument('--size',type=int,default=512);p.add_argument('--seed',type=int,default=20);p.add_argument('--no-coatings',action='store_true');a=p.parse_args()
    kind='animated' if a.animated else 'static';source=a.source or ROOT/'models'/f'dragonfly_{kind}_balanced_optics.glb'
    suffix='wings_uncoated' if a.no_coatings else 'iridescent'
    output=a.output or ROOT/'models'/f'dragonfly_{kind}_balanced_{suffix}.glb'
    print(json.dumps(enrich(source,output,size=a.size,seed=a.seed,coatings=not a.no_coatings),indent=2))
