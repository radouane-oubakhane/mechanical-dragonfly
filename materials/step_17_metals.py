"""Step 17: deterministic tiled PBR metal maps, planar charts and GLB enrichment.
Run after Step 16. No change to source positions, triangle order, node transforms,
rig or animation. Metal faces get explicit flat normals and tangent frames.
"""
from pathlib import Path
from copy import deepcopy
import argparse
import io
import json
import sys
import numpy as np
from PIL import Image
from scipy.ndimage import gaussian_filter

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'optimization'))
from step_16_geometry import read_glb, write_glb, array, stats

# Mean roughness, variation, relief slope strength, tile length in mm, anisotropy.
FINISHES = {
    'titanium': (.34, .10, .10, 3.0, .38),
    'dark_titanium': (.39, .09, .09, 1.5, .25),
    'polished_steel': (.19, .05, .035, 2.0, .18),
    'hardened_steel': (.29, .08, .065, 2.5, .28),
    'copper': (.28, .07, .045, 1.4, .16),
    'bronze': (.35, .10, .075, 2.0, .23),
    'gold': (.25, .045, .025, 1.0, .12),
    'tin': (.33, .065, .045, 1.0, .12),
}


def metal_family(material):
    """Only classify named opaque metallic materials; leave glass etc. alone."""
    p = material.get('pbrMetallicRoughness', {})
    if material.get('alphaMode', 'OPAQUE') != 'OPAQUE' or p.get('metallicFactor', 0) < .7:
        return None
    name = material.get('name', '').lower()
    if 'titanium' in name: return 'dark_titanium' if 'dark' in name else 'titanium'
    if 'hardened' in name: return 'hardened_steel'
    if 'steel' in name or 'fastener' in name: return 'polished_steel'
    if 'copper' in name: return 'copper'
    if 'bronze' in name: return 'bronze'
    if 'gold' in name or name == 'electrical contacts': return 'gold'
    if 'tin solder' in name: return 'tin'
    return None


def texture_maps(family, size=512, seed=17):
    """Periodic, seeded horizontal brushing, fine scoring and micro roughness.
    Base map is a neutral tint multiplier (sRGB); ORM-like map stores roughness
    in green and metallic multiplier in blue. Normal map is tangent-space RGB.
    """
    if family not in FINISHES: raise ValueError('Unknown metal finish')
    if size < 64 or size > 2048 or size & (size-1): raise ValueError('Use a power of two from 64 to 2048')
    mean, variation, relief, _, _ = FINISHES[family]
    rng = np.random.default_rng(seed + list(FINISHES).index(family)*101)
    noise = rng.standard_normal((size,size))
    brush = gaussian_filter(noise, (.65, size/14), mode='wrap')
    brush /= max(brush.std(), 1e-9)
    micro = gaussian_filter(rng.standard_normal((size,size)), .65, mode='wrap')
    micro /= max(micro.std(), 1e-9)
    # Scoring: occasional narrow, continuous rows, modulated along their length.
    score = np.zeros(size)
    for row in rng.choice(size, max(3,size//42), replace=False): score[row] = rng.uniform(.5,1.0)
    score = gaussian_filter(score,.45,mode='wrap')[:,None]
    modulation = .55 + .45*np.cos(2*np.pi*np.arange(size)/size)[None,:]**2
    grooves = score*modulation
    field = np.clip(.65*brush+.20*micro-2.0*grooves,-3,3)
    color = np.clip(.965 + .012*field - .045*grooves,.80,1)
    base = np.repeat(color[:,:,None],3,axis=2)
    rough = np.clip(mean + variation*(.22*field + .35*grooves),.07,.8)
    mr = np.stack([np.ones_like(rough),rough,np.ones_like(rough)],axis=-1)
    height = .65*brush + .12*micro - 1.7*grooves
    dx = (np.roll(height,-1,1)-np.roll(height,1,1))*.5
    dy = (np.roll(height,-1,0)-np.roll(height,1,0))*.5
    # Image row direction is glTF +V. Tangent-space slope uses that convention.
    n = np.stack([-dx*relief,-dy*relief,np.ones_like(dx)],axis=-1)
    n /= np.linalg.norm(n,axis=-1,keepdims=True)
    return {k: np.round(np.clip(v,0,1)*255).astype(np.uint8) for k,v in
            [('basecolor',base),('metalrough',mr),('normal',n*.5+.5)]}


def png_bytes(rgb):
    buffer=io.BytesIO(); Image.fromarray(rgb).save(buffer,format='PNG',optimize=True)
    return buffer.getvalue()


def generate_library(directory, size=512, seed=17):
    directory=Path(directory);directory.mkdir(parents=True,exist_ok=True)
    return {family:{key: png_bytes(value) for key,value in texture_maps(family,size,seed).items()} for family in FINISHES}


def face_charts(vertices, indices, tile_mm):
    """Map each triangle to its dominant signed plane; UV units are physical.
    Return expanded face corners and consistent flat normal / tangent frames.
    No face moves, is removed, or changes winding.
    """
    p=vertices[indices.reshape(-1,3)].astype(np.float64)
    cross=np.cross(p[:,1]-p[:,0],p[:,2]-p[:,0]); norm=np.linalg.norm(cross,axis=1)
    if np.any(norm <= 0): raise ValueError('Degenerate triangle in metal mesh')
    n=cross/norm[:,None]
    axis=np.argmax(np.abs(n),axis=1)
    # Cyclic u,v axes: cross(u,v) points along positive dominant axis.
    basis=np.eye(3); u=basis[(axis+1)%3]; v=basis[(axis+2)%3]
    v*=np.where(n[np.arange(len(n)),axis]>=0,1.,-1.)[:,None]
    uv=np.stack([(p*u[:,None,:]).sum(axis=2),(p*v[:,None,:]).sum(axis=2)],axis=-1)/(tile_mm*.001)
    # Tangent = derivative of surface position w.r.t. U at fixed V.
    t=np.cross(v,n); t/=np.linalg.norm(t,axis=1)[:,None]
    tangent=np.column_stack([t,np.ones(len(t))])
    # Exact face-corner deduplication retains hard edges and chart seams.
    rows=np.concatenate([p.reshape(-1,3),uv.reshape(-1,2),np.repeat(n,3,axis=0),np.repeat(tangent,3,axis=0)],axis=1).astype('<f4')
    unique, inverse=np.unique(rows,axis=0,return_inverse=True)
    return unique[:,:3],inverse.reshape(-1).astype('<u4'),unique[:,3:5],unique[:,5:8],unique[:,8:12]


def enrich(source,destination,texture_dir=ROOT/'textures/metals',size=512,seed=17,anisotropy=True):
    source,destination=Path(source),Path(destination)
    if source.resolve()==destination.resolve(): raise ValueError('Never overwrite source')
    doc, raw=read_glb(source); before=stats(doc)
    if doc.get('textures'): raise ValueError('Start from untextured Step 16 GLB')
    binary=bytearray(raw);doc=deepcopy(doc)
    doc['images']=[];doc['textures']=[];doc['samplers']=[dict(wrapS=10497,wrapT=10497,magFilter=9729,minFilter=9987)]
    def view(data,target=None):
        binary.extend(b'\0'*(-len(binary)%4)); offset=len(binary);binary.extend(data)
        item=dict(buffer=0,byteOffset=offset,byteLength=len(data))
        if target: item['target']=target
        doc['bufferViews'].append(item);return len(doc['bufferViews'])-1
    def accessor(data,typ,component,target,bounds=False):
        data=np.ascontiguousarray(data);item=dict(bufferView=view(data.tobytes(),target),componentType=component,count=len(data),type=typ)
        if bounds: item.update(min=data.min(axis=0).tolist(),max=data.max(axis=0).tolist())
        doc['accessors'].append(item);return len(doc['accessors'])-1
    library=generate_library(texture_dir,size,seed); texture_indices={}
    for family,maps in library.items():
        texture_indices[family]={}
        for role,data in maps.items():
            name=f'{family}_{role}';(Path(texture_dir)/f'{name}.png').write_bytes(data)
            doc['images'].append(dict(name=name,bufferView=view(data),mimeType='image/png'))
            doc['textures'].append(dict(name=name,source=len(doc['images'])-1,sampler=0))
            texture_indices[family][role]=len(doc['textures'])-1
    families={}
    for i,m in enumerate(doc['materials']):
        family=metal_family(m)
        if family is None: continue
        families[i]=family; tex=texture_indices[family];p=m['pbrMetallicRoughness']
        # Map stores absolute roughness; base color and original metallic factor retain their identity.
        p.update(baseColorTexture=dict(index=tex['basecolor']),metallicRoughnessTexture=dict(index=tex['metalrough']),roughnessFactor=1.0)
        m['normalTexture']=dict(index=tex['normal'],scale=1.0)
        if anisotropy:
            m.setdefault('extensions',{})['KHR_materials_anisotropy']=dict(anisotropyStrength=FINISHES[family][4],anisotropyRotation=0.)
    if anisotropy and families: doc.setdefault('extensionsUsed',[]).append('KHR_materials_anisotropy')
    count=0
    for mesh in doc['meshes']:
        for prim in mesh['primitives']:
            family=families.get(prim.get('material'))
            if family is None: continue
            if set(prim['attributes'])!={'POSITION'} or prim.get('mode',4)!=4: raise ValueError('Unsupported source primitive')
            verts=array(doc,raw,prim['attributes']['POSITION']);ids=array(doc,raw,prim['indices'])
            p,f,uv,n,t=face_charts(verts,ids,FINISHES[family][3])
            prim['attributes']={'POSITION':accessor(p,'VEC3',5126,34962,True),'TEXCOORD_0':accessor(uv,'VEC2',5126,34962),'NORMAL':accessor(n,'VEC3',5126,34962),'TANGENT':accessor(t,'VEC4',5126,34962)}
            prim['indices']=accessor(f,'SCALAR',5125,34963);count+=1
    destination.parent.mkdir(parents=True,exist_ok=True);write_glb(destination,doc,binary)
    return dict(before=before,after=stats(doc),textured_materials=len(families),textured_primitives=count,texture_images=len(doc['images']),source_bytes=source.stat().st_size,output_bytes=destination.stat().st_size,anisotropy=anisotropy)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--animated',action='store_true');p.add_argument('--mode',choices=['ultra','balanced','performance'],default='balanced');p.add_argument('--source',type=Path);p.add_argument('--output',type=Path);p.add_argument('--size',type=int,default=512);p.add_argument('--seed',type=int,default=17);p.add_argument('--no-anisotropy',action='store_true');a=p.parse_args()
    kind='animated' if a.animated else 'static';source=a.source or ROOT/'models'/((('dragonfly_animated.glb' if a.animated else 'step_15_assembled.glb')) if a.mode=='ultra' else f'dragonfly_{kind}_{a.mode}.glb')
    output=a.output or ROOT/'models'/f'dragonfly_{kind}_{a.mode}_metals.glb'
    print(json.dumps(enrich(source,output,size=a.size,seed=a.seed,anisotropy=not a.no_anisotropy),indent=2))
