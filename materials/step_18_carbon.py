"""Step 18: deterministic woven carbon-composite PBR textures for Step 17 GLBs."""
from pathlib import Path
import argparse
import json
import numpy as np
from scipy.ndimage import gaussian_filter
from step_17_metals import ROOT, read_glb, write_glb, array, stats, face_charts, png_bytes

# Tile length (mm), roughness, clearcoat strength and clearcoat roughness.
FINISHES = {'structural_twill': (2.4, .36, .42, .25), 'fine_twill': (1.6, .30, .62, .19)}

def carbon_family(material):
    """Only target named carbon composites, not generic black polymers."""
    name=material.get('name','').lower()
    if material.get('alphaMode','OPAQUE')!='OPAQUE' or not any(k in name for k in ('carbon fiber','carbon composite')): return None
    return 'fine_twill' if any(k in name for k in ('wing','cranial','face')) else 'structural_twill'

def texture_maps(family,size=512,seed=18):
    """Four repeating maps from a 2-over/2-under twill with four tows per axis."""
    if family not in FINISHES: raise ValueError('Unknown composite finish')
    if size<64 or size>2048 or size&(size-1): raise ValueError('Power of two from 64 to 2048 required')
    _,roughness,_,_=FINISHES[family]
    rng=np.random.default_rng(seed+list(FINISHES).index(family)*101)
    v,u=np.mgrid[0:size,0:size].astype(float)*(4/size)
    ix,iy=np.floor(u).astype(int),np.floor(v).astype(int)
    warp=((ix-iy)%4<2).astype(float)
    blend=gaussian_filter(warp,max(.6,size/300),mode='wrap')
    # Warp runs along V, weft along U. Tow profile and fine parallel fibers.
    profile=blend*np.sin(np.pi*(u%1))**.45+(1-blend)*np.sin(np.pi*(v%1))**.45
    fibers=.5+.5*(blend*np.cos(2*np.pi*12*u)+(1-blend)*np.cos(2*np.pi*12*v))
    micro=gaussian_filter(rng.standard_normal((size,size)),.65,mode='wrap'); micro/=max(micro.std(),1e-9)
    color=np.clip(.57+.22*profile+.12*fibers+.07*blend+.012*micro,.38,1)
    base=np.repeat(color[...,None],3,axis=2)
    rough=np.clip(roughness+.09*(1-profile)+.025*(fibers-.5)+.009*micro,.12,.7)
    mr=np.stack([np.ones_like(rough),rough,np.ones_like(rough)],axis=-1)
    h=.72*profile+.13*fibers+.075*blend+.018*micro
    dx=(np.roll(h,-1,1)-np.roll(h,1,1))*.5; dy=(np.roll(h,-1,0)-np.roll(h,1,0))*.5
    n=np.stack([-dx*1.25,-dy*1.25,np.ones_like(dx)],axis=-1); n/=np.linalg.norm(n,axis=-1,keepdims=True)
    angle=blend*np.pi/2
    aniso=np.stack([.5+.5*np.cos(angle),.5+.5*np.sin(angle),.3+.5*profile],axis=-1)
    return {k:np.round(np.clip(x,0,1)*255).astype(np.uint8) for k,x in [('basecolor',base),('metalrough',mr),('normal',n*.5+.5),('anisotropy',aniso)]}

def generate_library(directory,size=512,seed=18):
    directory=Path(directory);directory.mkdir(parents=True,exist_ok=True)
    result={}
    for family in FINISHES:
        result[family]={}
        for role,rgb in texture_maps(family,size,seed).items():
            data=png_bytes(rgb);(directory/f'{family}_{role}.png').write_bytes(data);result[family][role]=data
    return result

def enrich(source,destination,texture_dir=ROOT/'textures/carbon',size=512,seed=18,anisotropy=True,clearcoat=True):
    source,destination=Path(source),Path(destination)
    if source.resolve()==destination.resolve(): raise ValueError('Never overwrite source')
    doc,raw=read_glb(source);before=stats(doc);binary=bytearray(raw)
    if not doc.get('textures'): raise ValueError('Expected Step 17 metal-enriched GLB')
    families={i:f for i,m in enumerate(doc['materials']) if (f:=carbon_family(m)) is not None}
    if not families or any('normalTexture' in doc['materials'][i] for i in families): raise ValueError('Expected untextured carbon materials')
    def view(data,target=None):
        binary.extend(b'\0'*(-len(binary)%4));offset=len(binary);binary.extend(data)
        item=dict(buffer=0,byteOffset=offset,byteLength=len(data))
        if target: item['target']=target
        doc['bufferViews'].append(item);return len(doc['bufferViews'])-1
    def accessor(data,typ,component,target,bounds=False):
        data=np.ascontiguousarray(data);item=dict(bufferView=view(data.tobytes(),target),componentType=component,count=len(data),type=typ)
        if bounds:item.update(min=data.min(axis=0).tolist(),max=data.max(axis=0).tolist())
        doc['accessors'].append(item);return len(doc['accessors'])-1
    sampler=len(doc.setdefault('samplers',[]));doc['samplers'].append(dict(wrapS=10497,wrapT=10497,magFilter=9729,minFilter=9987))
    texture_indices={}
    for family,maps in generate_library(texture_dir,size,seed).items():
        texture_indices[family]={}
        for role,data in maps.items():
            name=f'carbon_{family}_{role}'
            doc['images'].append(dict(name=name,bufferView=view(data),mimeType='image/png'))
            doc['textures'].append(dict(name=name,source=len(doc['images'])-1,sampler=sampler));texture_indices[family][role]=len(doc['textures'])-1
    for i,family in families.items():
        m=doc['materials'][i];p=m['pbrMetallicRoughness'];tex=texture_indices[family]
        p.update(baseColorTexture=dict(index=tex['basecolor']),metallicRoughnessTexture=dict(index=tex['metalrough']),roughnessFactor=1.)
        m['normalTexture']=dict(index=tex['normal'],scale=1.)
        if anisotropy:m.setdefault('extensions',{})['KHR_materials_anisotropy']=dict(anisotropyStrength=.42,anisotropyRotation=0.,anisotropyTexture=dict(index=tex['anisotropy']))
        if clearcoat:m.setdefault('extensions',{})['KHR_materials_clearcoat']=dict(clearcoatFactor=FINISHES[family][2],clearcoatRoughnessFactor=FINISHES[family][3])
    for enabled,ext in [(anisotropy,'KHR_materials_anisotropy'),(clearcoat,'KHR_materials_clearcoat')]:
        if enabled and ext not in doc.setdefault('extensionsUsed',[]):doc['extensionsUsed'].append(ext)
    count=0
    for mesh in doc['meshes']:
        for prim in mesh['primitives']:
            family=families.get(prim.get('material'))
            if family is None:continue
            if set(prim['attributes'])!={'POSITION'} or prim.get('mode',4)!=4:raise ValueError('Unsupported source primitive')
            verts=array(doc,raw,prim['attributes']['POSITION']);ids=array(doc,raw,prim['indices'])
            p,f,uv,n,t=face_charts(verts,ids,FINISHES[family][0])
            prim['attributes']={'POSITION':accessor(p,'VEC3',5126,34962,True),'TEXCOORD_0':accessor(uv,'VEC2',5126,34962),'NORMAL':accessor(n,'VEC3',5126,34962),'TANGENT':accessor(t,'VEC4',5126,34962)}
            prim['indices']=accessor(f,'SCALAR',5125,34963);count+=1
    destination.parent.mkdir(parents=True,exist_ok=True);write_glb(destination,doc,binary)
    return dict(before=before,after=stats(doc),carbon_materials=len(families),carbon_primitives=count,texture_images=len(doc['images']),source_bytes=source.stat().st_size,output_bytes=destination.stat().st_size)

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--animated',action='store_true');p.add_argument('--mode',choices=['ultra','balanced','performance'],default='balanced');p.add_argument('--source',type=Path);p.add_argument('--output',type=Path);p.add_argument('--size',type=int,default=512);p.add_argument('--seed',type=int,default=18);p.add_argument('--no-anisotropy',action='store_true');p.add_argument('--no-clearcoat',action='store_true');a=p.parse_args()
    kind='animated' if a.animated else 'static';source=a.source or ROOT/'models'/f'dragonfly_{kind}_{a.mode}_metals.glb';output=a.output or ROOT/'models'/f'dragonfly_{kind}_{a.mode}_carbon.glb'
    print(json.dumps(enrich(source,output,size=a.size,seed=a.seed,anisotropy=not a.no_anisotropy,clearcoat=not a.no_clearcoat),indent=2))
