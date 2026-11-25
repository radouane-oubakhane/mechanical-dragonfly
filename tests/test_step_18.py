"""Validate cumulative carbon material exports without depending on a renderer."""
from pathlib import Path
import sys,io
import numpy as np
from PIL import Image
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'materials'))
from step_18_carbon import read_glb,stats,texture_maps,carbon_family,FINISHES
from test_step_17 import read_accessor

def check(kind):
    src,b=read_glb(ROOT/'models'/f'dragonfly_{kind}_balanced_metals.glb')
    out,c=read_glb(ROOT/'models'/f'dragonfly_{kind}_balanced_carbon.glb')
    assert stats(src)==stats(out) and src['nodes']==out['nodes'] and src['scenes']==out['scenes']
    assert src.get('animations')==out.get('animations') and c[:len(b)]==b
    for field in ('accessors','bufferViews','images','textures','samplers'):
        assert src[field]==out[field][:len(src[field])]
    assert len(out['images'])==32 and len(out['textures'])==32
    for im in out['images'][24:]:
        v=out['bufferViews'][im['bufferView']];offset=v['byteOffset']
        with Image.open(io.BytesIO(c[offset:offset+v['byteLength']])) as img:
            assert img.size==(512,512) and img.mode=='RGB';img.verify()
    count=0
    for m,n in zip(src['materials'],out['materials']):
        if carbon_family(m) is None: assert m==n;continue
        count+=1
        assert m['pbrMetallicRoughness']['baseColorFactor']==n['pbrMetallicRoughness']['baseColorFactor']
        assert 'normalTexture' in n and 'KHR_materials_clearcoat' in n['extensions']
        assert 'anisotropyTexture' in n['extensions']['KHR_materials_anisotropy']
    assert count==8
    count=0
    for old,new in zip(src['meshes'],out['meshes']):
        for p,q in zip(old['primitives'],new['primitives']):
            if carbon_family(src['materials'][p['material']]) is None: assert p==q;continue
            def corners(doc,buf,prim):return read_accessor(doc,buf,prim['attributes']['POSITION'])[read_accessor(doc,buf,prim['indices'])]
            assert np.array_equal(corners(src,b,p),corners(out,c,q))
            uv,n,t=[read_accessor(out,c,q['attributes'][k]) for k in ('TEXCOORD_0','NORMAL','TANGENT')]
            assert all(np.isfinite(a).all() for a in (uv,n,t))
            assert np.allclose(np.linalg.norm(n,axis=1),1,atol=1e-6)
            assert np.allclose(np.linalg.norm(t[:,:3],axis=1),1,atol=1e-6)
            assert np.allclose(np.sum(n*t[:,:3],axis=1),0,atol=1e-6)
            count+=1
    print(f'{kind}: 8 carbon materials, {count} recharted primitives, exact corners, all original materials/animation preserved except targeted carbon, 32 embedded PNGs')

if __name__=='__main__':
    for family in FINISHES:
        a=texture_maps(family);b=texture_maps(family)
        assert all(np.array_equal(a[k],b[k]) for k in a)
        assert all(v.shape==(512,512,3) and v.dtype==np.uint8 for v in a.values())
        assert a['basecolor'].std()>0 and a['normal'][...,0].std()>0
        assert a['anisotropy'][...,0].std()>10 and a['anisotropy'][...,1].std()>10
        assert not np.array_equal(a['basecolor'],texture_maps(family,seed=19)['basecolor'])
    check('static');check('animated')
    import trimesh
    s=trimesh.load(ROOT/'models/dragonfly_static_balanced_carbon.glb',force='scene',process=False)
    assert len(s.graph.nodes_geometry)==1709
    print('Static GLB reimport passed; deterministic textures and all Step 18 checks passed.')
