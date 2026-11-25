"""Regression checks on the shipped textured static and animated GLBs."""
from pathlib import Path
import sys
import io
import numpy as np
from PIL import Image
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'materials'))
from step_17_metals import read_glb,array,stats,metal_family,texture_maps


def read_accessor(doc,buf,index):
    a=doc['accessors'][index];v=doc['bufferViews'][a['bufferView']]
    width={'VEC2':2,'VEC3':3,'VEC4':4,'SCALAR':1}[a['type']]
    dtype={5126:'<f4',5125:'<u4',5123:'<u2',5121:'u1'}[a['componentType']]
    data=np.frombuffer(buf,dtype,count=a['count']*width,offset=v.get('byteOffset',0)+a.get('byteOffset',0))
    return data if width==1 else data.reshape(-1,width)


def check(kind):
    src,b=read_glb(ROOT/'models'/f'dragonfly_{kind}_balanced.glb')
    out,c=read_glb(ROOT/'models'/f'dragonfly_{kind}_balanced_metals.glb')
    assert stats(src)==stats(out)
    assert src['nodes']==out['nodes'] and src['scenes']==out['scenes']
    assert src.get('animations')==out.get('animations')
    assert c[:len(b)]==b # Original accessors including all animation values untouched.
    assert src['accessors']==out['accessors'][:len(src['accessors'])]
    assert len(out['images'])==24 and len(out['textures'])==24
    for im in out['images']:
        v=out['bufferViews'][im['bufferView']];p=v['byteOffset'];d=c[p:p+v['byteLength']]
        with Image.open(io.BytesIO(d)) as img:
            assert img.size==(512,512) and img.mode=='RGB';img.verify()
    for m,n in zip(src['materials'],out['materials']):
        if metal_family(m) is None: assert m==n
        else:
            assert m['pbrMetallicRoughness']['baseColorFactor']==n['pbrMetallicRoughness']['baseColorFactor']
            assert 'normalTexture' in n and 'KHR_materials_anisotropy' in n['extensions']
    count=0
    for old,new in zip(src['meshes'],out['meshes']):
        for p,q in zip(old['primitives'],new['primitives']):
            if metal_family(src['materials'][p['material']]) is None:
                assert p==q;continue
            def corners(doc,buf,prim):
                return read_accessor(doc,buf,prim['attributes']['POSITION'])[read_accessor(doc,buf,prim['indices'])]
            assert np.array_equal(corners(src,b,p),corners(out,c,q))
            uv,n,t=[read_accessor(out,c,q['attributes'][k]) for k in ('TEXCOORD_0','NORMAL','TANGENT')]
            assert np.isfinite(uv).all() and np.isfinite(n).all() and np.isfinite(t).all()
            assert np.allclose(np.linalg.norm(n,axis=1),1,atol=1e-6)
            assert np.allclose(np.linalg.norm(t[:,:3],axis=1),1,atol=1e-6)
            assert np.allclose(np.sum(n*t[:,:3],axis=1),0,atol=1e-6)
            assert np.all(t[:,3]==1)
            count+=1
    print(f'{kind}: {count} textured primitives, exact triangle corners, valid tangent frames, 24 PNGs; hierarchy and animations unchanged')

if __name__=='__main__':
    a=texture_maps('titanium',64);b=texture_maps('titanium',64)
    assert all(np.array_equal(a[k],b[k]) for k in a)
    assert all(not np.array_equal(a[k],texture_maps('titanium',64,18)[k]) for k in a)
    check('static');check('animated')
    import trimesh
    s=trimesh.load(ROOT/'models/dragonfly_static_balanced_metals.glb',force='scene',process=False)
    assert len(s.graph.nodes_geometry)==1709
    print('Static GLB reimport: 1709 geometry nodes; all Step 17 checks passed')
