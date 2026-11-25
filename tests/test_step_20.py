"""Verify geometry, material isolation, maps and rig preservation for Step 20."""
from collections import Counter
from copy import deepcopy
from pathlib import Path
import io
import sys
import tempfile
import numpy as np
from PIL import Image
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'materials'))
from step_20_wing_iridescence import (FINISHES, read_glb, array, stats, texture_maps, enrich, upgrade_material)


def read_attribute(doc, raw, index):
    a=doc['accessors'][index]
    if a['type']!='VEC2':return array(doc,raw,index)
    v=doc['bufferViews'][a['bufferView']]
    assert a['componentType']==5126 and 'byteStride' not in v
    return np.frombuffer(raw,dtype='<f4',count=2*a['count'],offset=v.get('byteOffset',0)+a.get('byteOffset',0)).reshape(-1,2)


def check(kind):
    a,b=read_glb(ROOT/'models'/f'dragonfly_{kind}_balanced_optics.glb')
    c,d=read_glb(ROOT/'models'/f'dragonfly_{kind}_balanced_iridescent.glb')
    assert d[:len(b)]==b, 'Existing binary buffer must remain unchanged'
    assert stats(a)==stats(c) and stats(c)['triangles']==504756
    assert stats(c)['animation_channels']==(16 if kind=='animated' else 0)
    assert set(a)==set(c)
    for key in a:
        if key not in ('meshes','materials','buffers','bufferViews','accessors','images','textures','samplers','extensionsUsed'):
            assert a[key]==c[key],key
    for key in ('bufferViews','accessors','images','textures','samplers'):
        assert c[key][:len(a[key])]==a[key],key
    assert set(a['extensionsUsed'])==set(c['extensionsUsed'])
    ids={i for i,m in enumerate(a['materials']) if m.get('name') in FINISHES};assert len(ids)==4
    tex={t['name'].removeprefix('wing_'):i for i,t in enumerate(c['textures']) if t.get('name','').startswith('wing_')}
    assert set(tex)=={'thickness','metalrough','normal'} and len(c['images'])==35
    expected=texture_maps();
    for role,index in tex.items():
        image=c['images'][c['textures'][index]['source']];view=c['bufferViews'][image['bufferView']];offset=view.get('byteOffset',0)
        img=np.asarray(Image.open(io.BytesIO(d[offset:offset+view['byteLength']])))
        assert np.array_equal(img,expected[role]);assert img.shape==(512,512,3)
    for i,(m,n) in enumerate(zip(a['materials'],c['materials'])):
        if i not in ids:assert m==n;continue
        assert n==upgrade_material(deepcopy(m),tex)
        assert n['alphaMode']=='OPAQUE' and n['pbrMetallicRoughness']['metallicFactor']==0
        e=n['extensions'];assert e['KHR_materials_ior']['ior']==1.46
        assert .95<=e['KHR_materials_transmission']['transmissionFactor']<=.97
        assert 'KHR_materials_volume' not in e
        coat=e['KHR_materials_iridescence'];assert 0<coat['iridescenceFactor']<1
        assert coat['iridescenceThicknessMinimum']<coat['iridescenceThicknessMaximum']
        assert coat['iridescenceThicknessTexture']['index']==tex['thickness']
    count=0
    for m,n in zip(a['meshes'],c['meshes']):
        for p,q in zip(m['primitives'],n['primitives']):
            if p.get('material') not in ids:assert p==q;continue
            assert p['material']==q['material'];assert set(q['attributes'])=={'POSITION','NORMAL','TANGENT','TEXCOORD_0'}
            old=array(a,b,p['attributes']['POSITION'])[array(a,b,p['indices'])]
            new=array(c,d,q['attributes']['POSITION'])[array(c,d,q['indices'])]
            assert np.array_equal(old,new), 'Triangle corners must remain identical'
            uv=read_attribute(c,d,q['attributes']['TEXCOORD_0']);normal=array(c,d,q['attributes']['NORMAL']);tangent=array(c,d,q['attributes']['TANGENT'])
            assert np.isfinite(uv).all() and len(uv)==len(normal)==len(tangent)
            assert np.allclose(np.linalg.norm(normal,axis=1),1,atol=2e-6)
            assert np.allclose(np.linalg.norm(tangent[:,:3],axis=1),1,atol=2e-6)
            assert np.allclose(np.sum(normal*tangent[:,:3],axis=1),0,atol=2e-6)
            assert np.all(tangent[:,3]==1);count+=1
    assert count==1144
    instances=Counter(c['materials'][p['material']]['name'] for n in c['nodes'] if 'mesh' in n for p in c['meshes'][n['mesh']]['primitives'] if p.get('material') in ids)
    assert instances=={'Neutral clear film':980,'Cool clear film':56,'Teal clear film':56,'Violet clear film':52}
    print(f'{kind}: 1144 wing panels; identical triangles, prior buffer, materials, node hierarchy and animation; 35 embedded maps')

if __name__=='__main__':
    x,y=texture_maps(128),texture_maps(128)
    assert all(np.array_equal(x[k],y[k]) for k in x)
    assert not np.array_equal(x['thickness'],texture_maps(128,seed=21)['thickness'])
    assert np.ptp(x['thickness'][...,1])>50 and np.all(x['metalrough'][...,2]==0)
    try:texture_maps(100)
    except ValueError:pass
    else:raise AssertionError('Invalid resolution accepted')
    check('static');check('animated')
    source=ROOT/'models/dragonfly_static_balanced_optics.glb'
    with tempfile.TemporaryDirectory() as tmp:
        target=Path(tmp)/'uncoated.glb';enrich(source,target,Path(tmp)/'maps',size=64,coatings=False)
        doc,_=read_glb(target)
        assert len(doc['images'])==34
        assert all('KHR_materials_iridescence' not in m.get('extensions',{}) for m in doc['materials'] if m.get('name') in FINISHES)
        assert any('KHR_materials_iridescence' in m.get('extensions',{}) for m in doc['materials'] if m.get('name') not in FINISHES)
        try:enrich(source,source)
        except ValueError:pass
        else:raise AssertionError('Source overwrite allowed')
    import trimesh
    scene=trimesh.load(ROOT/'models/dragonfly_static_balanced_iridescent.glb',force='scene',process=False)
    assert len(scene.graph.nodes_geometry)==1709
    print('Determinism, input validation, uncoated variant, overwrite rejection and GLB reimport passed.')
