"""Check exact GLB buffer preservation and optical extension assignments."""
from pathlib import Path
from collections import Counter
import sys
import tempfile
from copy import deepcopy
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'materials'))
from step_19_optics import read_glb, stats, optical_family, FINISHES, EXTENSIONS, enrich, upgrade_material

def check(kind):
    source=ROOT/'models'/f'dragonfly_{kind}_balanced_carbon.glb'
    dest=ROOT/'models'/f'dragonfly_{kind}_balanced_optics.glb'
    a,b=read_glb(source);c,d=read_glb(dest)
    assert b==d, 'Binary geometry, images and keyframes must be identical'
    assert stats(a)==stats(c)
    assert stats(c)['triangles']==504756
    assert stats(c)['animation_channels']==(16 if kind=='animated' else 0)
    assert set(a)==set(c)
    for k in a:
        if k not in ('materials','extensionsUsed'): assert a[k]==c[k],k
    assert set(c['extensionsUsed'])==set(a['extensionsUsed'])|set(EXTENSIONS)
    assert len(c['images'])==32 and len(c['materials'])==len(a['materials'])
    count=Counter();indices={}
    for i,(m,n) in enumerate(zip(a['materials'],c['materials'])):
        family=optical_family(m)
        if family is None: assert m==n;continue
        indices[i]=family;count[family]+=1;f=FINISHES[family]
        assert n==upgrade_material(deepcopy(m),family)
        assert n['alphaMode']=='OPAQUE' and not n['doubleSided']
        assert n['pbrMetallicRoughness']['metallicFactor']==0
        assert n['pbrMetallicRoughness']['baseColorFactor'][3]==1
        e=n['extensions'];assert e['KHR_materials_ior']['ior']==f.ior
        assert 0<e['KHR_materials_transmission']['transmissionFactor']<=1
        v=e['KHR_materials_volume'];assert 0<v['thicknessFactor']<.001 and v['attenuationDistance']>0
        assert all(0<=x<=1 for x in v['attenuationColor'])
        assert ('KHR_materials_iridescence' in e)==bool(f.coating_strength)
    assert count==Counter(compound=6,ocellus=1,power_window=1,thorax_window=1)
    instances=Counter(indices[p['material']] for n in c['nodes'] if 'mesh' in n for p in c['meshes'][n['mesh']]['primitives'] if p.get('material') in indices)
    assert instances==Counter(compound=482,ocellus=3,power_window=1,thorax_window=1)
    print(f'{kind}: 9 optical materials, 487 instances; exact original binary, geometry, 32 textures, rig and unrelated materials preserved')

if __name__=='__main__':
    check('static');check('animated')
    source=ROOT/'models/dragonfly_static_balanced_carbon.glb'
    with tempfile.TemporaryDirectory() as tmp:
        path=Path(tmp)/'uncoated.glb';enrich(source,path,False)
        doc,_=read_glb(path)
        assert EXTENSIONS[-1] not in doc['extensionsUsed']
        assert all(EXTENSIONS[-1] not in m.get('extensions',{}) for m in doc['materials'])
        try: enrich(source,source)
        except ValueError: pass
        else: raise AssertionError('Source overwrite must be rejected')
    import trimesh
    s=trimesh.load(ROOT/'models/dragonfly_static_balanced_optics.glb',force='scene',process=False)
    assert len(s.graph.nodes_geometry)==1709
    print('Uncoated variant, overwrite rejection and static GLB reimport passed.')
