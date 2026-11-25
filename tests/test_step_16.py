"""Verify all original triangles, rig pivots, animation samples, and caching."""
import sys
from pathlib import Path
from collections import defaultdict
import json
import tempfile
import unittest
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from optimization.step_16_geometry import read_glb, array, leaf_geometry, stats, optimize


def groups(doc, binary):
    result=defaultdict(list)
    for parent in doc['nodes']:
        if 'mesh' in parent:continue
        for i in parent.get('children',[]):
            node=doc['nodes'][i]
            if 'mesh' not in node:continue
            v,f,m=leaf_geometry(doc,binary,node)
            material=json.dumps({k:v for k,v in doc['materials'][m].items() if k not in ('name','extras')},sort_keys=True)
            result[parent['name'],material].append(v[f].reshape(-1,3,3))
    return {k:np.concatenate(v) for k,v in result.items()}


def rig(doc,binary):
    result={}
    for a in doc.get('animations',[]):
        for channel in a['channels']:
            node=doc['nodes'][channel['target']['node']];s=a['samplers'][channel['sampler']]
            result[node['name']]=(node['translation'],node['rotation'],array(doc,binary,s['input']),array(doc,binary,s['output']))
    return result


class OptimizationTests(unittest.TestCase):
    def test_exports(self):
        for kind,filename in [('static','step_15_assembled.glb'),('animated','dragonfly_animated.glb')]:
            doc,bin=read_glb(ROOT/'models'/filename); g=groups(doc,bin); r=rig(doc,bin)
            for mode in ('balanced','performance'):
                with self.subTest(kind=kind,mode=mode):
                    out,b=read_glb(ROOT/'models'/f'dragonfly_{kind}_{mode}.glb')
                    self.assertEqual(stats(doc)['triangles'],stats(out)['triangles'])
                    self.assertLess(stats(out)['mesh_nodes'],stats(doc)['mesh_nodes'])
                    og=groups(out,b);self.assertEqual(set(g),set(og))
                    for k in g:
                        np.testing.assert_allclose(g[k],og[k],rtol=0,atol=2e-8,err_msg=str(k))
                    or_=rig(out,b);self.assertEqual(set(r),set(or_))
                    for k in r:
                        for a1,a2 in zip(r[k],or_[k]):np.testing.assert_array_equal(a1,a2)
                    def topology(d):
                        return {n['name']:{d['nodes'][i]['name'] for i in n.get('children',[]) if 'mesh' not in d['nodes'][i]} for n in d['nodes'] if 'mesh' not in n}
                    self.assertEqual(topology(doc),topology(out))
                    if mode=='balanced':
                        def transparent(d):
                            return sum(d['materials'][d['meshes'][n['mesh']]['primitives'][0]['material']].get('alphaMode')=='BLEND' for n in d['nodes'] if 'mesh' in n)
                        self.assertEqual(transparent(doc),transparent(out))
    def test_cache_and_ultra(self):
        src=ROOT/'models'/'step_15_assembled.glb'
        with tempfile.TemporaryDirectory() as tmp:
            dest=Path(tmp)/'x.glb';self.assertFalse(optimize(src,dest,'ultra')['cache_hit']);self.assertEqual(src.read_bytes(),dest.read_bytes());self.assertTrue(optimize(src,dest,'ultra')['cache_hit'])
            dest.write_bytes(b'broken');self.assertFalse(optimize(src,dest,'ultra')['cache_hit']);self.assertEqual(src.read_bytes(),dest.read_bytes())
            with self.assertRaises(ValueError):optimize(src,src)

if __name__=='__main__':unittest.main(verbosity=2)
