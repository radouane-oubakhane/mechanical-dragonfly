"""Final release geometry, regression and animation structure checks."""
from pathlib import Path
import sys
import numpy as np
import trimesh
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'components')]
from step_15_thorax_armor import create_thorax_armor
from simulation.animation import read_glb, rig_specs


def load(name): return trimesh.load(ROOT/'models'/name,force='scene',process=False)


def world(doc):
    out={}
    def walk(i,parent):
        n=doc['nodes'][i]
        if 'matrix' in n:m=np.asarray(n['matrix']).reshape(4,4,order='F')
        else:
            m=np.eye(4);m[:3,3]=n.get('translation',[0,0,0]);assert n.get('rotation',[0,0,0,1])==[0,0,0,1]
        m=parent@m
        if 'mesh' in n:out[n['name']]=m
        for c in n.get('children',[]):walk(c,m)
    for i in doc['scenes'][doc.get('scene',0)]['nodes']:walk(i,np.eye(4))
    return out


def run():
    m=create_thorax_armor();assert len(m.scene.geometry)==98 and len(m.anchors)==4
    assert all(g.is_watertight and g.is_winding_consistent and g.volume>0 for g in m.scene.geometry.values())
    for s in ('left','right'):
        for i in range(3):
            g=m.scene.geometry[f'side_{s}_{i}_vented'];assert g.euler_number==-4 # 3 through-holes
    assert m.scene.geometry['inspection_window'].visual.material.alphaMode=='BLEND'
    np.testing.assert_allclose(create_thorax_armor(2).scene.extents,2*m.scene.extents)
    for x in (0,-1,float('nan')):
        try:create_thorax_armor(x)
        except ValueError:pass
        else:raise AssertionError('invalid scale accepted')
    print('PASS: armor geometry, 18 side through-slots, glazing, scale validation',flush=True)
    prev=load('step_14_assembled.glb');final=load('step_15_assembled.glb');assert len(prev.geometry)==5671 and len(final.geometry)==5769
    for n in prev.graph.nodes_geometry:
        a,g=prev.graph[n];b,h=final.graph[n];np.testing.assert_allclose(a,b,atol=1e-10);np.testing.assert_array_equal(prev.geometry[g].faces,final.geometry[h].faces);np.testing.assert_allclose(prev.geometry[g].vertices,final.geometry[h].vertices,atol=1e-10)
    assert all(g.is_watertight and g.is_winding_consistent and g.volume>0 and np.isfinite(g.vertices).all() for g in final.geometry.values())
    np.testing.assert_allclose(final.extents*1000,[255.71473812,400.18210709,60.44],atol=1e-4)
    print('PASS: all 5,671 previous exported meshes unchanged; 5,769 valid final meshes; nominal dimensions',flush=True)
    # Targeted roof clearance, not a full intersection test.
    roof=[n for n in final.graph.nodes_geometry if n.startswith(('thorax_armor/roof_longitudinal','thorax_armor/roof_vent_bar','thorax_armor/inspection_window'))]
    for n in roof:
        t,g=final.graph[n];assert trimesh.transform_points(final.geometry[g].vertices,t)[:,2].min()>.0186
    print('PASS: roof and window above central drive envelope',flush=True)
    d,b=read_glb(ROOT/'models'/'step_15_assembled.glb');a,ab=read_glb(ROOT/'models'/'dragonfly_animated.glb')
    w0,w1=world(d),world(a);assert w0.keys()==w1.keys()
    for n in w0:np.testing.assert_allclose(w0[n],w1[n],atol=1e-10)
    clip=a['animations'][0];assert len(clip['channels'])==16 and len(clip['samplers'])==16
    def array(idx,width):
        ac=a['accessors'][idx];v=a['bufferViews'][ac['bufferView']];return np.frombuffer(ab,dtype='<f4',count=ac['count']*width,offset=v.get('byteOffset',0)+ac.get('byteOffset',0)).reshape(-1,width)
    for c in clip['channels']:
        s=clip['samplers'][c['sampler']];t=array(s['input'],1).ravel();q=array(s['output'],4)
        assert np.all(np.diff(t)>0) and np.isclose(t[-1],2)
        np.testing.assert_allclose(np.linalg.norm(q,axis=1),1,atol=1e-6);np.testing.assert_allclose(q[0],[0,0,0,1],atol=1e-6);np.testing.assert_allclose(q[-1],q[0],atol=1e-6);assert np.max(np.abs(q[:,:3]))>0
    names={n['name']:n for n in a['nodes']}
    for i in range(1,10):assert a['nodes'].index(names[f'rig_abdomen_{i+1:02d}']) in names[f'rig_abdomen_{i:02d}']['children']
    print('PASS: 16 animation channels, normalized looping quaternions, exact neutral-pose transforms and nested abdomen hierarchy',flush=True)

if __name__=='__main__':run()
