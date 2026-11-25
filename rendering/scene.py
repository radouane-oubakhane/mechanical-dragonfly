"""Read the project's restricted glTF layout without discarding PBR extensions."""
from pathlib import Path
from dataclasses import dataclass
import sys, io
import numpy as np
from PIL import Image
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'optimization'))
from step_16_geometry import read_glb

@dataclass
class Draw:
    start:int
    count:int
    material:int
    center:np.ndarray

class Scene:
    def __init__(self,path):
        doc,raw=path if isinstance(path,tuple) else read_glb(path); self.materials=doc['materials']; self.draws=[]
        self.images=[]
        for t in doc.get('textures',[]):
            view=doc['bufferViews'][doc['images'][t['source']]['bufferView']]; start=view.get('byteOffset',0)
            self.images.append(np.array(Image.open(io.BytesIO(raw[start:start+view['byteLength']])).convert('RGBA')))
        def accessor(i):
            a=doc['accessors'][i];v=doc['bufferViews'][a['bufferView']]
            if 'sparse' in a or 'byteStride' in v: raise ValueError('Unsupported accessor')
            width={'SCALAR':1,'VEC2':2,'VEC3':3,'VEC4':4}[a['type']];dt={5126:'<f4',5125:'<u4',5123:'<u2',5121:'u1'}[a['componentType']]
            return np.frombuffer(raw,dt,a['count']*width,v.get('byteOffset',0)+a.get('byteOffset',0)).reshape(-1,width)
        parts=[]; offset=0
        def visit(i,parent):
            nonlocal offset
            node=doc['nodes'][i]
            if 'matrix' in node: local=np.array(node['matrix']).reshape(4,4,order='F')
            else:
                local=np.eye(4);local[:3,:3]=Rotation.from_quat(node.get('rotation',[0,0,0,1])).as_matrix() @ np.diag(node.get('scale',[1,1,1]));local[:3,3]=node.get('translation',[0,0,0])
            world=parent@local
            for p in doc['meshes'][node['mesh']]['primitives'] if 'mesh' in node else []:
                if p.get('mode',4)!=4: raise ValueError('Only triangles supported')
                a=p['attributes']; ix=accessor(p['indices']).ravel(); pos=accessor(a['POSITION'])[ix]@world[:3,:3].T+world[:3,3]
                def norm(x):return x/np.maximum(np.linalg.norm(x,axis=1,keepdims=True),1e-20)
                if 'NORMAL' in a: n=norm(accessor(a['NORMAL'])[ix]@np.linalg.inv(world[:3,:3]))
                else:
                    tri=pos.reshape(-1,3,3);n=np.repeat(norm(np.cross(tri[:,1]-tri[:,0],tri[:,2]-tri[:,0])),3,axis=0)
                uv=accessor(a['TEXCOORD_0'])[ix] if 'TEXCOORD_0' in a else np.zeros((len(ix),2))
                if 'TANGENT' in a:
                    t=accessor(a['TANGENT'])[ix]; tangent=np.c_[norm(t[:,:3]@world[:3,:3].T),t[:,3]]
                else:
                    axis=np.tile([0,0,1.],(len(ix),1));axis[np.abs(n[:,2])>.9]=[0,1,0]; tangent=np.c_[norm(np.cross(axis,n)),np.ones(len(ix))]
                parts.append(np.ascontiguousarray(np.c_[pos,n,uv,tangent],dtype='f4'));self.draws.append(Draw(offset,len(ix),p['material'],pos.mean(axis=0)));offset+=len(ix)
            for child in node.get('children',[]):visit(child,world)
        for i in doc['scenes'][doc.get('scene',0)]['nodes']:visit(i,np.eye(4))
        self.vertices=np.concatenate(parts);self.bounds=np.array([self.vertices[:,:3].min(axis=0),self.vertices[:,:3].max(axis=0)])
        self.triangles=len(self.vertices)//3

    def transparent(self,draw):
        m=self.materials[draw.material];return m.get('alphaMode')=='BLEND' or m.get('extensions',{}).get('KHR_materials_transmission',{}).get('transmissionFactor',0)>0
