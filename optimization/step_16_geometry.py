"""Step 16: lossless triangle batching in glTF parent space.

Balanced keeps alpha-blended objects separate for sorting. Performance batches
those too, trading transparency sorting quality for fewer draw submissions.
Both preserve the existing rig and animation keyframes; neither decimates.
This deliberately supports the restricted GLB layout produced by Dragonfly.
"""
from collections import defaultdict
from copy import deepcopy
from pathlib import Path
import argparse
import hashlib
import json
import shutil
import struct
import time
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
VERSION = '16.1'


def read_glb(path):
    raw = Path(path).read_bytes()
    magic, version, length = struct.unpack_from('<4sII', raw)
    if (magic, version, length) != (b'glTF', 2, len(raw)):
        raise ValueError('Expected GLB 2.0')
    n, kind = struct.unpack_from('<I4s', raw, 12)
    if kind != b'JSON': raise ValueError('Missing JSON chunk')
    doc = json.loads(raw[20:20+n]); off = 20+n
    size, kind = struct.unpack_from('<I4s', raw, off)
    if kind != b'BIN\0' or off+8+size != len(raw):
        raise ValueError('Expected a single BIN chunk')
    return doc, raw[off+8:]


def write_glb(path, doc, binary):
    binary.extend(b'\0' * (-len(binary) % 4))
    doc['buffers'] = [{'byteLength': len(binary)}]
    data = json.dumps(doc, separators=(',', ':')).encode()
    data += b' ' * (-len(data) % 4)
    raw = (struct.pack('<4sII', b'glTF', 2, 28+len(data)+len(binary))
           + struct.pack('<I4s', len(data), b'JSON') + data
           + struct.pack('<I4s', len(binary), b'BIN\0') + binary)
    Path(path).write_bytes(raw)


def array(doc, binary, index):
    a = doc['accessors'][index]; v = doc['bufferViews'][a['bufferView']]
    dtype = {5126:'<f4', 5125:'<u4', 5123:'<u2', 5121:'u1'}[a['componentType']]
    width = {'SCALAR':1, 'VEC3':3, 'VEC4':4}[a['type']]
    if 'sparse' in a or 'byteStride' in v or v.get('buffer',0) != 0:
        raise ValueError('Unsupported accessor layout')
    out = np.frombuffer(binary, dtype=dtype, count=a['count']*width,
                        offset=v.get('byteOffset',0)+a.get('byteOffset',0))
    return out if width == 1 else out.reshape(-1,width)


def leaf_geometry(doc, binary, node):
    p, = doc['meshes'][node['mesh']]['primitives']
    if set(p['attributes']) != {'POSITION'} or p.get('mode',4)!=4:
        raise ValueError('Only indexed position-only triangles are supported')
    if any(k in node for k in ('rotation','translation','scale','skin','children')):
        raise ValueError('Unsupported mesh node transform')
    matrix = np.asarray(node.get('matrix',np.eye(4).flatten(order='F'))).reshape(4,4,order='F')
    if np.linalg.det(matrix[:3,:3]) <= 0: raise ValueError('Unexpected reflected transform')
    vertices = array(doc,binary,p['attributes']['POSITION'])
    indices = array(doc,binary,p['indices'])
    return vertices @ matrix[:3,:3].T + matrix[:3,3], indices, p['material']


def stats(doc):
    nodes = [n for n in doc['nodes'] if 'mesh' in n]
    prims = [p for n in nodes for p in doc['meshes'][n['mesh']]['primitives']]
    return dict(mesh_nodes=len(nodes), unique_meshes=len(doc['meshes']),
                primitive_instances=len(prims), materials=len(doc['materials']),
                triangles=sum(doc['accessors'][p['indices']]['count']//3 for p in prims),
                animation_channels=sum(len(a['channels']) for a in doc.get('animations',[])))


def optimize(source, destination, mode='balanced', cache=True):
    """Optimize a static or already-rigged Dragonfly GLB; return measured stats."""
    source,destination=Path(source),Path(destination)
    if source.resolve()==destination.resolve(): raise ValueError('Do not overwrite original')
    if mode not in ('ultra','balanced','performance'): raise ValueError('Invalid mode')
    destination.parent.mkdir(parents=True,exist_ok=True)
    stamp=destination.with_suffix('.'+destination.suffix.lstrip('.')+'.cache.json')
    digest=hashlib.sha256(source.read_bytes()).hexdigest()
    key=dict(version=VERSION,source_sha256=digest,mode=mode)
    start=time.perf_counter()
    if cache and stamp.exists() and destination.exists():
        previous=json.loads(stamp.read_text())
        if previous.get('key')==key and previous.get('output_sha256')==hashlib.sha256(destination.read_bytes()).hexdigest():
            return dict(previous['report'],cache_hit=True,elapsed_s=time.perf_counter()-start)
    src, binary=read_glb(source)
    if mode=='ultra':
        shutil.copyfile(source,destination); out=src
    else:
        if src.get('skins') or src.get('textures') or src.get('extensionsUsed'):
            raise ValueError('This optimizer supports untextured Dragonfly GLBs only')
        out=deepcopy(src)
        out.update(nodes=[],meshes=[],materials=[],accessors=[],bufferViews=[])
        buf=bytearray(); copied={}; shared_meshes={}; material_map={}; material_keys={}
        for i,material in enumerate(src['materials']):
            signature=json.dumps({k:v for k,v in material.items() if k!='name'},sort_keys=True)
            if signature not in material_keys:
                material_keys[signature]=len(out['materials']);out['materials'].append(deepcopy(material))
            j=material_keys[signature];material_map[i]=j
            name=material.get('name')
            if name and name!=out['materials'][j].get('name'):
                aliases=out['materials'][j].setdefault('extras',{}).setdefault('aliases',[])
                if name not in aliases: aliases.append(name)
        def pack(data, typ, component, target=None, bounds=False):
            data=np.ascontiguousarray(data);buf.extend(b'\0'*(-len(buf)%4));offset=len(buf);buf.extend(data.tobytes())
            view=dict(buffer=0,byteOffset=offset,byteLength=data.nbytes)
            if target: view['target']=target
            i=len(out['bufferViews']);out['bufferViews'].append(view)
            a=dict(bufferView=i,componentType=component,count=len(data),type=typ)
            if bounds:
                a.update(min=np.atleast_1d(data.min(axis=0)).tolist(),max=np.atleast_1d(data.max(axis=0)).tolist())
            j=len(out['accessors']);out['accessors'].append(a);return j
        def copy_acc(i):
            if i not in copied:
                a=src['accessors'][i];v=src['bufferViews'][a['bufferView']]
                copied[i]=pack(array(src,binary,i),a['type'],a['componentType'],v.get('target'),'min' in a)
            return copied[i]
        def append_node(node):
            idx=len(out['nodes']);out['nodes'].append(node);return idx
        # Non-mesh nodes include the root and all animated joints. Keep them intact.
        mapping={i:append_node({k:deepcopy(v) for k,v in n.items() if k!='children'})
                 for i,n in enumerate(src['nodes']) if 'mesh' not in n}
        if any(i not in mapping for s in src['scenes'] for i in s['nodes']):
            raise ValueError('Expected non-mesh scene roots')
        for old,new in mapping.items():
            parent=src['nodes'][old];children=[];batches=defaultdict(list)
            for i in parent.get('children',[]):
                n=src['nodes'][i]
                if i in mapping: children.append(mapping[i]);continue
                p,=src['meshes'][n['mesh']]['primitives'];mat=material_map[p['material']]
                if mode=='balanced' and out['materials'][mat].get('alphaMode')=='BLEND':
                    # Share identical source geometry while preserving individual draw/sort nodes.
                    key2=(p['attributes']['POSITION'],p['indices'],mat)
                    if key2 not in shared_meshes:
                        primitive=dict(attributes={'POSITION':copy_acc(key2[0])},indices=copy_acc(key2[1]),material=mat,mode=4)
                        shared_meshes[key2]=len(out['meshes']);out['meshes'].append(dict(primitives=[primitive]))
                    node=deepcopy(n);node['mesh']=shared_meshes[key2];children.append(append_node(node))
                else: batches[mat].append(n)
            for mat, group in batches.items():
                vertices=[];indices=[];offset=0
                for n in group:
                    v,f,_=leaf_geometry(src,binary,n);vertices.append(v);indices.append(f.astype(np.uint32)+offset);offset+=len(v)
                v=np.concatenate(vertices).astype('<f4');f=np.concatenate(indices).astype('<u4')
                va=pack(v,'VEC3',5126,34962,True);fa=pack(f,'SCALAR',5125,34963)
                name=f"{parent.get('name','group')}/batch_{mat:03d}"
                mi=len(out['meshes']);out['meshes'].append(dict(name=name,primitives=[dict(attributes={'POSITION':va},indices=fa,material=mat,mode=4)]))
                children.append(append_node(dict(name=name,mesh=mi,extras={'source_parts':len(group)})))
            if children: out['nodes'][new]['children']=children
        for scene in out['scenes']:scene['nodes']=[mapping[i] for i in scene['nodes']]
        for animation in out.get('animations',[]):
            for sampler in animation['samplers']:
                for k in ('input','output'):sampler[k]=copy_acc(sampler[k])
            for channel in animation['channels']:channel['target']['node']=mapping[channel['target']['node']]
        write_glb(destination,out,buf)
    report=dict(mode=mode,before=stats(src),after=stats(out),source_bytes=source.stat().st_size,
                output_bytes=destination.stat().st_size,elapsed_s=time.perf_counter()-start,cache_hit=False)
    stamp.write_text(json.dumps(dict(key=key,output_sha256=hashlib.sha256(destination.read_bytes()).hexdigest(),report=report),indent=2))
    return report


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--mode',choices=['ultra','balanced','performance'],default='balanced')
    p.add_argument('--animated',action='store_true');p.add_argument('--source',type=Path)
    p.add_argument('--output',type=Path);p.add_argument('--no-cache',action='store_true')
    args=p.parse_args();kind='animated' if args.animated else 'static'
    source=args.source or ROOT/'models'/('dragonfly_animated.glb' if args.animated else 'step_15_assembled.glb')
    destination=args.output or ROOT/'models'/f'dragonfly_{kind}_{args.mode}.glb'
    print(json.dumps(optimize(source,destination,args.mode,not args.no_cache),indent=2))
