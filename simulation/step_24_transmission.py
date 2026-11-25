"""Phase-locked visual transmission rig. Not a rigid linkage or collision solver."""
from pathlib import Path
import argparse, json, sys, tempfile
import numpy as np
from scipy.spatial.transform import Rotation, Slerp
from step_23_wing_kinematics import (ROOT,WINGS,read_glb,write_glb,array,angles,WingControl,_append_accessor,export_profiles)
sys.path.insert(0,str(ROOT/'components'));sys.path.insert(0,str(ROOT/'materials'))
from step_01_chassis import create_chassis
from step_04_transmission import create_transmission
from step_16_geometry import optimize,stats
import step_17_metals,step_18_carbon,step_19_optics,step_20_wing_iridescence
SOURCE=ROOT/'models/dragonfly_animated.glb'
PROFILES=ROOT/'models/dragonfly_wing_kinematics.glb'
OUTPUT=ROOT/'models/dragonfly_transmission_animated.glb'

def transform(p,r=np.eye(3)):
    m=np.eye(4);m[:3,:3]=r;m[:3,3]=p;return m

def basis(a,b):
    x=b-a;x=x/np.linalg.norm(x);y=np.cross([0.,0.,1.],x);y/=np.linalg.norm(y);return np.column_stack([x,y,np.cross(x,y)])

def geometry(wing):
    """Reference pivots from the original transmission and actuator definitions."""
    t=create_chassis().anchors['transmission'];tr=create_transmission();m=t@tr.anchors['actuator_'+wing]
    def world(p):return m[:3,:3]@(np.asarray(p)*.001)+m[:3,3]
    return dict(m=m,driver=(t@tr.anchors['driver_shaft_'+wing])[:3,3],output=(t@tr.anchors['output_shaft_'+wing])[:3,3],rocker=world((8,0,1.55)),lower=world((7.3,0,0)),upper=world((9.3,0,3.2)),crank=world((-3.8,0,3.35)),rod_start=world((5.65,0,0)))

def rest_groups(wing,g):
    tp='transmission/'+wing+'_';ap='actuator_'+wing+'/'
    def parts(role):return [tp+role+'_'+s for s in ('inner_race','shaft','gear_hub','gear','retainer')]
    return {
      'driver':(transform(g['driver']),parts('driver')),
      'output':(transform(g['output']),parts('output')+[tp+'output_coupling']+[ap+s for s in ('gear_crank_hub','gear_crank_arm','gear_crank_pin')]),
      'rocker':(transform(g['rocker']),[ap+s for s in ('rocker_lower_arm','rocker_upper_arm','lower_rocker_pin','upper_rocker_pin')]),
      'clevis':(transform(g['lower']),[ap+s for s in ('clevis_-1','clevis_1','clevis_pin')]),
      'piston':(transform(g['rod_start'],basis(g['rod_start'],g['lower'])),[ap+'piston_rod']),
      'link':(transform(g['crank'],basis(g['crank'],g['upper'])),[ap+'gear_to_rocker_link'])}

def rig_source(source,destination,geometries):
    doc,raw=read_glb(source);nodes=doc['nodes'];root=doc['scenes'][doc.get('scene',0)]['nodes'][0];children=nodes[root]['children'];names={nodes[i]['name']:i for i in children}
    for wing,g in geometries.items():
        for role,(rest,parts) in rest_groups(wing,g).items():
            selected=[names[p] for p in parts]
            for i in selected:
                old=np.asarray(nodes[i].get('matrix',np.eye(4).flatten(order='F'))).reshape(4,4,order='F');nodes[i]['matrix']=(np.linalg.inv(rest)@old).flatten(order='F').tolist();children.remove(i)
            children.append(len(nodes));nodes.append(dict(name=f'rig_drive_{wing}_{role}',translation=rest[:3,3].tolist(),rotation=Rotation.from_matrix(rest[:3,:3]).as_quat().tolist(),children=selected))
    write_glb(destination,doc,bytearray(raw))

def link_pose(a,b,a0,b0):
    """Endpoint-exact, telescoping approximation (length is allowed to vary)."""
    r0=basis(a0,b0);x0=r0[:,0];v=b-a;length=np.linalg.norm(v,axis=-1);x=v/length[:,None]
    cross=np.cross(x0,x);sine=np.linalg.norm(cross,axis=1);cosine=x@x0
    if np.any((sine<1e-12)&(cosine<0)):raise ValueError('Antiparallel linkage')
    r=Rotation.from_rotvec(cross/np.maximum(sine[:,None],1e-15)*np.arctan2(sine,cosine)[:,None])*Rotation.from_matrix(r0)
    s=np.ones((len(a),3));s[:,0]=length/np.linalg.norm(b0-a0)
    return {'translation':a.copy(),'rotation':r.as_quat(),'scale':s}

def drive_pose(wing,control,t,g):
    t=np.atleast_1d(np.asarray(t,float));f,_=angles(control,t);phase=2*np.pi*control.frequency*t+np.deg2rad(control.phase)
    rz=Rotation.from_rotvec(np.outer(phase,[0,0,1.]));rock=Rotation.from_rotvec(np.outer(np.deg2rad(-f),g['m'][:3,1]))
    lower=g['rocker']+rock.apply(g['lower']-g['rocker']);upper=g['rocker']+rock.apply(g['upper']-g['rocker']);crank=g['output']+rz.apply(g['crank']-g['output']);fixed=np.broadcast_to(g['rod_start'],lower.shape)
    return {'driver':{'rotation':rz.inv().as_quat()},'output':{'rotation':rz.as_quat()},'rocker':{'rotation':rock.as_quat()},'clevis':{'translation':lower},'piston':link_pose(fixed,lower,g['rod_start'],g['lower']),'link':link_pose(crank,upper,g['crank'],g['upper'])}

def export(source=SOURCE,destination=OUTPUT,profiles=PROFILES):
    if Path(destination).resolve() in (Path(source).resolve(),Path(profiles).resolve()):raise ValueError('Do not overwrite input')
    gs={w:geometry(w) for w in WINGS};ref,rb=read_glb(profiles);controls={a['name']:{w:WingControl(**c) for w,c in a['extras']['wing_controls'].items()} for a in ref['animations']}
    ts=[array(ref,rb,a['samplers'][0]['input']) for a in ref['animations']];duration=float(ts[0][-1]);fps=round((len(ts[0])-1)/duration)
    if any(not np.array_equal(t,ts[0]) for t in ts):raise ValueError('Expected shared timeline')
    with tempfile.TemporaryDirectory() as temp:
        d=Path(temp);p=[d/f'{i}.glb' for i in range(7)]
        rig_source(source,p[0],gs);optimize(p[0],p[1],cache=False);print('Rigged and batched',flush=True)
        step_17_metals.enrich(p[1],p[2],d/'metals');step_18_carbon.enrich(p[2],p[3],d/'carbon');step_19_optics.enrich(p[3],p[4]);step_20_wing_iridescence.enrich(p[4],p[5],d/'wings');print('Materials reapplied',flush=True)
        export_profiles(p[5],p[6],controls,duration,fps);doc,raw=read_glb(p[6])
    binary=bytearray(raw);names={n.get('name'):i for i,n in enumerate(doc['nodes'])}
    for a in doc['animations']:
        timeidx=a['samplers'][0]['input'];t=array(doc,raw,timeidx)
        for w,g in gs.items():
            for role,tracks in drive_pose(w,controls[a['name']][w],t,g).items():
                for path,v in tracks.items():
                    v=v.copy()
                    if path=='rotation':
                        for j in range(1,len(v)):
                            if np.dot(v[j-1],v[j])<0:v[j]*=-1
                        v[-1]=v[0] if np.dot(v[-1],v[0])>=0 else -v[0]
                    else:v[-1]=v[0]
                    a['channels'].append({'sampler':len(a['samplers']),'target':{'node':names[f'rig_drive_{w}_{role}'],'path':path}})
                    a['samplers'].append({'input':timeidx,'output':_append_accessor(doc,binary,v,'VEC4' if path=='rotation' else 'VEC3'),'interpolation':'LINEAR'})
        a['extras']['drive_model']='Phase-locked visual rig; rod and link length vary; not a physical constraint solver.'
    destination=Path(destination);destination.parent.mkdir(parents=True,exist_ok=True);write_glb(destination,doc,binary)
    return dict(stats=stats(doc),clips={a['name']:len(a['channels']) for a in doc['animations']},drive_groups=24,output=str(destination))

def pose_glb(source,destination,clip='hover',time=0.):
    """Bake all TRS animation channels for the existing studio renderer."""
    if Path(source).resolve()==Path(destination).resolve() or not np.isfinite(time):raise ValueError('Invalid destination or time')
    d,raw=read_glb(source);a=next(a for a in d['animations'] if a['name']==clip)
    for c in a['channels']:
        s=a['samplers'][c['sampler']];t=array(d,raw,s['input']);v=array(d,raw,s['output']);path=c['target']['path'];x=float(np.clip(time,t[0],t[-1]))
        value=Slerp(t,Rotation.from_quat(v))(x).as_quat() if path=='rotation' else np.array([np.interp(x,t,v[:,j]) for j in range(3)])
        d['nodes'][c['target']['node']][path]=value.tolist()
    d.pop('animations');write_glb(destination,d,bytearray(raw))

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--source',type=Path,default=SOURCE);p.add_argument('--profiles',type=Path,default=PROFILES);p.add_argument('--output',type=Path,default=OUTPUT);a=p.parse_args();print(json.dumps(export(a.source,a.output,a.profiles),indent=2))
