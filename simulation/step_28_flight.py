"""Cycle-averaged four-wing flight dynamics. SI units, Z up, +X forward.
Engineering demonstration only: assumed mass/inertia and empirical coefficients,
no wing wake, ground contact, motor limits, power or collision simulation.
"""
from dataclasses import dataclass, asdict, replace
from pathlib import Path
import argparse, csv, json, sys
import numpy as np
from scipy.spatial.transform import Rotation, Slerp
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'simulation'),str(ROOT/'components'),str(ROOT/'optimization')]
from step_23_wing_kinematics import WINGS, WingControl, wing_rotation, read_glb, write_glb, array, _append_accessor
from step_24_transmission import geometry, drive_pose
from step_13_wing_skeleton import make_wing_plan, wing_placements
from step_16_geometry import stats
SOURCE=ROOT/'models/dragonfly_legs_animated.glb'
OUTPUT=ROOT/'models/dragonfly_flight.glb'

@dataclass(frozen=True)
class FlightConfig:
    mass: float=.035
    inertia: tuple=(.00006,.00018,.00020)
    com: tuple=(-.012,0.,0.)
    density: float=1.225
    lift_coefficient: float=1.2
    drag_coefficient_area: float=.008
    amplitude_deg: float=30.
    max_frequency: float=40.
    gravity: float=9.81
    def __post_init__(self):
        a=np.array([self.mass,self.density,self.lift_coefficient,self.drag_coefficient_area,self.amplitude_deg,self.max_frequency,self.gravity]); j=np.asarray(self.inertia); c=np.asarray(self.com)
        if not np.isfinite(a).all() or np.any(a<=0) or self.amplitude_deg>35 or j.shape!=(3,) or not np.isfinite(j).all() or np.any(j<=0) or c.shape!=(3,) or not np.isfinite(c).all():raise ValueError('Invalid flight parameters')

@dataclass(frozen=True)
class AeroGeometry:
    area: np.ndarray
    moment2: np.ndarray
    centers: np.ndarray
    gain: np.ndarray
    allocation: np.ndarray

def aero_geometry(config=FlightConfig()):
    """Integrate projected wing cells using degree-2 triangle quadrature.
    The lift-weighted center uses span squared; its higher moments are approximate.
    """
    placements=wing_placements(); areas=[];moments=[];centers=[]
    for w in WINGS:
        pair,side=w.split('_'); plan=make_wing_plan(side,pair); p=plan.points_mm*.001
        M=placements['wing_skeleton_'+w]; area=moment=0.;weighted=np.zeros(3)
        for cell in plan.cells:
            for j in range(1,len(cell)-1):
                tri=p[[cell[0],cell[j],cell[j+1]]]; a=abs(np.cross(tri[1]-tri[0],tri[2]-tri[0])[2])/2
                q=np.array([[2/3,1/6,1/6],[1/6,2/3,1/6],[1/6,1/6,2/3]])@tri
                world=q@M[:3,:3].T+M[:3,3]; root_y=.0228 if side=='left' else -.0228
                r2=(world[:,1]-root_y)**2; area+=a;moment+=a*r2.mean();weighted+=a*np.mean(world*r2[:,None],axis=0)
        areas.append(area);moments.append(moment);centers.append(weighted/moment-np.asarray(config.com))
    centers=np.array(centers);moments=np.array(moments)
    # Mean squared sinusoidal angular velocity = (2*pi*f*A)^2/2.
    gain=.25*config.density*config.lift_coefficient*moments*(2*np.pi*np.deg2rad(config.amplitude_deg))**2
    alloc=np.vstack([np.ones(4),centers[:,1],-centers[:,0]])
    return AeroGeometry(np.array(areas),moments,centers,gain,alloc)

def allocate(geometry, total, torque_xy, max_frequency):
    """Minimum-norm thrust allocation, clipped to nonnegative bounded wing forces."""
    request=np.r_[total,torque_xy]; f=np.linalg.pinv(geometry.allocation)@request
    upper=geometry.gain*max_frequency**2
    return np.clip(f,0,upper),bool(np.any(f<0) or np.any(f>upper))

def forces(config,geo,rotation,velocity,frequency):
    """Four averaged body-Z wing forces, body torque, and world drag/gravity."""
    f=geo.gain*np.asarray(frequency)**2; body=np.array([0.,0.,f.sum()]); torque=np.cross(geo.centers,np.column_stack([np.zeros((4,2)),f])).sum(axis=0)
    drag=-.5*config.density*config.drag_coefficient_area*np.linalg.norm(velocity)*velocity
    force=rotation.apply(body)+drag-np.array([0.,0.,config.mass*config.gravity])
    return force,torque,f

def simulate(mode='hover',duration=4.,dt=1/480,config=FlightConfig(),geo=None):
    """Semi-implicit 6-DOF integrator with bounded lift and idealized yaw stabilization."""
    if mode not in ('hover','forward') or not np.isfinite([duration,dt]).all() or not (0<dt<=.01 and 0<duration<=30) or not np.isclose(duration/dt,round(duration/dt)):raise ValueError('Invalid mode or timeline')
    geo=aero_geometry(config) if geo is None else geo
    n=round(duration/dt)+1;t=np.linspace(0,duration,n); p=np.array([0.,0.,.8]);v=np.zeros(3);r=Rotation.identity();omega=np.zeros(3);I=np.array(config.inertia);ph=np.zeros(4);out={k:[] for k in ('position','velocity','rotation','omega','frequency','phase_cycles','lift','torque','yaw_assist','saturated','target_velocity')}
    for i,ti in enumerate(t):
        vx=.55*(1-np.exp(-max(ti-.6,0)/.7)) if mode=='forward' else 0.
        ax=2.5*(vx-v[0]) if mode=='forward' else -2.0*p[0]-2.8*v[0]
        accel=np.array([ax,-2*p[1]-2.8*v[1],5*(1.-p[2])-4*v[2]])
        desired=accel+np.array([0.,0.,config.gravity]); desired[:2]=np.clip(desired[:2],-2.,2.);desired[2]=max(desired[2],2.)
        z=desired/np.linalg.norm(desired); y=np.cross(z,[1.,0.,0.]);y/=np.linalg.norm(y); x=np.cross(y,z); goal=Rotation.from_matrix(np.column_stack([x,y,z]))
        err=(r.inv()*goal).as_rotvec(); requested=I*(65*err-14*omega)+np.cross(omega,I*omega)
        total=config.mass*np.linalg.norm(desired)/max(np.dot(r.apply([0,0,1]),z),.5)
        lift,sat=allocate(geo,total,requested[:2],config.max_frequency);freq=np.sqrt(lift/geo.gain)
        force,torque,lift=forces(config,geo,r,v,freq)
        # No wing-resolved yaw force in this model; explicit idealized auxiliary torque.
        assist=float(np.clip(requested[2],-.0003,.0003));torque[2]+=assist
        for k,val in zip(out,(p.copy(),v.copy(),r.as_quat(),omega.copy(),freq,ph.copy(),lift,torque.copy(),assist,sat,vx)):out[k].append(val)
        if i==n-1:break
        v+=dt*force/config.mass;p+=dt*v
        omega+=dt*(torque-np.cross(omega,I*omega))/I;r=r*Rotation.from_rotvec(omega*dt)
        ph+=dt*freq
    return {'time':t,**{k:np.asarray(v) for k,v in out.items()}}

def telemetry(path,data):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    names=['time_s','x_m','y_m','z_m','vx_m_s','vy_m_s','vz_m_s','qx','qy','qz','qw','wx_rad_s','wy_rad_s','wz_rad_s']+[w+'_hz' for w in WINGS]+[w+'_lift_N' for w in WINGS]+['tx_Nm','ty_Nm','tz_Nm','yaw_assist_Nm','saturated','target_vx_m_s']
    vals=np.column_stack([data[k] for k in ('time','position','velocity','rotation','omega','frequency','lift','torque','yaw_assist','saturated','target_velocity')]);np.savetxt(path,vals,delimiter=',',header=','.join(names),comments='')

def export(source,destination,runs,config=FlightConfig()):
    """Keep prior clips, add body translation/rotation and phase-driven wing/drive tracks.
    Head, abdomen and legs reuse periodic authored clips, not force-driven controls.
    """
    if Path(source).resolve()==Path(destination).resolve():raise ValueError('Do not overwrite input')
    d,raw=read_glb(source);b=bytearray(raw);nodes=d['nodes'];roots=d['scenes'][d.get('scene',0)]['nodes'];old_roots=list(roots);root=len(nodes);nodes.append({'name':'rig_flight_body','children':old_roots});roots[:]=[root]
    # Rotate around COM, not the previous chassis origin.
    offset=len(nodes);nodes.append({'name':'flight_com_offset','translation':(-np.asarray(config.com)).tolist(),'children':old_roots});nodes[root]['children']=[offset];nodes[root]['translation']=list(config.com)
    refs={a['name']:a for a in d['animations']};ids={n.get('name'):i for i,n in enumerate(nodes)};gs={w:geometry(w) for w in WINGS}
    for mode,data in runs.items():
        t=data['time'];a={'name':'flight_'+mode,'channels':[],'samplers':[],'extras':{'non_looping':True,'cycle_averaged':True,'config':asdict(config),'yaw_assist':'idealized auxiliary torque'}};ti=_append_accessor(d,b,t,'SCALAR')
        def add(node,path,values):
            v=np.array(values,copy=True)
            if path=='rotation':
                for j in range(1,len(v)):
                    if np.dot(v[j-1],v[j])<0:v[j]*=-1
            s={'input':ti,'output':_append_accessor(d,b,v,'VEC4' if path=='rotation' else 'VEC3'),'interpolation':'LINEAR'};a['channels'].append({'sampler':len(a['samplers']),'target':{'node':node,'path':path}});a['samplers'].append(s)
        add(root,'translation',data['position']);add(root,'rotation',data['rotation'])
        for i,w in enumerate(WINGS):
            control=WingControl(frequency=1.,flap=config.amplitude_deg,phase=-90. if w.startswith('hind') else 0.,pitch=10.)
            cycles=data['phase_cycles'][:,i];add(ids['rig_wing_'+w],'rotation',wing_rotation(w,control,cycles))
            for role,tracks in drive_pose(w,control,cycles,gs[w]).items():
                for path,val in tracks.items():add(ids[f'rig_drive_{w}_{role}'],path,val)
        ref=refs[mode]
        for c in ref['channels']:
            node=c['target']['node'];path=c['target']['path'];name=nodes[node].get('name','')
            if name.startswith(('rig_wing_','rig_drive_')):continue
            s=ref['samplers'][c['sampler']];ot=array(d,raw,s['input']);ov=array(d,raw,s['output']);q=np.mod(t-ot[0],ot[-1]-ot[0])+ot[0]
            vals=Slerp(ot,Rotation.from_quat(ov))(q).as_quat() if path=='rotation' else np.column_stack([np.interp(q,ot,ov[:,i]) for i in range(3)])
            add(node,path,vals)
        d['animations'].append(a)
    destination=Path(destination);destination.parent.mkdir(parents=True,exist_ok=True);write_glb(destination,d,b);return stats(d)

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--mode',choices=['all','hover','forward'],default='all');p.add_argument('--duration',type=float,default=4.);p.add_argument('--mass',type=float,default=.035);p.add_argument('--source',type=Path,default=SOURCE);p.add_argument('--output',type=Path,default=OUTPUT);p.add_argument('--telemetry-dir',type=Path,default=ROOT/'telemetry');a=p.parse_args();cfg=FlightConfig(mass=a.mass);g=aero_geometry(cfg);modes=['hover','forward'] if a.mode=='all' else [a.mode];runs={m:simulate(m,a.duration,config=cfg,geo=g) for m in modes}
    for m,data in runs.items():telemetry(a.telemetry_dir/f'flight_{m}.csv',data)
    report={'config':asdict(cfg),'wing_area_cm2':(g.area*1e4).tolist(),'hover_frequency_hz':float(np.sqrt(cfg.mass*cfg.gravity/g.gain.sum())), 'runs':{m:{'final_position_m':v['position'][-1].tolist(),'final_velocity_m_s':v['velocity'][-1].tolist(),'frequency_range_hz':[float(v['frequency'].min()),float(v['frequency'].max())],'saturated_fraction':float(v['saturated'].mean())} for m,v in runs.items()},'export':export(a.source,a.output,runs,cfg)}
    a.telemetry_dir.mkdir(parents=True,exist_ok=True);(a.telemetry_dir/'flight_report.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
if __name__=='__main__':main()
