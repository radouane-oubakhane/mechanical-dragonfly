"""Reduced-order rigid-body crash solver, SI units, Z-up. No CFD or deformation."""
from dataclasses import dataclass
import numpy as np
from scipy.spatial.transform import Rotation, Slerp
from scipy.spatial import ConvexHull

@dataclass(frozen=True)
class Physics:
    mass: float = .035
    inertia: tuple = (.00006,.00018,.00020)
    gravity: float = 9.81
    friction: float = .55
    restitution: float = .12
    drag: float = .006
    step: float = 1/480

COM = np.array([-.012,0.,0.])

def smooth(x):
    x=np.clip(x,0,1); return x**3*(10+x*(-15+6*x))

def hull_points(vertices):
    points=np.asarray(vertices,float)-COM
    return points[ConvexHull(points).vertices]

def crash(points, position, velocity, rotation, omega, duration, cfg=Physics()):
    """Semi-implicit integration; sequential unilateral contact impulses with
    Coulomb friction and low restitution. Convex neutral-pose proxy, not FEM.
    Angular velocity is world-frame. Returns contact telemetry and pose samples.
    """
    dt=cfg.step; m=cfg.mass; p=np.array(position,float); v=np.array(velocity,float); w=np.array(omega,float); r=rotation
    n=int(np.ceil(duration/dt))+1; ts=np.arange(n)*dt
    ps=[];vs=[];qs=[];ws=[];cs=[];js=[];gaps=[]; sleeps=[]; sleep_count=0; sleeping=False
    for i in range(n):
        rel=r.apply(points); gap=float((rel[:,2]+p[2]).min()); contact=gap<=.00015
        ps.append(p.copy());vs.append(v.copy());qs.append(r.as_quat());ws.append(w.copy());cs.append(contact);gaps.append(gap);sleeps.append(sleeping)
        impulse_total=0.
        if not sleeping and i<n-1:
            v+=dt*(np.array([0.,0.,-cfg.gravity])-cfg.drag/m*np.linalg.norm(v)*v)
            # Mild aerodynamic angular damping, with torque-free gyroscopic motion.
            mat=r.as_matrix(); inertia=mat@np.diag(cfg.inertia)@mat.T; inv=np.linalg.inv(inertia)
            w+=dt*(inv@(-np.cross(w,inertia@w))-.8*w)
            p+=dt*v; r=Rotation.from_rotvec(w*dt)*r
            rel=r.apply(points); z=rel[:,2]+p[2]; ids=np.flatnonzero(z<.00012)
            if len(ids):
                # Select deepest plus spatially separated supports to avoid
                # overcounting densely tessellated corners of the same surface.
                ids=ids[np.argsort(z[ids])]; chosen=[]
                for k in ids:
                    if not chosen or min(np.linalg.norm(rel[k]-rel[j]) for j in chosen)>.007: chosen.append(k)
                    if len(chosen)>=8:break
                mat=r.as_matrix(); inv=mat@np.diag(1/np.asarray(cfg.inertia))@mat.T
                normal=np.array([0.,0.,1.]); accum=np.zeros(len(chosen)); tangent=np.zeros((len(chosen),3)); bounce=[]
                for k in chosen:
                    vn=(v+np.cross(w,rel[k]))[2];bounce.append(-cfg.restitution*vn if vn<-.35 else 0.)
                def apply(j,a):
                    nonlocal v,w
                    v+=j/m;w+=inv@np.cross(a,j)
                def effective(a,d):return 1/m+np.dot(d,np.cross(inv@np.cross(a,d),a))
                for _ in range(8):
                    for j,k in enumerate(chosen):
                        a=rel[k]; speed=v+np.cross(w,a); old=accum[j]
                        accum[j]=max(0.,old+(bounce[j]-speed[2])/effective(a,normal)); apply((accum[j]-old)*normal,a)
                        speed=v+np.cross(w,a); lateral=speed.copy();lateral[2]=0.; norm=np.linalg.norm(lateral)
                        if norm>1e-10:
                            d=lateral/norm; trial=tangent[j]-d*norm/effective(a,d); limit=cfg.friction*accum[j]; length=np.linalg.norm(trial)
                            if length>limit:trial*=limit/length
                            apply(trial-tangent[j],a);tangent[j]=trial
                impulse_total=float(accum.sum())
                # Position stabilization only for actual penetration, never airborne.
                p[2]+=max(0.,-float(z.min()))
                # Lumped rolling resistance approximates small compliant contacts.
                w*=np.exp(-3.5*dt);v[:2]*=np.exp(-1.8*dt)
                if np.linalg.norm(v)<.025 and np.linalg.norm(w)<.22:sleep_count+=1
                else:sleep_count=0
                if sleep_count>int(.2/dt):v[:]=0;w[:]=0;sleeping=True
            else:sleep_count=0
        js.append(impulse_total)
    return dict(time=ts,position=np.array(ps),velocity=np.array(vs),rotation=np.array(qs),omega=np.array(ws),contact=np.array(cs),impulse=np.array(js),gap=np.array(gaps),sleep=np.array(sleeps))

class Motion:
    def __init__(self,s,points,cfg=Physics()):
        self.s=s; self.e={x['action']:float(x['time']) for x in s['timeline']}; self.points=points;self.cfg=cfg
        self.ground=-float(points[:,2].min());self.fall=self.e['fall'];sign=1 if s['failed_wing'].endswith('left') else -1
        p,r=self.flight(self.fall);p0,_=self.flight(self.fall-.0001);v=(p-p0)/.0001
        self.run=crash(points,p,v,r,(r*self.flight(self.fall-.0001)[1].inv()).as_rotvec()/.0001,s['duration']-self.fall,cfg)
        self.interp=Slerp(self.run['time'],Rotation.from_quat(self.run['rotation']))
        hits=np.flatnonzero(self.run['contact']);self.impact=self.fall+self.run['time'][hits[0]] if len(hits) else None
        # Integrated continuous phase: no resets at scene boundaries.
        self.ts=np.arange(0,s['duration']+cfg.step,cfg.step); self.clocks={}
        for name in ('fore_left','fore_right','hind_left','hind_right'):
            f=np.array([self.speed(t,name) for t in self.ts]); self.clocks[name]=np.r_[0,np.cumsum((f[:-1]+f[1:])*.5*cfg.step)]
    def progress(self,t,a,b):return float(smooth((t-self.e[a])/(self.e[b]-self.e[a])))
    def flight(self,t):
        q=lambda a,b:self.progress(t,a,b); h=self.s['flight_height_m']; travel=self.s['travel_m'];sign=1 if self.s['failed_wing'].endswith('left') else -1
        # Authored continuous C2 takeoff and cruise; physics starts at lift loss.
        # Last second introduces gradual sink and nonzero forward velocity.
        u=np.clip((t-self.e['flight'])/(self.fall-self.e['flight']),0,1)
        x=travel*(.15*q('takeoff','flight')+.72*u**3);
        height=self.ground+h*q('takeoff','flight')-.07*q('unstable','fall')
        u2=np.clip((t-self.e['unstable'])/(self.fall-self.e['unstable']),0,1)
        roll=sign*(6*q('failure','unstable')+88*u2**3);pitch=5*q('takeoff','flight')-5*q('failure','fall');yaw=sign*8*q('failure','fall')
        return np.array([x,0.,height]),Rotation.from_euler('xyz',[roll,pitch,yaw],degrees=True)
    def speed(self,t,w):
        q=lambda a,b:self.progress(t,a,b)
        speed=q('scan','takeoff')*(1-q('unstable','fall'))
        if w==self.s['failed_wing']:speed*=1-q('failure','unstable')
        return speed
    def sample(self,t):
        if t<self.fall:p,r=self.flight(t);contact=t<=self.e['takeoff']
        else:
            u=min(t-self.fall,self.run['time'][-1]);p=np.array([np.interp(u,self.run['time'],self.run['position'][:,j]) for j in range(3)]);r=self.interp(u);contact=bool(self.run['contact'][min(round(u/self.cfg.step),len(self.run['time'])-1)])
        return p,r,contact
    def amplitude(self,t,w):
        # Smoothly return to the neutral wing pose before rigid collision.
        p,_=self.flight(min(t,self.fall))
        # Bound downward tip excursion while legs are still on the floor.
        cap=.25+.75*smooth((p[2]-self.ground)/.12)
        return min(self.speed(t,w),float(cap))
    def clock(self,t,w):return float(np.interp(t,self.ts,self.clocks[w]))
