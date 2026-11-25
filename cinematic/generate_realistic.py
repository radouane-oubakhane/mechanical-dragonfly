"""Last Flight v2: continuous camera, reduced-order collision solver and real model rendering.
Requires the V2.0 project and original Last Flight update; no new Python dependencies.
"""
import argparse, copy, hashlib, json, math, shutil, subprocess, time
from pathlib import Path
import numpy as np
from PIL import Image
from scipy.spatial.transform import Rotation, Slerp
from generate import Actor as BaseActor, ROOT, SOURCE, WINGS, Scene, load_scenario
from physics_v2 import Motion, Physics, COM, hull_points, smooth
from step_22_studio import render
from studio import PRESETS, Studio

# Controlled, neutral key and fill. No exposure flicker during movement.
PRESETS['natural']=Studio(((.45,-.55,.9),(-.7,.4,.75),(.55,.75,.5)),((3.4,3.25,3.1),(1.5,1.65,1.85),(1.9,2.0,2.15)),(.16,.17,.19),(.18,.18,.19),(.06,.065,.075),.72,.006)

class Actor(BaseActor):
    def __init__(self,s,source=SOURCE,wing_rate=1.):
        super().__init__(source); self.s=s; self.wing_rate=wing_rate
        neutral=Scene((self.doc,self.raw)); self.motion=Motion(s,hull_points(neutral.vertices[:,:3])); self.root=self.doc['scenes'][self.doc.get('scene',0)]['nodes'][0]
    def pose(self,t):
        m=self.motion; s=self.s; e=m.e; p,r,contact=m.sample(t);doc=dict(self.doc);doc['nodes']=copy.deepcopy(self.rest);doc['materials']=copy.deepcopy(self.materials);doc.pop('animations',None)
        q=lambda a,b:m.progress(t,a,b)
        power=q('power_on','scan')*(1-q('twitch','shutdown'))
        if t>=m.fall:power*=.42+.58*(1-smooth((t-m.fall)/1.4))
        # Flight phases integrated rather than restarting each shot. All external
        # articulation settles into the neutral collision proxy before impact.
        active=1-q('unstable','fall'); head=q('power_on','scan')*active
        for i,key,name,ts,vs,interp in self.tracks:
            wing=next((w for w in WINGS if w in name),None)
            w=0.; clock=t
            if name.startswith('rig_wing'):
                w=m.amplitude(t,wing);clock=self.wing_rate*m.clock(t,wing)
            elif name.startswith('rig_drive'):
                # Keep continuous rotating components at their stopped phase;
                # no slerping full-turn gear rotations toward identity.
                clock=self.wing_rate*m.clock(t,wing); w=1.
            elif name.startswith('rig_head'):
                w=head;clock=t*.5
            elif name.startswith('rig_abdomen'):
                w=.5*q('takeoff','flight')*active;clock=t*.65
            elif name.startswith('rig_leg'):
                # Feet stay planted on takeoff pad; no walk-cycle foot sliding.
                w=.12*q('takeoff','flight')*active;clock=t*.45
            if not w:continue
            vt=float(ts[0]+(clock-ts[0])%(ts[-1]-ts[0]));rest=self.rest[i].get(key,[0,0,0,1] if key=='rotation' else [1,1,1] if key=='scale' else [0,0,0])
            if key=='rotation':
                val=interp(vt).as_quat();val=val if w==1 else Slerp([0,1],Rotation.from_quat([rest,val]))(w).as_quat()
            else:
                val=np.array([np.interp(vt,ts,vs[:,j]) for j in range(3)]);val=np.asarray(rest)*(1-w)+val*w
            doc['nodes'][i][key]=val.tolist()
        for mat in doc['materials']:
            if 'emissiveFactor' in mat:mat['emissiveFactor']=(np.asarray(mat['emissiveFactor'])*power).tolist()
        doc['nodes'][self.root]['rotation']=r.as_quat().tolist()
        doc['nodes'][self.root]['translation']=(p-r.apply(COM)).tolist()
        scene=Scene((doc,self.raw))
        return scene,dict(position=p,contact=contact,power=float(power),time=t)

def camera_at(m,t):
    # No camera cuts at event boundaries. Track COM, not per-frame mesh bounds.
    p,_,_=m.sample(t); e=m.e; q=lambda a,b:m.progress(t,a,b)
    wide=q('scan','flight')*(1-smooth((t-(m.impact or m.fall)-.3)/1.6))
    az=35+12*q('takeoff','flight')-12*smooth((t-m.fall)/2.5)
    el=31+5*q('takeoff','flight')-3*smooth((t-m.fall)/2.5)
    extent=.145+.11*wide
    # Look slightly ahead in flight and keep ground near lower frame edge.
    target=p+np.array([.015+.02*wide,0.,-.04*wide])
    return az,el,extent,target

def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--scenario',type=Path,default=ROOT/'scenarios/last_flight.json');ap.add_argument('--output',type=Path,default=None);ap.add_argument('--width',type=int,default=1280);ap.add_argument('--height',type=int,default=720);ap.add_argument('--fps',type=int,default=30);ap.add_argument('--supersample',type=int,default=1);ap.add_argument('--crf',type=int,default=16);ap.add_argument('--preset',choices=['natural','graphite','cinematic','gallery'],default='natural');ap.add_argument('--start',type=float,default=0.);ap.add_argument('--end',type=float);ap.add_argument('--preview',action='store_true',help='Continuous short crash preview (not a storyboard)');ap.add_argument('--resume',action='store_true');ap.add_argument('--frames-dir',type=Path);ap.add_argument('--physics-only',action='store_true');ap.add_argument('--wing-rate',type=float,default=1.,help='Multiply the authored 3 Hz wing and drive playback speed (visual only)');ap.add_argument('--motion-samples',type=int,default=1,help='Temporal samples for approximate 180-degree shutter blur (1–8)');a=ap.parse_args()
    if a.preview:a.width,a.height,a.fps,a.supersample=480,270,24,1
    if not 1<=a.fps<=120 or min(a.width,a.height)<64 or a.width%2 or a.height%2 or a.supersample not in (1,2,3) or not 0<=a.crf<=51 or not 1<=a.motion_samples<=8 or not .25<=a.wing_rate<=12:ap.error('Invalid video parameters')
    s=load_scenario(a.scenario);actor=Actor(s,wing_rate=a.wing_rate);m=actor.motion
    if a.preview:a.start,a.end=max(0,m.fall-.3),min(s['duration'],m.fall+1.2)
    out=(a.output or ROOT/'renders'/('last_flight_v2_preview.mp4' if a.preview else 'last_flight_v2.mp4')).resolve();out.parent.mkdir(parents=True,exist_ok=True)
    run=m.run; report={'gravity_m_s2':m.cfg.gravity,'friction':m.cfg.friction,'restitution':m.cfg.restitution,'physics_hz':1/m.cfg.step,'impact_time_s':m.impact,'fall_start_s':m.fall,'minimum_proxy_gap_m':float(run['gap'].min()),'final_speed_m_s':float(np.linalg.norm(run['velocity'][-1])),'settled':bool(run['sleep'][-1]),'collision_proxy_vertices':len(m.points)}
    np.savetxt(out.with_suffix('.csv'),np.column_stack([m.fall+run['time'],run['position'],run['velocity'],run['omega'],run['contact'],run['impulse'],run['gap']]),delimiter=',',header='time_s,x_m,y_m,z_m,vx,vy,vz,wx,wy,wz,contact,normal_impulse_Ns,gap_m',comments='');out.with_suffix('.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2),flush=True)
    if a.physics_only:return
    if not shutil.which('ffmpeg'):raise RuntimeError('Install ffmpeg')
    end=float(s['duration'] if a.end is None else a.end)
    if not 0<=a.start<end<=s['duration']:ap.error('Invalid time interval')
    times=np.arange(a.start,end-1e-8,1/a.fps);frames=a.frames_dir or out.with_name(out.stem+'_frames');frames.mkdir(parents=True,exist_ok=True);meta=frames/'manifest.json'
    manifest=dict(version=2,scenario=s,start=a.start,end=end,width=a.width,height=a.height,fps=a.fps,supersample=a.supersample,preset=a.preset,wing_rate=a.wing_rate,motion_samples=a.motion_samples,source_sha256=hashlib.sha256(SOURCE.read_bytes()).hexdigest())
    if a.resume:
        if not meta.exists() or json.loads(meta.read_text())!=manifest:raise ValueError('Resume requires identical settings and source model')
    else:
        for f in frames.glob('frame_*.png'):f.unlink()
        meta.write_text(json.dumps(manifest,indent=2))
    for i,t in enumerate(times):
        path=frames/f'frame_{i:06d}.png'
        if a.resume and path.exists():continue
        tmp=path.with_suffix('.tmp.png'); accum=None
        # Approximate exposure integration in display-linear space. This is not
        # path-traced shutter integration; it reduces frame-to-frame strobing.
        for j in range(a.motion_samples):
            sample=float(np.clip(t+((j+.5)/a.motion_samples-.5)*.5/a.fps,0,s['duration']))
            scene,st=actor.pose(sample);az,el,extent,target=camera_at(m,sample)
            render(scene,tmp,width=a.width,height=a.height,azimuth=az,elevation=el,projection='perspective',supersample=a.supersample,target=target,preset=a.preset,ground=True,camera_extent=extent,floor_z=0.,floor_fragment="studio_floor_realistic.frag")
            if a.motion_samples>1:
                with Image.open(tmp) as im: rgb=np.asarray(im.convert('RGB'),dtype=np.float32)/255
                lin=np.where(rgb<=.04045,rgb/12.92,((rgb+.055)/1.055)**2.4)
                accum=lin if accum is None else accum+lin
        if accum is not None:
            lin=accum/a.motion_samples; rgb=np.where(lin<=.0031308,12.92*lin,1.055*lin**(1/2.4)-.055)
            Image.fromarray(np.uint8(np.clip(rgb,0,1)*255+.5)).save(tmp)
        tmp.replace(path);print(f'Frame {i+1}/{len(times)} | {t:.3f}s',flush=True)
    subprocess.run(['ffmpeg','-nostdin','-hide_banner','-loglevel','error','-y','-framerate',str(a.fps),'-i',str(frames/'frame_%06d.png'),'-frames:v',str(len(times)),'-c:v','libx264','-preset','fast','-crf',str(a.crf),'-pix_fmt','yuv420p','-movflags','+faststart',str(out)],check=True);print(out)
if __name__=='__main__':main()
