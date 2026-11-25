"""Scripted Last Flight scenario: real model poses, fixed floor, tracked camera and MP4."""
import argparse, copy, json, math, shutil, subprocess, sys, time
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation, Slerp
ROOT=Path(__file__).resolve().parents[1]
for directory in ('rendering','simulation','optimization'):
    sys.path.insert(0,str(ROOT/directory))
from step_16_geometry import read_glb, array
from scene import Scene
from step_22_studio import render

ACTIONS=('idle','power_on','scan','takeoff','flight','failure','unstable','fall','impact','twitch','shutdown')
WINGS=('fore_left','fore_right','hind_left','hind_right')
SOURCE=ROOT/'models/dragonfly_legs_animated.glb'

def smooth(x):
    x=float(np.clip(x,0,1));return x*x*(3-2*x)

def load_scenario(path):
    s=json.loads(Path(path).read_text());events=s['timeline']; ts=[float(e['time']) for e in events]
    if tuple(e['action'] for e in events)!=ACTIONS or ts[0]!=0 or not all(np.isfinite(ts)) or any(b<=a for a,b in zip(ts,ts[1:])) or not ts[-1]<float(s['duration'])<=120:
        raise ValueError('Expected the eleven ordered Last Flight actions, starting at 0 and ending before duration (max 120 s).')
    if s['failed_wing'] not in WINGS or not 0<float(s['flight_height_m'])<=2 or not 0<float(s['travel_m'])<=2 or s['preset'] not in ('cinematic','graphite','gallery'):
        raise ValueError('Invalid wing, distance, altitude or lighting preset')
    return s

def state(s,t):
    e={v['action']:float(v['time']) for v in s['timeline']};t=float(np.clip(t,0,s['duration']))
    def p(a,b):return float(np.clip((t-e[a])/(e[b]-e[a]),0,1))
    def q(a,b):return smooth(p(a,b))
    phase=next(a for a in reversed(ACTIONS) if t>=e[a]);h=s['flight_height_m']; travel=s['travel_m']
    # A scripted cinematic trajectory, not a rigid-body collision simulation.
    height=h*q('takeoff','flight')-h*.18*q('failure','fall')
    height*=1-p('fall','impact')**2
    if e['impact']<=t<e['twitch']:
        u=p('impact','twitch');height=.032*abs(math.sin(2*math.pi*u))*(1-u)**2
    x=travel*(.12*q('takeoff','flight')+.58*q('flight','fall')+.27*p('fall','impact')+.03*q('impact','twitch'))
    sign=1 if s['failed_wing'].endswith('left') else -1
    roll=sign*(12*q('failure','unstable')+18*q('unstable','fall')+65*q('fall','impact')+35*q('impact','twitch'))
    roll+=sign*8*math.sin(8*t)*q('failure','unstable')*(1-q('fall','impact'))
    pitch=-9*q('takeoff','flight')+9*q('flight','failure')+24*q('fall','impact')-19*q('impact','twitch')
    yaw=sign*20*q('unstable','impact')
    power=q('power_on','scan')*(1-q('twitch','shutdown'))
    if phase in ('failure','unstable','fall','impact','twitch'):power*=.65+.35*math.sin(29*t)**2
    amplitude=q('scan','takeoff')*(1-q('fall','twitch'))
    twitch=(1-q('twitch','shutdown'))*.16*abs(math.sin(13*t)) if phase=='twitch' else 0.
    return dict(time=t,action=phase,x=x,height=max(0.,height),roll=roll,pitch=pitch,yaw=yaw,power=power,amplitude=amplitude,twitch=twitch,failed=q('failure','unstable'),head=q('power_on','scan')*(1-q('fall','impact')))

class Actor:
    def __init__(self,source=SOURCE):
        self.doc,self.raw=read_glb(source); self.rest=copy.deepcopy(self.doc['nodes']);self.materials=copy.deepcopy(self.doc['materials']);self.tracks=[]
        clip=next(a for a in self.doc['animations'] if a['name']=='hover')
        for ch in clip['channels']:
            sm=clip['samplers'][ch['sampler']];ts=array(self.doc,self.raw,sm['input']);vs=array(self.doc,self.raw,sm['output']);i=ch['target']['node'];key=ch['target']['path'];name=self.rest[i]['name'];
            self.tracks.append((i,key,name,ts,vs,Slerp(ts,Rotation.from_quat(vs)) if key=='rotation' else None))
    def pose(self,s,t):
        st=state(s,t);doc=dict(self.doc);doc['nodes']=copy.deepcopy(self.rest);doc['materials']=copy.deepcopy(self.materials);doc.pop('animations',None)
        events={a['action']:a['time'] for a in s['timeline']};failed=s['failed_wing'];stop=events['shutdown'];clock=min(t,stop)
        for i,key,name,ts,vs,interp in self.tracks:
            wing=next((w for w in WINGS if w in name),None);w=st['amplitude']+st['twitch'];local=clock
            if wing==failed:
                w*=1-st['failed'];local=min(clock,events['failure']+.24)
            if name.startswith('rig_head'):w=st['head'];local=clock*.6
            elif name.startswith('rig_abdomen'):w=st['amplitude']*.75+st['twitch'];local=clock*.7
            elif name.startswith('rig_leg'):w=.25*st['amplitude']+st['twitch'];local=clock*.8
            elif name.startswith('rig_drive'):
                # Freeze failed gears; other drive tracks wind down with the wings.
                local=min(local,events['impact']);w=1. if t>=events['scan'] else st['head']
            w=float(np.clip(w,0,1));vtime=float(ts[0]+(local-ts[0])%(ts[-1]-ts[0]));rest=self.rest[i].get(key,[0,0,0,1] if key=='rotation' else [1,1,1] if key=='scale' else [0,0,0])
            if key=='rotation':
                val=interp(vtime).as_quat();val=Slerp([0,1],Rotation.from_quat([rest,val]))(w).as_quat()
            else:
                val=np.array([np.interp(vtime,ts,vs[:,j]) for j in range(3)]);val=np.asarray(rest)*(1-w)+val*w
            doc['nodes'][i][key]=val.tolist()
        for m in doc['materials']:
            if 'emissiveFactor' in m:m['emissiveFactor']=(np.asarray(m['emissiveFactor'])*st['power']).tolist()
            if 'optical glass' in m.get('name','').lower():
                p=m['pbrMetallicRoughness'];c=p.get('baseColorFactor',[1,1,1,1]);p['baseColorFactor']=[v*(.32+.68*st['power']) for v in c[:3]]+[c[3]]
        # Transform the complete hierarchy, preserving local motion.
        root=doc['scenes'][doc.get('scene',0)]['nodes'][0]
        doc['nodes'][root]['rotation']=Rotation.from_euler('xyz',[st['roll'],st['pitch'],st['yaw']],degrees=True).as_quat().tolist()
        scene=Scene((doc,self.raw))
        # Place the lowest actual posed mesh above the same floor in every frame.
        shift=np.array([st['x'],0,st['height']-scene.bounds[0,2]+.0005],dtype='f4')
        scene.vertices[:,:3]+=shift;scene.bounds+=shift
        for d in scene.draws:d.center+=shift
        return scene,st

def shot(st,scene):
    # Track the actor rather than recomputing zoom from its changing geometry.
    a=st['action'];center=scene.bounds.mean(axis=0)
    if a in ('idle','power_on','scan','twitch','shutdown'):return 38,26,.17,center
    if a in ('takeoff','flight'):return 58,24,.24,center-np.array([0,0,min(st['height']*.4,.08)])
    if a in ('failure','unstable'):return 38,31,.21,center
    return 55,28,.26,center-np.array([0,0,min(st['height']*.3,.07)])

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--scenario',type=Path,default=ROOT/'scenarios/last_flight.json');p.add_argument('--output',type=Path,default=ROOT/'renders/last_flight.mp4');p.add_argument('--width',type=int,default=640);p.add_argument('--height',type=int,default=360);p.add_argument('--fps',type=int,default=12);p.add_argument('--supersample',type=int,default=1);p.add_argument('--preview',action='store_true',help='Render one image per event, held for half a second');p.add_argument('--crf',type=int,default=17);p.add_argument('--frames-dir',type=Path);p.add_argument('--resume',action='store_true');args=p.parse_args()
    if not 1<=args.fps<=120 or not 0<=args.crf<=51 or min(args.width,args.height)<64 or args.width%2 or args.height%2:raise ValueError('Invalid fps, CRF or dimensions')
    if not shutil.which('ffmpeg'):raise RuntimeError('Install ffmpeg: sudo apt install ffmpeg')
    s=load_scenario(args.scenario);actor=Actor();out=args.output.resolve();out.parent.mkdir(parents=True,exist_ok=True);frames=args.frames_dir or out.with_suffix('').with_name(out.stem+'_frames');frames.mkdir(parents=True,exist_ok=True)
    times=[e['time']+min(.35, (s['timeline'][i+1]['time']-e['time'])/2 if i+1<len(s['timeline']) else .35) for i,e in enumerate(s['timeline'])] if args.preview else np.arange(math.ceil(s['duration']*args.fps))/args.fps
    if args.preview:times.append(19.2 if s['timeline'][7]['time']==18 else (s['timeline'][7]['time']+s['timeline'][8]['time'])/2);times.sort()
    manifest={'scenario':s,'width':args.width,'height':args.height,'fps':args.fps,'supersample':args.supersample,'preview':args.preview};meta=frames/'manifest.json'
    if args.resume and (not meta.exists() or json.loads(meta.read_text())!=manifest):raise ValueError('Resume requires matching frame manifest')
    if not args.resume:
        for f in frames.glob('frame_*.png'):f.unlink()
        meta.write_text(json.dumps(manifest,indent=2))
    start=time.perf_counter();stats=[]
    for i,t in enumerate(times):
        path=frames/f'frame_{i:06d}.png'
        if args.resume and path.exists():continue
        scene,st=actor.pose(s,float(t));a,e,extent,target=shot(st,scene)
        # fixed camera extent and fixed z=0 ground plane.
        temp=path.with_suffix('.tmp.png');render(scene,temp,width=args.width,height=args.height,azimuth=a,elevation=e,projection='orthographic',supersample=args.supersample,target=target,preset=s['preset'],ground=True,camera_extent=extent,floor_z=0.);temp.replace(path)
        stats.append(st);print(f'Frame {i+1}/{len(times)} | {st["action"]}',flush=True)
    rate=2 if args.preview else args.fps
    subprocess.run(['ffmpeg','-nostdin','-hide_banner','-loglevel','error','-y','-framerate',str(rate),'-i',str(frames/'frame_%06d.png'),'-c:v','libx264','-preset','fast','-crf',str(args.crf),'-pix_fmt','yuv420p','-movflags','+faststart',str(out)],check=True)
    print(json.dumps({'output':str(out),'frames':len(times),'fps':rate,'seconds':len(times)/rate,'render_seconds':round(time.perf_counter()-start,2),'preview':args.preview},indent=2))
if __name__=='__main__':main()
