"""Cinematic shot planning, fixed camera framing, PNG and H.264 export."""
import argparse, json, math, shutil, subprocess, sys, tempfile, time
from dataclasses import dataclass
from pathlib import Path
import numpy as np
from PIL import Image, ImageDraw, ImageFont
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'rendering'));sys.path.insert(0,str(ROOT/'simulation'))
from step_22_studio import render
from scene import Scene
from step_23_wing_kinematics import read_glb,array
from step_24_transmission import pose_glb
STATIC=ROOT/'models/dragonfly_static_balanced_iridescent.glb'
ANIMATED=ROOT/'models/dragonfly_legs_animated.glb'
STAGES=[ROOT/'models'/('step_01_chassis.glb' if i==1 else f'step_{i:02d}_assembled.glb') for i in range(1,16)]
LABELS=('Chassis','Power core','Electronics','Transmission','Actuators','Wing joints','Head','Compound eyes','Facial sensors','Abdominal spine','Abdominal armor','Legs','Wing skeletons','Membranes','Final armor')
SEQUENCES=('showcase','assembly','turntable','macro','motion')
@dataclass(frozen=True)
class Shot:
    kind:str
    source:str
    azimuth:float
    elevation:float
    zoom:float
    target:tuple|None=None
    clip:str|None=None
    seconds:float=0.
    label:str=''

def plan(sequence='showcase',fps=24,duration=4.,clip='hover',motion_speed=.25):
    if sequence not in SEQUENCES or not isinstance(fps,int) or not 1<=fps<=120 or not math.isfinite(duration) or not .1<=duration<=120 or not math.isfinite(motion_speed) or not 0<motion_speed<=4:raise ValueError('Invalid sequence, fps, duration or speed')
    if clip not in ('hover','forward','turn_left'):raise ValueError('Unknown motion clip')
    n=max(2,round(duration*fps))
    def assembly():return [Shot('assembly',str(p),35,52,1.,label=f'{i:02d} / 15  {LABELS[i-1]}') for i,p in enumerate(STAGES,1) for _ in range(max(1,math.ceil(n/15)))]
    def turntable():return [Shot('turntable',str(STATIC),35+360*i/n,42,1.,label='FULL ASSEMBLY') for i in range(n)]
    def macro():return [Shot('macro',str(STATIC),45+35*(3*u*u-2*u**3),28+10*u,3.6,(.032,0,.005),label='OPTICAL & MECHANICAL DETAIL') for u in np.linspace(0,1,n)]
    def motion():return [Shot('motion',str(ANIMATED),35,48,1.,clip=clip,seconds=i/fps*motion_speed,label=f'{clip.upper()} / SLOW MOTION') for i in range(n)]
    f={'assembly':assembly,'turntable':turntable,'macro':macro,'motion':motion}
    return sum((f[k]() for k in f),[]) if sequence=='showcase' else f[sequence]()

def annotate(path,shot):
    with Image.open(path) as src:im=src.convert('RGB')
    d=ImageDraw.Draw(im);w,h=im.size
    try:font=ImageFont.truetype('DejaVuSans.ttf',max(12,round(h*.031)))
    except OSError:font=ImageFont.load_default()
    x=max(10,round(w*.025));y=h-max(40,round(h*.11))
    d.rectangle((0,y,w,h),fill=(12,16,22));d.text((x,y+4),'PROJECT DRAGONFLY',font=font,fill=(230,235,240));d.text((x,y+5+max(15,round(h*.035))),shot.label,font=font,fill=(128,203,213));im.save(path)

def encode(frames,output,fps):
    ff=shutil.which('ffmpeg')
    if not ff:raise RuntimeError('Install FFmpeg: sudo apt install ffmpeg')
    output=Path(output);output.parent.mkdir(parents=True,exist_ok=True)
    subprocess.run([ff,'-hide_banner','-loglevel','error','-y','-framerate',str(fps),'-i',str(Path(frames)/'frame_%06d.png'),'-c:v','libx264','-preset','fast','-crf','19','-pix_fmt','yuv420p','-movflags','+faststart',str(output)],check=True)

def film(output,sequence='showcase',width=1280,height=720,fps=24,duration=4.,supersample=1,preset='cinematic',clip='hover',motion_speed=.25,ground=False,titles=False,keep_frames=None):
    if min(width,height)<64 or width%2 or height%2 or max(width,height)*supersample>8192 or supersample not in (1,2,3):raise ValueError('Use even dimensions >=64 and supported supersampling')
    output=Path(output).resolve()
    if output.suffix.lower()!='.mp4':raise ValueError('Output must be .mp4')
    if not shutil.which('ffmpeg'):raise RuntimeError('Install FFmpeg before rendering')
    shots=plan(sequence,fps,duration,clip,motion_speed)
    ref=Scene(STATIC);bounds=ref.bounds.copy();del ref
    extent=float(np.linalg.norm(bounds[1]-bounds[0])/2*1.08/min(width/height,1.))
    if any(s.clip for s in shots):
        d,b=read_glb(ANIMATED);a=next(a for a in d['animations'] if a['name']==clip);end=max(float(array(d,b,s['input'])[-1]) for s in a['samplers']);del d,b
    start=time.perf_counter();unique=0;last=None;lastpath=None
    with tempfile.TemporaryDirectory(prefix='dragonfly-film-') as td:
        work=Path(td);frames=work/'frames';frames.mkdir()
        for i,shot in enumerate(shots):
            path=frames/f'frame_{i:06d}.png'
            if shot==last:shutil.copyfile(lastpath,path)
            else:
                source=Path(shot.source)
                if shot.clip:source=work/'pose.glb';pose_glb(shot.source,source,shot.clip,shot.seconds%end)
                render(source,path,width=width,height=height,azimuth=shot.azimuth,elevation=shot.elevation,projection='perspective' if shot.kind=='macro' else 'orthographic',zoom=shot.zoom,supersample=supersample,target=shot.target,preset=preset,ground=ground,camera_bounds=bounds,camera_extent=extent)
                if titles:annotate(path,shot)
                unique+=1;last=shot;lastpath=path;print(f'Frame {i+1}/{len(shots)} | {shot.kind}',flush=True)
        encode(frames,output,fps)
        if keep_frames:
            dest=Path(keep_frames);dest.mkdir(parents=True,exist_ok=True)
            for p in frames.glob('frame_*.png'):shutil.copyfile(p,dest/p.name)
    report=dict(sequence=sequence,frames=len(shots),unique_renders=unique,fps=fps,duration_seconds=len(shots)/fps,resolution=[width,height],preset=preset,ground=ground,titles=titles,motion_speed=motion_speed,render_seconds=round(time.perf_counter()-start,2),output=output.name)
    output.with_suffix('.json').write_text(json.dumps(report,indent=2)+'\n');return report

def still(output,width=1920,height=1080,supersample=2,preset='cinematic',composition='hero'):
    from studio import COMPOSITIONS
    a,e,z,t=COMPOSITIONS[composition]
    return render(STATIC,output,width=width,height=height,azimuth=a,elevation=e,zoom=z,target=t,supersample=supersample,preset=preset,ground=True)

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--sequence',choices=(*SEQUENCES,'still'),default='showcase');p.add_argument('--output',type=Path);p.add_argument('--width',type=int,default=1280);p.add_argument('--height',type=int,default=720);p.add_argument('--fps',type=int,default=24);p.add_argument('--duration',type=float,default=4.);p.add_argument('--supersample',type=int,default=1);p.add_argument('--preset',choices=['graphite','gallery','cinematic'],default='cinematic');p.add_argument('--clip',choices=['hover','forward','turn_left'],default='hover');p.add_argument('--motion-speed',type=float,default=.25);p.add_argument('--ground',action='store_true');p.add_argument('--titles',action='store_true');p.add_argument('--keep-frames',type=Path);p.add_argument('--composition',choices=['hero','top','portrait'],default='hero')
    a=vars(p.parse_args());composition=a.pop('composition');sequence=a['sequence'];a['output']=a['output'] or ROOT/'renders'/('dragonfly_cinematic.png' if sequence=='still' else f'dragonfly_{sequence}.mp4')
    if sequence=='still':result=still(**{k:a[k] for k in ('output','width','height','supersample','preset')},composition=composition)
    else:result=film(**a)
    print(json.dumps(result,indent=2))
if __name__=='__main__':main()
