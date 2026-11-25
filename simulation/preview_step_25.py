"""Render sampled head poses using the Stage 22 studio renderer."""
import argparse, tempfile, sys
from pathlib import Path
import numpy as np
from PIL import Image, ImageDraw
from step_25_head_articulation import ROOT,OUTPUT,read_glb,array
from step_24_transmission import pose_glb
sys.path.insert(0,str(ROOT/'rendering'))
from step_22_studio import render

def preview(source=OUTPUT,output=ROOT/'renders/dragonfly_head_motion.gif',clip='hover',frames=8):
    if not 4<=frames<=60:raise ValueError('Use 4–60 frames')
    d,b=read_glb(source);a=next(a for a in d['animations'] if a['name']==clip);end=max(float(array(d,b,s['input'])[-1]) for s in a['samplers']);images=[]
    with tempfile.TemporaryDirectory() as td:
        p=Path(td)
        for t in np.arange(frames)*end/frames:
            pose_glb(source,p/'pose.glb',clip,float(t))
            render(p/'pose.glb',p/'frame.png',width=800,height=600,azimuth=35,elevation=30,zoom=6,target=[.045,0,.005],preset='graphite',ground=False)
            with Image.open(p/'frame.png') as im:images.append(im.convert('RGB'))
    output=Path(output);output.parent.mkdir(parents=True,exist_ok=True);images[0].save(output,save_all=True,append_images=images[1:],duration=round(end*1000/frames),loop=0,disposal=2)
    sheet=Image.new('RGB',(800,650),(18,22,30));draw=ImageDraw.Draw(sheet)
    for j in range(4):
        i=j*frames//4;x=j%2*400;y=j//2*325;sheet.paste(images[i].resize((400,300),Image.Resampling.LANCZOS),(x,y));draw.text((x+8,y+303),f'{clip} | {i*end/frames:.2f} s',(220,230,240))
    sheet.save(output.with_suffix('.png'))
if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--source',type=Path,default=OUTPUT);p.add_argument('--output',type=Path,default=ROOT/'renders/dragonfly_head_motion.gif');p.add_argument('--clip',choices=['hover','forward','turn_left'],default='hover');p.add_argument('--frames',type=int,default=8);preview(**vars(p.parse_args()))
