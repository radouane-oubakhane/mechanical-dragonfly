"""Render a slow-motion transmission cycle with fixed framing parameters."""
from pathlib import Path
import tempfile, argparse,sys
import numpy as np
from PIL import Image,ImageDraw
from step_24_transmission import ROOT,OUTPUT,pose_glb,read_glb
sys.path.insert(0,str(ROOT/'rendering'))
from step_22_studio import render

def preview(source=OUTPUT,output=ROOT/'renders/dragonfly_transmission.gif',clip='hover',frames=8):
    if not 4<=frames<=60:raise ValueError('Use 4–60 frames')
    doc,_=read_glb(source);a=next(a for a in doc['animations'] if a['name']==clip);freq=a['extras']['wing_controls']['fore_left']['frequency'];images=[]
    with tempfile.TemporaryDirectory() as temp:
        p=Path(temp)
        for t in np.arange(frames)/(freq*frames):
            pose_glb(source,p/'pose.glb',clip,float(t));render(p/'pose.glb',p/'frame.png',width=900,height=600,azimuth=40,elevation=55,zoom=4.3,target=[.006,0,.012],preset='graphite',ground=False)
            with Image.open(p/'frame.png') as im:images.append(im.convert('RGB'))
    output=Path(output);output.parent.mkdir(parents=True,exist_ok=True);images[0].save(output,save_all=True,append_images=images[1:],duration=130,loop=0,disposal=2)
    sheet=Image.new('RGB',(900,640),(18,22,30));draw=ImageDraw.Draw(sheet)
    for i in range(4):
        x=i%2*450;y=i//2*320;sheet.paste(images[i*frames//4].resize((450,300),Image.Resampling.LANCZOS),(x,y));draw.text((x+8,y+301),f'{clip} | {i*90} deg',(220,230,240))
    sheet.save(output.with_suffix('.png'))
if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--source',type=Path,default=OUTPUT);p.add_argument('--output',type=Path,default=ROOT/'renders/dragonfly_transmission.gif');p.add_argument('--clip',choices=['hover','forward','turn_left'],default='hover');p.add_argument('--frames',type=int,default=8);preview(**vars(p.parse_args()))
