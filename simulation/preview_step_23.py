"""Render an actual slow-motion wing cycle using the existing studio renderer."""
from pathlib import Path
import argparse, tempfile, sys
from PIL import Image, ImageDraw
import numpy as np
from step_23_wing_kinematics import pose_glb, OUTPUT, ROOT, read_glb
sys.path.insert(0,str(ROOT/'rendering'))
from step_22_studio import render

def preview(source=OUTPUT, output=ROOT/'renders/dragonfly_wing_motion.gif', clip='hover', frames=12):
    if not 4<=frames<=60: raise ValueError('Use 4–60 preview frames')
    doc,_=read_glb(source); a=next(a for a in doc['animations'] if a['name']==clip)
    freq=a['extras']['wing_controls']['fore_left']['frequency']
    images=[]
    with tempfile.TemporaryDirectory() as temp:
        temp=Path(temp)
        for t in np.arange(frames)/(freq*frames):
            pose_glb(source,temp/'pose.glb',clip,float(t))
            render(temp/'pose.glb',temp/'frame.png',width=960,height=640,azimuth=35,elevation=34,zoom=.95,preset='graphite',ground=False)
            with Image.open(temp/'frame.png') as im: images.append(im.convert('RGB'))
    output=Path(output);output.parent.mkdir(parents=True,exist_ok=True)
    images[0].save(output,save_all=True,append_images=images[1:],duration=100,loop=0,disposal=2)
    sheet=Image.new('RGB',(960,680),(15,20,28));draw=ImageDraw.Draw(sheet)
    for i in range(4):
        x=(i%2)*480;y=(i//2)*340; index=i*frames//4
        sheet.paste(images[index].resize((480,320),Image.Resampling.LANCZOS),(x,y))
        draw.text((x+12,y+323),f'{clip} | phase {index*360/frames:.0f} deg',(220,230,240))
    sheet.save(output.with_suffix('.png'))
    print(f'{frames} rendered poses | {output}')

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--source',type=Path,default=OUTPUT);p.add_argument('--output',type=Path,default=ROOT/'renders/dragonfly_wing_motion.gif');p.add_argument('--clip',choices=['hover','forward','turn_left'],default='hover');p.add_argument('--frames',type=int,default=12);preview(**vars(p.parse_args()))
