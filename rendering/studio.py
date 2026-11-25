"""Deterministic linear-HDR studio environments and projected soft ground shadows."""
from dataclasses import dataclass
import numpy as np
from PIL import Image, ImageDraw, ImageFilter

@dataclass(frozen=True)
class Studio:
    directions: tuple
    radiance: tuple
    ambient: tuple
    floor: tuple
    background: tuple
    shadow_strength: float
    softness: float

PRESETS = {
    'graphite': Studio(((.35,-.65,1),(-.8,.6,.7),(.8,.7,.45)), ((3.5,3.2,2.9),(1.4,1.8,2.5),(2.4,2.2,2.0)),(.12,.15,.20),(.16,.18,.21),(.035,.045,.065),.72,.004),
    'gallery': Studio(((.45,-.4,1),(-.7,.5,.9),(.3,.9,.65)), ((3.2,3.1,3),(2.4,2.5,2.8),(1.6,1.6,1.5)),(.25,.27,.30),(.55,.57,.60),(.28,.30,.34),.55,.006),
    'cinematic': Studio(((.4,-.7,.8),(-.5,.8,.65),(.9,.5,.3)), ((4.0,2.8,1.7),(.55,1.15,2.9),(1.8,2.2,3.0)),(.065,.085,.13),(.085,.105,.14),(.018,.025,.04),.78,.005),
}
COMPOSITIONS = {'hero': (30,50,1.,None), 'top': (12,76,1.,None), 'portrait': (45,28,3.4,(.032,0,.005))}

def normalized(v):
    v=np.asarray(v,float);return v/np.linalg.norm(v)

def environment(studio, width=512, height=256):
    """Equirectangular RGB linear radiance, with softbox lobes above unity."""
    u=(np.arange(width)+.5)/width;v=(np.arange(height)+.5)/height
    lon=(u-.5)*2*np.pi;lat=(v-.5)*np.pi
    r=np.stack(np.broadcast_arrays(np.cos(lat[:,None])*np.cos(lon),np.cos(lat[:,None])*np.sin(lon),np.sin(lat[:,None])+np.zeros_like(lon)),axis=-1)
    out=np.broadcast_to(np.array(studio.ambient),(height,width,3)).copy()
    out+=np.maximum(r[...,2:],0)*.14
    for d,c,k in zip(studio.directions,studio.radiance,(22,12,18)):
        # Smooth-edged luminous panels represented by directional lobes.
        out+=np.maximum(r@normalized(d),0)[...,None]**k*np.array(c)*1.8
    return np.ascontiguousarray(out,dtype='f4')

def shadow_atlas(scene,studio,resolution=768,floor_z=None):
    """Project opaque geometry along key light to floor; add near-floor contact mask.
    Output RG channels: blurred key silhouette and local contact shadow. This is
    not a ray-traced shadow or general ambient occlusion solution.
    """
    z=float(scene.bounds[0,2])-.0006 if floor_z is None else float(floor_z)
    light=normalized(studio.directions[0]); slope=light[:2]/light[2]
    opaque=np.concatenate([scene.vertices[d.start:d.start+d.count,:3] for d in scene.draws if not scene.transparent(d)]).reshape(-1,3,3)
    flat=opaque[...,:2]; heights=opaque[...,2]-z
    projected=flat-heights[...,None]*slope
    lo=np.minimum(flat.min(axis=(0,1)),projected.min(axis=(0,1)))-.035
    hi=np.maximum(flat.max(axis=(0,1)),projected.max(axis=(0,1)))+.035
    # Square atlas for isotropic blur; center at model/shadow bounds.
    center=(lo+hi)/2; extent=float(max(hi-lo));lo=center-extent/2;hi=center+extent/2
    def raster(tri):
        img=Image.new('L',(resolution,resolution));draw=ImageDraw.Draw(img)
        pix=np.rint((tri-lo)/extent*(resolution-1)).astype(np.int32)
        for a in pix.tolist():draw.polygon([tuple(p) for p in a],fill=255)
        return img
    key=raster(projected).filter(ImageFilter.GaussianBlur(studio.softness*resolution/extent))
    close=flat[heights.max(axis=1)<.006]
    contact=raster(close).filter(ImageFilter.GaussianBlur(.0018*resolution/extent))
    atlas=np.ascontiguousarray(np.stack([np.asarray(key),np.asarray(contact)],axis=-1))
    return atlas,lo,hi,z
