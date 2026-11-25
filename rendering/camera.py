"""Z-up model framing with orthographic and perspective projection."""
import numpy as np

def unit(x):return x/np.linalg.norm(x)

def camera(bounds,aspect,azimuth=35,elevation=45,projection='orthographic',zoom=1.,target=None,extent_override=None):
    center=np.mean(bounds,axis=0) if target is None else np.asarray(target,float)
    a,e=np.deg2rad([azimuth,elevation]);direction=np.array([np.cos(e)*np.cos(a),np.cos(e)*np.sin(a),np.sin(e)])
    right=unit(np.cross(-direction,[0,0,1.]));up=np.cross(right,-direction)
    from itertools import product
    corners=np.array(list(product(*zip(*bounds))))-center
    extent=max(np.max(np.abs(corners@up)),np.max(np.abs(corners@right))/aspect)*1.08/zoom
    if extent_override is not None:extent=float(extent_override)/zoom
    distance=max(np.linalg.norm(bounds[1]-bounds[0])*2,extent*3.4);eye=center+direction*distance
    view=np.eye(4);view[:3,:3]=[right,up,direction];view[:3,3]=-view[:3,:3]@eye
    near=max(.001,distance*.15);far=distance*2.5
    if projection=='orthographic':
        p=np.diag([1/(extent*aspect),1/extent,-2/(far-near),1.]);p[2,3]=-(far+near)/(far-near)
    else:
        f=distance/extent;p=np.zeros((4,4));p[0,0]=f/aspect;p[1,1]=f;p[2,2]=-(far+near)/(far-near);p[2,3]=-2*far*near/(far-near);p[3,2]=-1
    return p@view,eye
