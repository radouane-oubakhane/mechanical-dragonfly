import numpy as np
from scipy.spatial.transform import Rotation
from generate_realistic import Actor, load_scenario, ROOT, camera_at
from physics_v2 import crash, Physics

def main():
    # Analytic free fall (drag disabled), before impact.
    pts=np.array([[-.01,-.01,-.01],[.01,.01,.01]])
    c=Physics(drag=0)
    f=crash(pts,[0,0,1],[0,0,0],Rotation.identity(),[0,0,0],.1,c)
    t=f['time'][-1]; assert abs(f['position'][-1,2]-(1-.5*c.gravity*t*t))<.0011
    assert abs(f['velocity'][-1,2]+c.gravity*t)<1e-9
    # Horizontal speed decays under friction in a ground-supported proxy.
    pts=np.array([[x,y,z] for x in (-.03,.03) for y in (-.02,.02) for z in (-.01,.01)])
    f=crash(pts,[0,0,.01],[.3,0,0],Rotation.identity(),[0,0,0],1.,c)
    assert abs(f['velocity'][-1,0])<.05 and f['gap'].min()>=-1e-8
    s=load_scenario(ROOT/'scenarios/last_flight.json');a=Actor(s);m=a.motion
    assert m.fall<m.impact<m.fall+.4; assert m.run['gap'].min()>=-1e-8
    assert m.run['sleep'][-1] and np.linalg.norm(m.run['velocity'][-1])==0
    assert Rotation.from_quat(m.run['rotation'][-1]).apply([0,0,1])[2]<-.9
    assert np.max(abs(np.linalg.norm(m.run['rotation'],axis=1)-1))<1e-8
    for e in m.e.values():
        x=camera_at(m,max(0,e-1e-6));y=camera_at(m,min(s['duration'],e+1e-6));assert np.linalg.norm(x[3]-y[3])<.001 and abs(x[2]-y[2])<.001
    for t in [0,6.5,7.5,8.5,14,17.9,18,18.18,18.25,19,20,24.9]:
        scene,_=a.pose(t);assert scene.triangles==504756;assert scene.bounds[0,2]>-.0005,(t,scene.bounds[0,2])
    print('PASS: gravity, friction, nonpenetration, final overturned pose, camera continuity, 12 rendered-geometry poses, original triangle count')
if __name__=='__main__':main()
