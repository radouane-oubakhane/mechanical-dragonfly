from generate import *

def test():
    s=load_scenario(ROOT/'scenarios/last_flight.json')
    assert state(s,0)['height']==0
    assert state(s,12)['height']==s['flight_height_m']
    assert state(s,19)['height']<state(s,18)['height']
    assert state(s,20)['height']==0
    assert state(s,24.9)['power']==0
    actor=Actor()
    for t in (0,12,19,24.9):
        scene,st=actor.pose(s,t)
        assert scene.triangles==504756
        assert abs(scene.bounds[0,2]-st['height']-.0005)<1e-6
    print('Scenario pose tests passed')
if __name__=='__main__':test()
