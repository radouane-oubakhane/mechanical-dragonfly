"""Final removable thorax armor; nominal geometry in mm, output in meters."""
from pathlib import Path
import numpy as np
import trimesh
from step_01_chassis import Module, create_chassis
from step_11_abdominal_armor import shell_panel, _tube
from step_14_wing_membranes import build_stage_14


def create_thorax_armor(scale: float = 1.0) -> Module:
    """Open-sided titanium shell with inspection glazing and ventilation.

    The roof stands above the transmission; cutaway sides leave wing drives
    and legs exposed. Fasteners and stays intentionally engage their mounts.
    """
    if not np.isfinite(scale) or scale <= 0:
        raise ValueError('scale must be positive and finite')
    mm=.001*scale; scene=trimesh.Scene()
    def material(name, rgb, metal, rough, alpha=1):
        return trimesh.visual.material.PBRMaterial(name=name,baseColorFactor=[*rgb,alpha],metallicFactor=metal,roughnessFactor=rough,alphaMode='BLEND' if alpha<1 else 'OPAQUE')
    ti=material('Thorax satin titanium',[.53,.57,.61],.92,.32)
    dark=material('Thorax carbon composite',[.045,.055,.065],.06,.43)
    steel=material('Thorax polished steel',[.78,.80,.82],.96,.21)
    glass=material('Thorax inspection glazing',[.55,.78,.83],0,.17,.24)
    cyan=material('Thorax identification cyan',[.06,.58,.70],.25,.30)
    def add(name,m,mat):
        m.apply_scale(mm); m.visual=trimesh.visual.TextureVisuals(material=mat)
        scene.add_geometry(m,node_name=name,geom_name=name)
    def box(name,size,center,mat):
        m=trimesh.creation.box(extents=size);m.apply_translation(center);add(name,m,mat)
    def tube(name,a,b,r,mat): add(name,_tube(a,b,r),mat)
    # Central roof: an actual open window surrounded by separate metal rails.
    # All roof parts sit above the highest central transmission component.
    for s in (-1,1):
        box(f'roof_longitudinal_{s}',[46,2.0,.65],[0,s*6.3,19.4],ti)
        box(f'roof_carbon_inlay_{s}',[35,.65,.22],[0,s*6.3,19.84],dark)
        for x in (-21,-10,10,21):
            tube(f'roof_fastener_{s}_{x}',[x,s*6.3,19.72],[x,s*6.3,20.02],.32,steel)
    for s in (-1,1):
        # end cap made of separate bars, leaving six through-slots per end
        for j in range(7):
            box(f'roof_vent_bar_{s}_{j}',[.82,10.6,.65],[s*(10.6+j*1.85),0,19.4],ti)
        box(f'roof_end_beam_{s}',[1.3,14.6,.75],[s*23,0,19.4],ti)
    box('inspection_window',[19.8,10.5,.40],[0,0,19.37],glass)
    for s in (-1,1):
        box(f'window_retainer_{s}',[20.2,.45,.3],[0,s*5.18,19.65],dark)
        box(f'window_end_retainer_{s}',[.45,10.6,.3],[s*9.85,0,19.65],dark)
    # Three paired side panels leave open gaps around actuator channels.
    for sign,side in ((1,'left'),(-1,'right')):
        angle=0 if sign==1 else np.pi
        for i,(x,length) in enumerate(((-20,7),(0,16),(20,7))):
            # Upper lateral sectors; mirrored on the right.
            a,b=(-.08,.58) if sign==1 else (np.pi-.58,np.pi+.08)
            m=shell_panel(length,15.0,15.0,a,b,t0=0,t1=1,clearance=0,thickness=.55,vents=True)
            m.apply_translation([x,0,0]);add(f'side_{side}_{i}_vented',m,ti if i!=1 else dark)
            for dx in (-length*.36,length*.36):
                xx=x+dx; q=np.array([xx,sign*15.2,1.4]);
                tube(f'side_{side}_{i}_stud_{dx}',[xx,sign*14.0,1.4],q,.23,steel)
                tube(f'side_{side}_{i}_cap_{dx}',q,[xx,sign*15.8,1.4],.34,steel)
            # The three cross frames provide real side attachment surfaces.
            tube(f'side_{side}_{i}_frame_stay',[x,sign*13.1,1.6],[x,sign*15.2,1.6],.48,ti)
        # External roof support struts avoid the actuator channels at x=±13.
        for x in (-20,20):
            tube(f'roof_{side}_stay_{x}',[x,sign*13.1,5.2],[x,sign*12.7,11.6],.55,ti)
            tube(f'roof_{side}_arch_{x}',[x,sign*12.7,11.6],[x,sign*6.3,19.4],.62,ti)
        for i in range(3):
            box(f'side_index_{side}_{i}',[.75,.15,.7],[-2+i*2,sign*15.58,2.8],cyan)
    # Shallow ventral keel: protects battery/spine without covering hips.
    for x in (-18,0,18):
        box(f'keel_panel_{x}',[15,5,.55],[x,0,-12.6],dark)
        tube(f'keel_standoff_{x}',[x,0,-10.5],[x,0,-12.6],.43,ti)
        for y in (-1.7,1.7):
            tube(f'keel_fastener_{x}_{y}',[x,y,-12.8],[x,y,-13.05],.28,steel)
    # Raised industrial identity marker, not an engraved serial number.
    for i,(w,l) in enumerate(((2.4,.32),(1.3,.32),(2.4,.32))):
        box(f'dorsal_identity_{i}',[w,l,.12],[0,6.3+(i-1)*.48,20.0],cyan)
    anchors={'mount':np.eye(4)}
    for n,p in [('inspection',(0,0,19.57)),('service_left',(0,15.55,1.4)),('service_right',(0,-15.55,1.4))]:
        f=np.eye(4);f[:3,3]=np.array(p)*mm;anchors[n]=f
    return Module('thorax_armor',scene,anchors)


def add_thorax_armor(scene, scale=1.0):
    """Append to an owned scene, preserving all existing geometry and transforms."""
    armor=create_thorax_armor(scale)
    placement=create_chassis(scale).anchors['thorax_armor'] @ np.linalg.inv(armor.anchors['mount'])
    for node in armor.scene.graph.nodes_geometry:
        local,g=armor.scene.graph[node]; name=f'{armor.name}/{node}'
        if name in scene.graph.nodes: raise ValueError(f'duplicate node {name}')
        scene.add_geometry(armor.scene.geometry[g].copy(),node_name=name,geom_name=name,transform=placement@local)
    return armor,scene


def build_stage_15(scale: float=1.0) -> tuple[Module,trimesh.Scene]:
    """Build all fifteen construction stages from Python source."""
    _,scene=build_stage_14(scale)
    return add_thorax_armor(scene,scale)


if __name__=='__main__':
    out=Path(__file__).resolve().parents[1]/'models';out.mkdir(exist_ok=True)
    armor,scene=build_stage_15();armor.scene.export(str(out/'step_15_thorax_armor.glb'))
    scene.export(str(out/'step_15_assembled.glb'))
    print(len(armor.scene.geometry),len(scene.geometry),scene.extents*1000)
