"""Stage 04: four dorsal spur-gear channels, with modular actuator sockets.
Coordinates are meters; dimensions in the construction code are millimeters.
This is visual concept geometry, not a load-rated or powered transmission.
"""
from pathlib import Path
import numpy as np
import trimesh
from step_01_chassis import Module, create_chassis
from step_02_power_core import attach_module
from step_03_electronics import build_stage_03


def gear_outline(teeth=12, module_mm=.5, phase=0.):
    """Sample a 20-degree involute-inspired external tooth profile in mm.

    Includes simplified root transitions and deliberate angular backlash.
    The profile is not a manufacturing drawing (no generated root fillet).
    """
    pitch = 2*np.pi/teeth
    rp = teeth*module_mm/2
    rb = rp*np.cos(np.deg2rad(20))
    root, tip = rp-1.25*module_mm, rp+module_mm
    def inv(a): return np.tan(a)-a
    radii = np.linspace(rb, tip, 9)
    half = pitch/4-.025+inv(np.deg2rad(20))-inv(np.arccos(rb/radii))
    # Each tooth begins at its root valley and ends before the next valley.
    points = []
    for i in range(teeth):
        center = i*pitch+phase
        polar = [(-pitch/2,root),(-half[0]-.018,root)]
        polar += list(zip(-half,radii))
        polar += list(zip(half[::-1],radii[::-1]))
        polar += [(half[0]+.018,root)]
        points.extend((r*np.cos(center+a),r*np.sin(center+a)) for a,r in polar)
    return np.array(points)


def create_spur_gear(scale=1., phase=0.):
    """Return a closed, bored, 12-tooth spur gear mesh, centered at origin.

    Axis +Z, 0.5 mm nominal module, 1.1 mm face width, 1.4 mm bore.
    """
    if not np.isfinite(scale) or scale<=0: raise ValueError('invalid scale')
    outline = gear_outline(phase=phase)
    bore = .70*outline/np.linalg.norm(outline,axis=1)[:,None]
    n = len(outline)
    vertices = np.vstack([np.column_stack([p,np.full(n,z)])
                          for p,z in [(outline,-.55),(outline,.55),(bore,.55),(bore,-.55)]])
    faces=[]
    for k in range(4):
        for i in range(n):
            j=(i+1)%n
            a,b=k*n+i,k*n+j
            c,d=((k+1)%4)*n+j,((k+1)%4)*n+i
            faces.extend([(a,b,c),(a,c,d)])
    mesh=trimesh.Trimesh(vertices=vertices,faces=faces,process=True)
    mesh.fix_normals()
    mesh.apply_scale(.001*scale)
    return mesh


def create_transmission(scale: float=1.0) -> Module:
    """Return a dorsal transmission with four independently modeled channels.

    Mount at chassis 'transmission'. Sockets actuator_{fore|hind}_{left|right}
    point outward along local +X; shaft_* frames point up along local +Z.
    """
    if not np.isfinite(scale) or scale<=0: raise ValueError('scale must be positive and finite')
    mm=.001*scale
    scene=trimesh.Scene()
    anchors={'mount':np.eye(4)}
    def mat(name,color,metal,rough):
        return trimesh.visual.material.PBRMaterial(name=name,baseColorFactor=color,
                metallicFactor=metal,roughnessFactor=rough)
    ti=mat('Transmission brushed titanium appearance',[.57,.60,.64,1],.93,.35)
    steel=mat('Hardened gear steel',[.22,.25,.29,1],.95,.27)
    polished=mat('Polished bearing steel',[.72,.75,.79,1],.97,.18)
    bronze=mat('Bronze bushings',[.64,.38,.17,1],.84,.32)
    def add(name,mesh,material):
        mesh.visual=trimesh.visual.TextureVisuals(material=material)
        scene.add_geometry(mesh,node_name=name,geom_name=name)
    def box(name,dims,pos,material):
        m=trimesh.creation.box(extents=np.array(dims)*mm)
        m.apply_translation(np.array(pos)*mm); add(name,m,material)
    def cyl(name,r,h,pos,material,sections=40):
        m=trimesh.creation.cylinder(radius=r*mm,height=h*mm,sections=sections)
        m.apply_translation(np.array(pos)*mm); add(name,m,material)
    def ring(name,ri,ro,h,pos,material):
        m=trimesh.creation.annulus(r_min=ri,r_max=ro,height=h,sections=48)
        m.apply_scale(mm)
        m.apply_translation(np.array(pos)*mm); add(name,m,material)
    def sphere(name,r,pos,material):
        m=trimesh.creation.icosphere(subdivisions=1,radius=r*mm)
        m.apply_translation(np.array(pos)*mm); add(name,m,material)
    # Open ladder housing, supported by three narrow feet on the chassis rings.
    for side,y in [('left',5),('right',-5)]:
        box('housing_rail_'+side,[43,5,1],[0,y,.5],ti)
    for label,x in [('fore',20),('middle',0),('hind',-20)]:
        box('housing_crossbar_'+label,[3,19,1],[x,0,.5],ti)
        box('chassis_foot_'+label,[2.4,1.6,.4],[x,0,-.15],ti)
        # Small surface-mounted fastener heads.
        cyl('chassis_fastener_'+label,.8,.35,[x,0,1.175],polished,6)
    # Each channel has a separate driver/output gear pair on vertical shafts.
    for pair,signx in [('fore',1),('hind',-1)]:
        for side,signy in [('left',1),('right',-1)]:
            name=pair+'_'+side
            y=5*signy
            for role,x,phase in [('driver',7*signx,0.),('output',13*signx,np.pi/12)]:
                key=name+'_'+role
                # Bearing outer race rests on rail; inner race contacts shaft.
                ring(key+'_outer_race',1.10,1.45,.8,[x,y,1.4],steel)
                ring(key+'_inner_race',.64,.83,.8,[x,y,1.4],polished)
                for i in range(8):
                    theta=i*2*np.pi/8
                    sphere(key+f'_ball_{i}',.125,[x+.965*np.cos(theta),y+.965*np.sin(theta),1.4],polished)
                cyl(key+'_shaft',.64,3.6,[x,y,2.8],polished)
                ring(key+'_gear_hub',.64,.95,.35,[x,y,2.025],bronze)
                # Gear with real open bore, separate shaft, and retaining collar.
                gear=create_spur_gear(scale,phase)
                gear.apply_translation(np.array([x,y,2.75])*mm)
                add(key+'_gear',gear,steel if role=='driver' else ti)
                ring(key+'_retainer',.64,1.0,.35,[x,y,3.475],polished)
                if role=='output':
                    # A keyed-looking hexagonal coupling at the top of the shaft.
                    cyl(key+'_coupling',.95,.75,[x,y,4.475],bronze,6)
                frame=np.eye(4); frame[:3,3]=np.array([x,y,4.85 if role=='output' else 4.6])*mm
                anchors[f'{role}_shaft_{name}']=frame
            # Physical mounting ear bridges the rail to the outboard socket.
            x=13*signx
            box(name+'_mount_bridge',[4,2.5,1],[x,8.75*signy,.5],ti)
            box(name+'_actuator_plate',[4,1,3.5],[x,10*signy,2.25],ti)
            # Side-facing hex fasteners, above/below the attachment center.
            for i,dx in enumerate([-1.3,1.3]):
                m=trimesh.creation.cylinder(radius=.42*mm,height=.35*mm,sections=6)
                m.apply_transform(trimesh.transformations.rotation_matrix(-signy*np.pi/2,[1,0,0]))
                m.apply_translation(np.array([x+dx,10.675*signy,2.5])*mm)
                add(name+f'_plate_bolt_{i}',m,polished)
            frame=trimesh.transformations.rotation_matrix(signy*np.pi/2,[0,0,1])
            frame[:3,3]=np.array([x,10.5*signy,2.5])*mm
            anchors['actuator_'+name]=frame
    return Module('transmission',scene,anchors)


def build_stage_04(scale: float=1.0) -> tuple[Module,trimesh.Scene]:
    """Return the new module and cumulative scene, preserving stages 01–03."""
    _,_,prior=build_stage_03(scale)
    chassis=create_chassis(scale)
    transmission=create_transmission(scale)
    assembled=attach_module(Module('stage_03',prior,chassis.anchors),'transmission',transmission)
    return transmission,assembled


if __name__=='__main__':
    out=Path(__file__).resolve().parent
    transmission,assembled=build_stage_04()
    transmission.scene.export(str(out/'step_04_transmission.glb'))
    assembled.export(str(out/'step_04_assembled.glb'))
    print('Transmission meshes:',len(transmission.scene.geometry))
    print('Transmission anchors:',len(transmission.anchors))
    print('Assembly meshes:',len(assembled.geometry))
    print('Assembly dimensions (mm):',np.round(assembled.extents*1000,3))
