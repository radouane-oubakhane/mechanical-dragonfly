"""Stage 11: removable tapered armor panels and a terminal tail assembly.
All construction dimensions are nominal millimeters; exported meshes use meters.
"""
from pathlib import Path
import numpy as np
import trimesh
from step_01_chassis import Module, create_chassis
from step_10_abdominal_spine import segment_dimensions, build_stage_10, PITCHES


def _material(name, rgb, metal, rough):
    return trimesh.visual.material.PBRMaterial(name=name, baseColorFactor=[*rgb,1], metallicFactor=metal, roughnessFactor=rough)


def _palette():
    return {
        'ti': _material('Armor satin titanium', [.52,.56,.60], .94,.31),
        'carbon': _material('Armor carbon composite', [.035,.045,.055], .06,.45),
        'steel': _material('Armor polished fasteners', [.76,.79,.83], .97,.21),
        'bronze': _material('Tail bronze bearing', [.58,.34,.15], .85,.33),
        'cyan': _material('Armor cyan identification', [.08,.48,.60], .25,.37),
    }


def _add(scene, name, mesh, material, mm):
    mesh.apply_scale(mm)
    mesh.visual=trimesh.visual.TextureVisuals(material=material)
    scene.add_geometry(mesh, node_name=name, geom_name=name)


def _tube(a,b,r,inner=None,sections=20):
    a,b=np.asarray(a,float),np.asarray(b,float); d=b-a
    kw=dict(height=np.linalg.norm(d),sections=sections)
    m=trimesh.creation.cylinder(radius=r,**kw) if inner is None else trimesh.creation.annulus(r_min=inner,r_max=r,**kw)
    m.apply_transform(trimesh.geometry.align_vectors([0,0,1],d)); m.apply_translation((a+b)/2)
    return m


def _frame(p,mm,axis=None):
    f=np.eye(4); f[:3,3]=np.asarray(p)*mm
    if axis is not None: f[:3,:3]=trimesh.geometry.align_vectors([0,0,1],axis)[:3,:3]
    return f


def _check_scale(scale):
    if not np.isfinite(scale) or scale<=0: raise ValueError('scale must be positive and finite')


def shell_panel(length, r0, r1, theta0, theta1, t0=.16, t1=.84, clearance=.35, thickness=.42, vents=False):
    """Closed tapered annular sector with optional real through-slots.
    Local origin is the vertebra midpoint. The inner surface follows its taper.
    Grid cells omitted from both surfaces become holes; boundary walls close them.
    """
    ts=np.linspace(t0,t1,13); angles=np.linspace(theta0,theta1,17)
    nx,na=len(ts),len(angles); vertices=[]; faces=[]; edges={}
    for layer in (0,1):
        for t in ts:
            r=r0+(r1-r0)*t+clearance+layer*thickness
            for angle in angles:
                vertices.append([length*(.5-t),r*np.cos(angle),r*np.sin(angle)])
    n=nx*na
    for i in range(nx-1):
        for j in range(na-1):
            # Three transverse slots through the center of the panel.
            if vents and i in (3,5,7) and 5<=j<=10: continue
            a=i*na+j; b=(i+1)*na+j; c=b+1; d=a+1
            faces.extend([(a,b,c),(a,c,d),(a+n,c+n,b+n),(a+n,d+n,c+n)])
            for u,v in ((a,b),(b,c),(c,d),(d,a)):
                key=tuple(sorted((u,v)))
                if key in edges: del edges[key]
                else: edges[key]=(u,v)
    for u,v in edges.values():
        faces.extend([(v,u,u+n),(v,u+n,v+n)])
    m=trimesh.Trimesh(vertices=vertices,faces=faces,process=True)
    m.remove_unreferenced_vertices(); m.fix_normals()
    return m


def create_abdominal_armor(index: int, scale: float=1.0) -> Module:
    """Four removable armor sectors, recessed seams, fasteners and ID markers."""
    L,r0,r1=segment_dimensions(index); _check_scale(scale); mm=.001*scale
    scene=trimesh.Scene(); materials=_palette(); anchors={'mount':_frame([0,0,0],mm)}
    for label,angle,material in [('left',0,'carbon'),('dorsal',np.pi/2,'ti'),('right',np.pi,'carbon'),('ventral',3*np.pi/2,'ti')]:
        half=np.pi/4-.045
        m=shell_panel(L,r0,r1,angle-half,angle+half,vents=label in ('left','right'))
        _add(scene,'panel_'+label,m,materials[material],mm)
        for k,t in enumerate((.24,.72)):
            r=r0+(r1-r0)*t
            for s in (-1,1):
                a=angle+s*.49; direction=np.array([0,np.cos(a),np.sin(a)])
                center=np.array([L*(.5-t),0,0])
                # Fastener shaft bridges the outer shell to the underlying frame.
                _add(scene,f'stud_{label}_{k}_{s}',_tube(center+(r-.13)*direction,center+(r+.79)*direction,.10,sections=12),materials['steel'],mm)
                _add(scene,f'cap_{label}_{k}_{s}',_tube(center+(r+.77)*direction,center+(r+.91)*direction,.19,sections=16),materials['steel'],mm)
        anchors['panel_'+label]=_frame([0,(.5*(r0+r1)+.77)*np.cos(angle),(.5*(r0+r1)+.77)*np.sin(angle)],mm,[0,np.cos(angle),np.sin(angle)])
    # Ten-segment index as two raised seven-segment digits, not engraved text.
    digits={'0':'abcdef','1':'bc','2':'abdeg','3':'abcdg','4':'bcfg','5':'acdfg','6':'acdefg','7':'abc','8':'abcdefg','9':'abcdfg'}
    bars={'a':(0,.40,.46,.10),'g':(0,0,.46,.10),'d':(0,-.40,.46,.10), 'b':(.27,.20,.10,.32),'c':(.27,-.20,.10,.32),'e':(-.27,-.20,.10,.32),'f':(-.27,.20,.10,.32)}
    for k,digit in enumerate(f'{index:02d}'):
        for letter in digits[digit]:
            bx,by,w,h=bars[letter]; x=bx+(k-.5)*.76
            t=.5-x/L; r=r0+(r1-r0)*t
            m=trimesh.creation.box(extents=[w,h,.10]); m.apply_translation([x,by,r+.79])
            _add(scene,f'id_{k}_{letter}',m,materials['cyan'],mm)
    return Module(f'abdominal_armor_{index:02d}',scene,anchors)


def create_tail(scale: float=1.0) -> Module:
    """Terminal flange, paired cerci, pivot hardware and sensor tip."""
    _check_scale(scale); mm=.001*scale; scene=trimesh.Scene(); p=_palette()
    def tube(name,a,b,r,mat='ti',inner=None): _add(scene,name,_tube(a,b,r,inner),p[mat],mm)
    tube('mounting_flange',[0,0,0],[-.9,0,0],1.68,inner=.92)
    tube('terminal_housing',[-.9,0,0],[-2.5,0,0],1.30,inner=.63)
    tube('sensor_core',[-1.2,0,0],[-2.9,0,0],.54,'carbon')
    tube('sensor_tip',[-2.9,0,0],[-3.18,0,0],.45,'cyan')
    anchors={'mount':_frame([0,0,0],mm),'sensor':_frame([-3.18,0,0],mm,[-1,0,0])}
    for s,side in ((1,'left'),(-1,'right')):
        q=np.array([-1.85,s*1.10,0]); axis=np.array([0,0,1.])
        tube('pivot_'+side,q-.47*axis,q+.47*axis,.24,'steel')
        tube('bearing_'+side,q-.32*axis,q+.32*axis,.47,'bronze',inner=.25)
        a=q+np.array([-.24,s*.14,0]); b=np.array([-3.65,s*1.42,.06]); c=np.array([-5.05,s*1.30,.20])
        tube('cerci_base_'+side,a,b,.41)
        tube('cerci_tip_'+side,b,c,.26,'carbon')
        m=trimesh.creation.icosphere(subdivisions=2,radius=.26); m.apply_translation(c); _add(scene,'cerci_end_'+side,m,p['ti'],mm)
        anchors['pivot_'+side]=_frame(q,mm,axis)
    return Module('tail',scene,anchors)


def _insert(scene,module,placement):
    for node in module.scene.graph.nodes_geometry:
        local,g=module.scene.graph[node]; name=f'{module.name}/{node}'
        scene.add_geometry(module.scene.geometry[g].copy(),node_name=name,geom_name=name,transform=placement@local)


def add_armor(scene,segments,origin,scale=1.0):
    """Add armor to an owned scene using the actual vertebral anchor chain."""
    modules={}; transform=origin.copy()
    for index in range(1,11):
        segment=segments[f'abdominal_segment_{index:02d}']; placement=transform@np.linalg.inv(segment.anchors['mount'])
        armor=create_abdominal_armor(index,scale)
        _insert(scene,armor,placement@segment.anchors['armor']@np.linalg.inv(armor.anchors['mount']))
        modules[armor.name]=armor; transform=placement@segment.anchors['next']
    tail=create_tail(scale); _insert(scene,tail,transform@np.linalg.inv(tail.anchors['mount'])); modules[tail.name]=tail
    return modules


def build_stage_11(scale: float=1.0) -> tuple[dict[str,Module],trimesh.Scene]:
    segments,scene=build_stage_10(scale)
    return add_armor(scene,segments,create_chassis(scale).anchors['abdomen'],scale),scene


if __name__=='__main__':
    out=Path(__file__).resolve().parents[1]/'models'; out.mkdir(exist_ok=True)
    modules,scene=build_stage_11(); modules['abdominal_armor_01'].scene.export(str(out/'step_11_abdominal_armor.glb'))
    modules['tail'].scene.export(str(out/'step_11_tail.glb')); scene.export(str(out/'step_11_assembled.glb'))
    print('New meshes:',sum(len(m.scene.geometry) for m in modules.values()))
    print('Assembly meshes:',len(scene.geometry),'dimensions (mm):',scene.extents*1000)
