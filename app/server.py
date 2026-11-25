"""Local-only browser control console; no CDN, external services or new packages."""
from pathlib import Path
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler
from urllib.parse import urlsplit, unquote
import argparse, json, sys, mimetypes, io, shutil
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'simulation'))
from step_28_flight import FlightConfig, simulate
from step_25_head_articulation import aim_target, head_rotations
from step_26_abdominal_articulation import curvature_pose,joint_rotations,NAMES
from step_27_leg_kinematics import LEGS, JOINTS,leg_geometry,inverse_kinematics
from scipy.spatial.transform import Rotation
from step_23_wing_kinematics import read_glb,write_glb
import tempfile

STAGES={str(i):f'step_{i:02d}_'+('chassis' if i==1 else 'assembled')+'.glb' for i in range(1,16)}
MODELS={**STAGES,'flight':'dragonfly_flight.glb','balanced':'dragonfly_static_balanced_iridescent.glb','performance':'dragonfly_static_performance.glb'}

def pose(request):
    kind=request['kind']; out={}
    if kind=='head':
        a=aim_target(np.array(request['target'],float));q=head_rotations(a)
        out=dict(zip(('rig_head_yaw','rig_head_pitch'),[v.tolist() for v in q]));return {'rotations':out,'angles':a.tolist()}
    if kind=='abdomen':
        a=curvature_pose(float(request['vertical']),float(request['lateral']));out=dict(zip(NAMES,joint_rotations(a).tolist()));return {'rotations':out,'angles':a.tolist()}
    if kind=='leg':
        leg=request['leg'];p=leg_geometry(leg)[1];delta=np.array(request['offset_mm'],float)
        if delta.shape!=(3,) or not np.isfinite(delta).all() or np.any(abs(delta)>10):raise ValueError('Offsets must be finite and within 10 mm')
        q,e,ok=inverse_kinematics(leg,p+delta*.001);rot=Rotation.from_rotvec(q[:,None]*leg_geometry(leg)[2]).as_quat()
        return {'rotations':dict(zip([f'rig_leg_{leg}_{j}' for j in JOINTS],rot.tolist())) if ok else {},'error_mm':e*1000,'reachable':ok}
    raise ValueError('Unknown pose kind')

def flight(request):
    duration=float(request.get('duration',4));mass=float(request.get('mass_g',35));freq=float(request.get('max_frequency',40))
    if not 1<=duration<=10 or not 5<=mass<=100 or not 10<=freq<=80:raise ValueError('Flight parameter outside application range')
    # Snap to integrator grid; sample telemetry at 60 Hz for the browser.
    duration=round(duration*480)/480;d=simulate(request.get('mode','hover'),duration=duration,config=FlightConfig(mass=mass/1000,max_frequency=freq));idx=np.unique(np.r_[np.arange(0,len(d['time']),8),len(d['time'])-1]);return {k:d[k][idx].tolist() for k in ('time','position','velocity','rotation','frequency','phase_cycles','saturated')}

def export_pose(request,destination):
    model=MODELS[request['model']];doc,b=read_glb(ROOT/'models'/model);nodes=doc['nodes'];overrides=request.get('transforms',{})
    if len(overrides)>200:raise ValueError('Too many overrides')
    for key,trs in overrides.items():
        idx=int(key)
        if not 0<=idx<len(nodes) or 'matrix' in nodes[idx] or not nodes[idx].get('name','').startswith('rig_'):raise ValueError('Invalid rig node')
        for name,val in trs.items():
            a=np.array(val,float);size=4 if name=='rotation' else 3
            if name not in ('rotation','translation','scale') or a.shape!=(size,) or not np.isfinite(a).all() or np.any(abs(a)>100):raise ValueError('Invalid transform')
            if name=='rotation':
                if abs(np.linalg.norm(a)-1)>1e-4:raise ValueError('Invalid quaternion')
            nodes[idx][name]=a.tolist()
    doc.pop('animations',None);write_glb(destination,doc,bytearray(b))

class Handler(BaseHTTPRequestHandler):
    def send_json(self,value,status=200):
        data=json.dumps(value,allow_nan=False).encode();self.send_response(status);self.send_header('Content-Type','application/json');self.send_header('Content-Length',str(len(data)));self.end_headers();self.wfile.write(data)
    def send_file(self,p,download=False):
        self.send_response(200);self.send_header('Content-Type', 'model/gltf-binary' if p.suffix=='.glb' else mimetypes.guess_type(p.name)[0] or 'application/octet-stream');self.send_header('Content-Length',str(p.stat().st_size))
        if download:self.send_header('Content-Disposition','attachment; filename="'+p.name+'"')
        self.end_headers();
        with p.open('rb') as f:shutil.copyfileobj(f,self.wfile)
    def do_GET(self):
        route=unquote(urlsplit(self.path).path)
        try:
            if route=='/api/models':return self.send_json(MODELS)
            if route=='/studio-preview.png':return self.send_file(ROOT/'renders/dragonfly_studio_graphite.png')
            if route.startswith('/model/'):
                p=ROOT/'models'/MODELS[route.removeprefix('/model/')];return self.send_file(p)
            if route in ('/','/index.html','/viewer.js','/ui.js','/style.css'):
                return self.send_file(ROOT/'app'/('index.html' if route=='/' else route[1:]))
            self.send_error(404)
        except (KeyError,FileNotFoundError):self.send_error(404)
        except (BrokenPipeError,ConnectionResetError):pass
    def do_POST(self):
        try:
            if self.headers.get('Host','').split(':')[0] not in ('127.0.0.1','localhost'):return self.send_json({'error':'Local host required'},403)
            origin=self.headers.get('Origin');expected=f'http://{self.headers.get("Host")}'
            if origin and origin!=expected: return self.send_json({'error':'Cross-origin request rejected'},403)
            if self.headers.get_content_type()!='application/json':return self.send_json({'error':'JSON required'},415)
            n=int(self.headers.get('Content-Length',0))
            if not 0<n<=100000:return self.send_json({'error':'Invalid request size'},400)
            data=json.loads(self.rfile.read(n))
            if self.path=='/api/pose':return self.send_json(pose(data))
            if self.path=='/api/flight':return self.send_json(flight(data))
            if self.path=='/api/render':
                sys.path.insert(0,str(ROOT/'rendering'))
                from step_22_studio import render
                with tempfile.TemporaryDirectory() as temp:
                    source=Path(temp)/'pose.glb';output=Path(temp)/'studio.png';export_pose(data,source)
                    az=float(data.get('azimuth',25));el=float(data.get('elevation',55));zoom=float(data.get('zoom',1));preset=data.get('preset','graphite')
                    if not np.isfinite([az,el,zoom]).all() or not -360<=az<=360 or not 1<=el<=89 or not .2<=zoom<=12 or preset not in ('graphite','gallery','cinematic'):raise ValueError('Invalid studio camera')
                    render(source,output,width=1200,height=800,azimuth=az,elevation=el,zoom=zoom,preset=preset,ground=False);return self.send_file(output,True)
            if self.path=='/api/export':
                with tempfile.TemporaryDirectory() as temp:
                    p=Path(temp)/'dragonfly_pose.glb';export_pose(data,p);return self.send_file(p,True)
            self.send_error(404)
        except (ValueError,KeyError,TypeError,IndexError) as e:self.send_json({'error':str(e)},400)
        except (BrokenPipeError,ConnectionResetError):pass
        except Exception as e:self.send_json({'error':str(e)},500)

def main():
    p=argparse.ArgumentParser();p.add_argument('--port',type=int,default=8765);p.add_argument('--no-browser',action='store_true');a=p.parse_args();server=ThreadingHTTPServer(('127.0.0.1',a.port),Handler);url=f'http://127.0.0.1:{server.server_port}';print('Dragonfly Studio:',url,flush=True)
    if not a.no_browser:
        import webbrowser;webbrowser.open(url)
    try:server.serve_forever()
    except KeyboardInterrupt:pass
    finally:server.server_close()
if __name__=='__main__':main()
