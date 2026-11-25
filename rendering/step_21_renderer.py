"""Dragonfly offscreen OpenGL renderer; no Blender or window server required."""
from pathlib import Path
import argparse, ctypes as C, json, time
import numpy as np
from PIL import Image
from gl_backend import Context,I,U,F,P,S,B
from scene import Scene,ROOT
from camera import camera
SHADERS=Path(__file__).resolve().parent/'shaders'

def compile_program(ctx):
    create=ctx.fn('CreateShader',U,U); source=ctx.fn('ShaderSource',None,U,I,P,P);compile_=ctx.fn('CompileShader',None,U); get=ctx.fn('GetShaderiv',None,U,U,P); log=ctx.fn('GetShaderInfoLog',None,U,I,P,P)
    shaders=[]
    for kind,file in [(0x8B31,'pbr.vert'),(0x8B30,'pbr.frag')]:
        shader=create(kind);src=(S*1)((SHADERS/file).read_bytes());source(shader,1,src,None);compile_(shader);ok=I();get(shader,0x8B81,C.byref(ok))
        if not ok.value:
            buf=C.create_string_buffer(8192);log(shader,len(buf),None,buf);raise RuntimeError(buf.value.decode())
        shaders.append(shader)
    program=ctx.fn('CreateProgram',U)();attach=ctx.fn('AttachShader',None,U,U)
    for shader in shaders:attach(program,shader)
    ctx.fn('LinkProgram',None,U)(program);ok=I();ctx.fn('GetProgramiv',None,U,U,P)(program,0x8B82,C.byref(ok))
    if not ok.value:
        buf=C.create_string_buffer(8192);ctx.fn('GetProgramInfoLog',None,U,I,P,P)(program,len(buf),None,buf);raise RuntimeError(buf.value.decode())
    for shader in shaders:ctx.fn('DeleteShader',None,U)(shader)
    return program

def render(source,output,width=1600,height=1000,azimuth=12,elevation=65,projection='orthographic',zoom=1.,supersample=1,exposure=1.,target=None):
    if Path(source).resolve()==Path(output).resolve():raise ValueError('Output must not overwrite source')
    if min(width,height)<16 or max(width,height)*supersample>8192 or supersample not in (1,2,3):raise ValueError('Invalid image dimensions/supersampling')
    if not (0<abs(elevation)<90 or elevation==0) or zoom<=0 or exposure<=0:raise ValueError('Invalid camera parameters')
    start=time.perf_counter();scene=Scene(source);vp,eye=camera(scene.bounds,width/height,azimuth,elevation,projection,zoom,target);w,h=width*supersample,height*supersample
    with Context(w,h) as ctx:
        fn=ctx.fn;getstr=fn('GetString',S,U);renderer=getstr(0x1F01).decode();version=getstr(0x1F02).decode()
        program=compile_program(ctx);fn('UseProgram',None,U)(program)
        loc=fn('GetUniformLocation',I,U,S);locations={}
        def L(name):
            if name not in locations:locations[name]=loc(program,name.encode())
            return locations[name]
        u1i=fn('Uniform1i',None,I,I);u1f=fn('Uniform1f',None,I,F);u3=fn('Uniform3f',None,I,F,F,F);u4=fn('Uniform4f',None,I,F,F,F,F)
        def f4(n,x):u4(L(n),*map(float,x))
        def f3(n,x):u3(L(n),*map(float,x))
        mat=np.ascontiguousarray(vp.T,dtype='f4');fn('UniformMatrix4fv',None,I,I,B,P)(L('vp'),1,0,mat.ctypes.data);f3('eye',eye);u1f(L('exposure'),exposure)
        vao=U();vbo=U();fn('GenVertexArrays',None,I,P)(1,C.byref(vao));fn('BindVertexArray',None,U)(vao);fn('GenBuffers',None,I,P)(1,C.byref(vbo));fn('BindBuffer',None,U,U)(0x8892,vbo);fn('BufferData',None,U,C.c_ssize_t,P,U)(0x8892,scene.vertices.nbytes,scene.vertices.ctypes.data,0x88E4)
        for index,size,offset in [(0,3,0),(1,3,12),(2,2,24),(3,4,32)]:
            fn('EnableVertexAttribArray',None,U)(index);fn('VertexAttribPointer',None,U,I,U,B,I,P)(index,size,0x1406,0,48,P(offset))
        texids=(U*(len(scene.images)+1))();fn('GenTextures',None,I,P)(len(texids),texids);bindtex=fn('BindTexture',None,U,U);active=fn('ActiveTexture',None,U);param=fn('TexParameteri',None,U,U,I);upload=fn('TexImage2D',None,U,I,I,I,I,I,U,U,P);mip=fn('GenerateMipmap',None,U)
        for id_,image in zip(texids,[*scene.images,np.full((1,1,4),255,dtype=np.uint8)]):
            bindtex(0x0DE1,id_);ih,iw=image.shape[:2];upload(0x0DE1,0,0x8058,iw,ih,0,0x1908,0x1401,image.ctypes.data);mip(0x0DE1)
            for key,val in [(0x2801,0x2703),(0x2800,0x2601),(0x2802,0x2901),(0x2803,0x2901)]:param(0x0DE1,key,val)
        slots=['base','mr','normal','thickness','aniso'];
        for i,s in enumerate(slots):u1i(L(s+'_tex'),i)
        def material(mid,transparent):
            m=scene.materials[mid];p=m.get('pbrMetallicRoughness',{});e=m.get('extensions',{});ir=e.get('KHR_materials_iridescence',{});an=e.get('KHR_materials_anisotropy',{});cc=e.get('KHR_materials_clearcoat',{})
            f4('base',p.get('baseColorFactor',[1,1,1,1]));f4('surface',[p.get('metallicFactor',1),p.get('roughnessFactor',1),m.get('normalTexture',{}).get('scale',1),e.get('KHR_materials_ior',{}).get('ior',1.5)])
            f4('optical',[e.get('KHR_materials_transmission',{}).get('transmissionFactor',0),ir.get('iridescenceFactor',0),ir.get('iridescenceIor',1.3),cc.get('clearcoatFactor',0)])
            f4('coating',[ir.get('iridescenceThicknessMinimum',100),ir.get('iridescenceThicknessMaximum',400),an.get('anisotropyStrength',0),cc.get('clearcoatRoughnessFactor',.2)])
            f3('emissive',m.get('emissiveFactor',[0,0,0]));u1i(L('is_transparent'),int(transparent))
            for i,(slot,tex) in enumerate(zip(slots,[p.get('baseColorTexture'),p.get('metallicRoughnessTexture'),m.get('normalTexture'),ir.get('iridescenceThicknessTexture'),an.get('anisotropyTexture')])):
                active(0x84C0+i);bindtex(0x0DE1,texids[tex['index']] if tex else texids[-1]);u1i(L('has_'+slot),int(tex is not None))
        fn('Viewport',None,I,I,I,I)(0,0,w,h);fn('ClearColor',None,F,F,F,F)(.035,.045,.06,1);fn('Clear',None,U)(0x4000|0x0100);enable=fn('Enable',None,U);disable=fn('Disable',None,U);depthmask=fn('DepthMask',None,B);enable(0x0B71);draw=fn('DrawArrays',None,U,I,I)
        opaque=sorted((d for d in scene.draws if not scene.transparent(d)),key=lambda d:d.material);trans=sorted((d for d in scene.draws if scene.transparent(d)),key=lambda d:float(np.dot(d.center-eye,d.center-eye)),reverse=True)
        for transparent,items in [(False,opaque),(True,trans)]:
            if transparent:enable(0x0BE2);fn('BlendFunc',None,U,U)(0x0302,0x0303);depthmask(0);enable(0x0B44);fn('CullFace',None,U)(0x0405)
            previous=None
            for d in items:
                if d.material!=previous:material(d.material,transparent);previous=d.material
                draw(4,d.start,d.count)
        depthmask(1);disable(0x0BE2);disable(0x0B44);fn('Finish',None)();img=np.empty((h,w,4),dtype=np.uint8);fn('ReadPixels',None,I,I,I,I,U,U,P)(0,0,w,h,0x1908,0x1401,img.ctypes.data)
        err=fn('GetError',U)();
        if err:raise RuntimeError(f'OpenGL error 0x{err:x}')
    im=Image.fromarray(img[::-1,:,:3].copy());
    if supersample>1:im=im.resize((width,height),Image.Resampling.LANCZOS)
    output=Path(output);output.parent.mkdir(parents=True,exist_ok=True);im.save(output)
    return dict(renderer=renderer,opengl=version,triangles=scene.triangles,opaque_draws=len(opaque),transparent_draws=len(trans),textures=len(scene.images),size=[width,height],supersample=supersample,seconds=round(time.perf_counter()-start,3),output=output.name)

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--source',type=Path,default=ROOT/'models/dragonfly_static_balanced_iridescent.glb');p.add_argument('--output',type=Path,default=ROOT/'renders/dragonfly_stage21.png');p.add_argument('--width',type=int,default=1600);p.add_argument('--height',type=int,default=1000);p.add_argument('--azimuth',type=float,default=12);p.add_argument('--elevation',type=float,default=65);p.add_argument('--projection',choices=['orthographic','perspective'],default='orthographic');p.add_argument('--zoom',type=float,default=1.);p.add_argument('--supersample',type=int,default=1);p.add_argument('--exposure',type=float,default=1.);p.add_argument('--target',type=float,nargs=3,help='Camera target in meters');a=p.parse_args();print(json.dumps(render(**vars(a)),indent=2))
