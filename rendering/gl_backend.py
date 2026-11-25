"""Minimal Linux EGL/OpenGL 3.3 offscreen backend (Mesa or hardware drivers)."""
import ctypes as C
import ctypes.util

I=C.c_int; U=C.c_uint; F=C.c_float; P=C.c_void_p; S=C.c_char_p; B=C.c_ubyte

def bind(lib,name,restype,*args):
    fn=getattr(lib,name); fn.restype=restype; fn.argtypes=list(args); return fn

class Context:
    def __init__(self,width,height):
        self.egl=C.CDLL(ctypes.util.find_library('EGL') or 'libEGL.so.1')
        self.gl=C.CDLL(ctypes.util.find_library('GL') or 'libGL.so.1')
        e=self.egl
        self.terminate=bind(e,'eglTerminate',U,P)
        self.destroy_surface=bind(e,'eglDestroySurface',U,P,P)
        self.destroy_context=bind(e,'eglDestroyContext',U,P,P)
        self.make_current=bind(e,'eglMakeCurrent',U,P,P,P,P)
        getproc=bind(e,'eglGetProcAddress',P,S)
        ptr=getproc(b'eglGetPlatformDisplayEXT')
        self.display=None
        if ptr:
            self.display=C.CFUNCTYPE(P,U,P,P)(ptr)(0x31DD,None,None) # surfaceless Mesa
        init=bind(e,'eglInitialize',U,P,P,P)
        a,b=I(),I()
        if not self.display or not init(self.display,C.byref(a),C.byref(b)):
            self.display=bind(e,'eglGetDisplay',P,P)(None)
            if not init(self.display,C.byref(a),C.byref(b)): raise RuntimeError('EGL initialization failed; install an EGL/OpenGL driver')
        if not bind(e,'eglBindAPI',U,U)(0x30A2): raise RuntimeError('OpenGL EGL API unavailable')
        attrs=(I*17)(0x3033,1,0x3040,8,0x3024,8,0x3023,8,0x3022,8,0x3021,8,0x3025,24,0x3038,0,0)
        config=P(); count=I()
        if not bind(e,'eglChooseConfig',U,P,P,P,I,P)(self.display,attrs,C.byref(config),1,C.byref(count)) or not count.value: raise RuntimeError('No EGL pbuffer config')
        self.surface=bind(e,'eglCreatePbufferSurface',P,P,P,P)(self.display,config,(I*5)(0x3057,width,0x3056,height,0x3038))
        self.context=bind(e,'eglCreateContext',P,P,P,P,P)(self.display,config,None,(I*7)(0x3098,3,0x30FB,3,0x30FD,1,0x3038))
        if not self.surface or not self.context or not self.make_current(self.display,self.surface,self.surface,self.context): raise RuntimeError('Cannot create OpenGL 3.3 context')
    def fn(self,name,restype,*args): return bind(self.gl,'gl'+name,restype,*args)
    def close(self):
        self.make_current(self.display,None,None,None)
        self.destroy_context(self.display,self.context);self.destroy_surface(self.display,self.surface);self.terminate(self.display)
    def __enter__(self):return self
    def __exit__(self,*args): self.close()
