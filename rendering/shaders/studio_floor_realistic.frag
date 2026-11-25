#version 330 core
in vec2 ndc;out vec4 color;
uniform mat4 inv_vp;
uniform sampler2D shadow_tex;
uniform vec2 shadow_lo,shadow_hi;
uniform vec3 floor_color,background;
uniform float floor_z,strength,exposure;
uniform int ground_enabled;
vec3 display_rgb(vec3 c){c=max(c,vec3(0));c=clamp((c*(2.51*c+.03))/(c*(2.43*c+.59)+.14),0.,1.);return mix(12.92*c,1.055*pow(c,vec3(1./2.4))-.055,step(vec3(.0031308),c));}
float hash2(vec2 p){return fract(sin(dot(p,vec2(127.1,311.7)))*43758.5453);}
float grain(vec2 p){vec2 i=floor(p),f=fract(p);f=f*f*(3.-2.*f);return mix(mix(hash2(i),hash2(i+vec2(1,0)),f.x),mix(hash2(i+vec2(0,1)),hash2(i+vec2(1,1)),f.x),f.y);}
void main(){
 vec4 a=inv_vp*vec4(ndc,-1,1),b=inv_vp*vec4(ndc,1,1);vec3 p=a.xyz/a.w,q=b.xyz/b.w;vec3 ray=q-p;
 float t=(floor_z-p.z)/ray.z;vec3 hit=p+t*ray;
 float vignette=1.-.25*clamp(dot(ndc,ndc)*.5,0.,1.);
 vec3 bg=background*(.65+.45*(ndc.y*.5+.5))*vignette;
 if(ground_enabled==1 && t>=0. && ray.z<-.000001){
  vec2 uv=(hit.xy-shadow_lo)/(shadow_hi-shadow_lo);vec2 sh=vec2(0);
  if(all(greaterThanEqual(uv,vec2(0)))&&all(lessThanEqual(uv,vec2(1))))sh=texture(shadow_tex,uv).rg;
  float shade=(1.-strength*sh.r)*(1.-.68*sh.g);
  float falloff=exp(-dot(hit.xy,hit.xy)*8.);float footprint=max(length(dFdx(hit.xy)),length(dFdy(hit.xy)));float detail=(grain(hit.xy*160.)-.5)*.085*(1.-smoothstep(.001,.006,footprint))+(grain(hit.xy*9.)-.5)*.08;vec3 f=floor_color*(.55+.45*falloff)*(1.+detail)*shade*vignette;
  bg=mix(bg,f,exp(-max(0.,length(hit.xy)-.35)*2.));
 }
 color=vec4(display_rgb(bg*exposure),1);
}
