#version 330 core
in vec3 pos; in vec3 N0; in vec2 texcoord; in vec4 T0;
out vec4 color;
uniform vec3 eye, emissive;
uniform vec4 base;
uniform vec4 surface; // metallic, roughness, normal scale, IOR
uniform vec4 optical; // transmission, iridescence, coating IOR, clearcoat
uniform vec4 coating; // min nm, max nm, anisotropy, coat roughness
uniform sampler2D base_tex, mr_tex, normal_tex, thickness_tex, aniso_tex;
uniform int has_base,has_mr,has_normal,has_thickness,has_aniso,is_transparent;
uniform float exposure;
const float PI=3.14159265;
vec3 srgb(vec3 c){return mix(c/12.92,pow((c+.055)/1.055,vec3(2.4)),step(vec3(.04045),c));}
vec3 display_rgb(vec3 c){c=max(c,vec3(0));c=clamp((c*(2.51*c+.03))/(c*(2.43*c+.59)+.14),0.,1.);return mix(12.92*c,1.055*pow(c,vec3(1./2.4))-.055,step(vec3(.0031308),c));}
vec3 fresnel(vec3 f,float v){return f+(1.-f)*pow(1.-clamp(v,0.,1.),5.);}
vec3 studio(vec3 r,float rough){
    float a=max(dot(r,normalize(vec3(.3,-.6,1.))),0.);
    float b=max(dot(r,normalize(vec3(-.8,.6,.7))),0.);
    float c=max(dot(r,normalize(vec3(.8,.7,.25))),0.);
    float k=mix(40.,3.,rough);
    return vec3(.12,.15,.19)+vec3(2.8,2.7,2.5)*pow(a,k)+vec3(1.7,2.0,2.5)*pow(b,k*.5)+vec3(1.7,1.5,1.3)*pow(c,k*.7)+max(r.z,0.)*.24;
}
vec3 direct(vec3 N,vec3 V,vec3 L,vec3 radiance,vec3 albedo,float metal,float rough,vec3 f0,vec3 T,vec3 B,float aniso){
    float nl=max(dot(N,L),0.),nv=max(dot(N,V),.001);if(nl<=0.)return vec3(0);
    vec3 H=normalize(V+L);float nh=max(dot(N,H),.001),vh=max(dot(V,H),0.);
    float alpha=rough*rough,ax=max(.002,alpha*(1.+aniso)),ay=max(.002,alpha*(1.-aniso*.75));
    float d=pow(dot(H,T)/ax,2.)+pow(dot(H,B)/ay,2.)+nh*nh;
    float D=1./max(PI*ax*ay*d*d,.000001);float k=pow(rough+1.,2.)/8.;
    float G=nv/(nv*(1.-k)+k)*nl/(nl*(1.-k)+k);vec3 F=fresnel(f0,vh);
    return radiance*nl*((1.-F)*(1.-metal)*albedo/PI+D*G*F/max(4.*nv*nl,.001));
}
void main(){
    vec3 N=normalize(N0),T=normalize(T0.xyz-dot(T0.xyz,N)*N);vec3 B=cross(N,T)*T0.w;
    if(!gl_FrontFacing){N=-N;B=-B;}
    if(has_normal==1){vec3 n=texture(normal_tex,texcoord).xyz*2.-1.;n.xy*=surface.z;N=normalize(mat3(T,B,N)*n);}
    T=normalize(T-dot(T,N)*N);B=cross(N,T)*T0.w;
    vec4 bc=base;if(has_base==1){vec4 t=texture(base_tex,texcoord);bc*=vec4(srgb(t.rgb),t.a);}
    float metal=surface.x,rough=surface.y;if(has_mr==1){vec3 mr=texture(mr_tex,texcoord).rgb;metal*=mr.b;rough*=mr.g;}
    rough=clamp(rough,.065,1.);vec3 V=normalize(eye-pos);float nv=max(dot(N,V),0.);
    float dielectric=pow((surface.w-1.)/(surface.w+1.),2.);vec3 f0=mix(vec3(dielectric),bc.rgb,metal);
    float a=coating.z;if(has_aniso==1){vec3 at=texture(aniso_tex,texcoord).rgb;vec2 dir=normalize(at.rg*2.-1.+vec2(.00001));T=normalize(T*dir.x+B*dir.y);B=cross(N,T);a*=at.b;}
    if(optical.y>0.){float thick=mix(coating.x,coating.y,has_thickness==1?texture(thickness_tex,texcoord).g:.5);
        float c=sqrt(max(0.,1.-(1.-nv*nv)/(optical.z*optical.z)));vec3 wave=vec3(650.,510.,445.);
        vec3 film=.5+.5*cos(4.*PI*optical.z*thick*c/wave);f0=mix(f0,clamp(f0*(.45+1.7*film),0.,1.),optical.y);}
    vec3 F=fresnel(f0,nv);vec3 R=reflect(-V,N);vec3 env=studio(R,rough);
    vec3 lit=env*F*(1.-rough*.5)+bc.rgb*(1.-metal)*vec3(.25,.28,.34)*(1.-F);
    lit+=direct(N,V,normalize(vec3(.3,-.6,1.)),vec3(2.4,2.3,2.2),bc.rgb,metal,rough,f0,T,B,a);
    lit+=direct(N,V,normalize(vec3(-.8,.6,.7)),vec3(1.3,1.6,2.),bc.rgb,metal,rough,f0,T,B,a);
    lit+=direct(N,V,normalize(vec3(.8,.7,.25)),vec3(.7,.65,.6),bc.rgb,metal,rough,f0,T,B,a);
    lit+=optical.w*studio(R,coating.w)*fresnel(vec3(.04),nv)+emissive;
    float alpha=bc.a;
    if(optical.x>0.){ // deliberately approximate transmission by sorted surface blending, not ray refraction
        float fr=clamp(max(F.r,max(F.g,F.b)),0.,1.);alpha=clamp((1.-optical.x)+fr*.85,.025,.95);
        lit=mix(bc.rgb*.30,env*F/max(alpha,.025)+bc.rgb*.06,.75)+emissive;
    }
    color=vec4(display_rgb(lit*exposure),is_transparent==1?alpha:1.);
}
