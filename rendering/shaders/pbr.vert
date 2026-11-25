#version 330 core
layout(location=0) in vec3 position;
layout(location=1) in vec3 normal;
layout(location=2) in vec2 uv;
layout(location=3) in vec4 tangent;
uniform mat4 vp;
out vec3 pos; out vec3 N0; out vec2 texcoord; out vec4 T0;
void main(){pos=position;N0=normal;texcoord=uv;T0=tangent;gl_Position=vp*vec4(position,1.0);}
