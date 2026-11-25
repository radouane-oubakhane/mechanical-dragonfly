#version 330 core
out vec2 ndc;
void main(){vec2 p=vec2((gl_VertexID<<1)&2,gl_VertexID&2)*2.-1.;ndc=p;gl_Position=vec4(p,0,1);}
