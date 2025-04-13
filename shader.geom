#version 450
//[
layout(points)in;
layout(triangle_strip, max_vertices=4)out;
//]

layout(location=0) out vec2 C;

void E(float u,float v){
    C=vec2(u+1,v+1);
    gl_Position=vec4(u,v,0,1);
    EmitVertex();
}

void main(){
    E(-1,-1);
    E(1,-1);
    E(-1,1);
    E(1,1);
}
