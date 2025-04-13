#ifdef SPIRV
#version 450
#endif
// a vertex shader is required, but all necessary position logic is in the geometry shader
void main() {
    gl_Position=vec4(0);
}
