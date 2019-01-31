#version 120

// deferred rendering vertex shader
// G-Buffer!!!!
attribute vec3 position;

varying vec2 texture_coord;

void main(void) {
        texture_coord = position.xy*0.5 + 0.5;
        gl_Position = vec4(position , 1.0); 
}
