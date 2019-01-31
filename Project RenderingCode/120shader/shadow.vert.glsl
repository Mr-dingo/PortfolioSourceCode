#version 120

attribute vec3 position;

uniform mat4 proj_mat;
uniform mat4 view_mat;
uniform mat4 model_mat;
varying vec4 frag_vert;

void main(void) {
   frag_vert =view_mat* model_mat * vec4(position , 1);
   gl_Position = proj_mat * view_mat* model_mat * vec4(position , 1.0); // 바 뀔가능성 도있음..
}
