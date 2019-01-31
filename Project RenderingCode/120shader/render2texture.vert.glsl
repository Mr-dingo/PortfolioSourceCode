#version 120
// render to texture shader ( vert)
attribute vec3 position;
attribute vec3 normal;

uniform mat4 proj_mat;
uniform mat4 view_mat;
uniform mat4 model_mat;
varying vec4 frag_vert;
varying vec3 frag_normal;
varying vec4 ori_vert;
varying vec3 ori_norm;

void main(void) {
        frag_vert =view_mat* model_mat * vec4(position , 1.0f);
        frag_normal = (view_mat * model_mat * vec4(normal , 0 )).xyz ;
        ori_vert = vec4(position,1.0f);
        // ori_norm = normal;
        gl_Position = proj_mat * view_mat* model_mat * vec4(position , 1.0); // 바 뀔가능성 도있음..
}
