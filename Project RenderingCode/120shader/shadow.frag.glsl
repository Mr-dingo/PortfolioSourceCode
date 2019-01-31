#version 120
uniform mat4 proj_mat;
varying vec4 frag_vert;

void main () {
        vec4 last_pos = proj_mat * frag_vert;
        last_pos = last_pos / last_pos[3];
        //gl_FragCoord
        gl_FragColor = vec4(last_pos[2],last_pos[2],last_pos[2],1);
}
