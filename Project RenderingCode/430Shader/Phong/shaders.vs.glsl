#version 430 core
layout(location = 0 ) in vec4 position;
layout(location = 1 ) in vec3 normal;
uniform mat4 view_mat;
uniform mat4 proj_mat;

uniform vec3 light_pos;

out vec3 v_o_normal;
out vec3 v_lightDir;
out vec3 v_halfDir;
out vec3 v_position;
out vec3 v_normal;

void main ()
{
	gl_Position = proj_mat*view_mat*position;
	vec3 worldpos = (view_mat*position).xyz;
	v_o_normal = (view_mat*vec4(normal,0.0)).xyz;
	v_lightDir = (view_mat*vec4(light_pos,0.0)).xyz;
	v_lightDir = normalize(v_lightDir);
	worldpos = normalize(worldpos);
	v_halfDir = normalize(-worldpos.xyz + v_lightDir);
	v_position = position.xyz;
	v_normal = normal.xyz;

}
