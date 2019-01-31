#version 430 core
uniform float TessLevelInner;
uniform float TessLevelOuter;

layout(vertices = 3) out;

in vec3 v_o_normal[];
in vec3 v_lightDir[];
in vec3 v_halfDir[];
in vec3 v_position[];
in vec3 v_normal[];

out vec3 tc_o_normal[];
out vec3 tc_lightDir[];
out vec3 tc_halfDir[];
out vec3 tc_position[];
out vec3 tc_normal[];

#define ID gl_InvocationID
void main(void)
{
	 if(ID == 0){
		gl_TessLevelOuter[0] = TessLevelOuter;
		gl_TessLevelOuter[1] = TessLevelOuter;
		gl_TessLevelOuter[2] = TessLevelOuter;
		gl_TessLevelInner[0] = TessLevelInner;
	 }
	
	gl_out[gl_InvocationID].gl_Position = gl_in[gl_InvocationID].gl_Position;
	tc_o_normal[ID] = v_o_normal[ID];
	tc_lightDir[ID] = v_lightDir[ID];
	tc_halfDir[ID] = v_halfDir[ID];
	tc_position[ID] = v_position[ID];
	tc_normal[ID] = v_normal[ID];

}
