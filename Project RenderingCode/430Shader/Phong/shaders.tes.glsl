#version 430 core

layout(triangles) in;
in vec3 tc_o_normal[];
in vec3 tc_lightDir[];
in vec3 tc_halfDir[];
in vec3 tc_position[];
in vec3 tc_normal[];

out vec3 te_o_normal;
out vec3 te_lightDir;
out vec3 te_halfDir;

void main()
{	
	vec3 p0 = gl_TessCoord.x * tc_position[0];
    vec3 p1 = gl_TessCoord.y * tc_position[1];
    vec3 p2 = gl_TessCoord.z * tc_position[2];

	gl_Position = gl_in[0].gl_Position * gl_TessCoord.x + gl_in[1].gl_Position * gl_TessCoord.y+ gl_in[2].gl_Position * gl_TessCoord.z;
	float x = gl_TessCoord.x;
	float y = gl_TessCoord.y;
	float z = gl_TessCoord.z;
	if(x>y && x>z){}
	else if(y>x && y>z){}
	else{}
	te_o_normal = tc_o_normal[0] * gl_TessCoord.x + tc_o_normal[1] * gl_TessCoord.y+ tc_o_normal[2] * gl_TessCoord.z;
	te_lightDir = tc_lightDir[0] * gl_TessCoord.x + tc_lightDir[1] * gl_TessCoord.y+ tc_lightDir[2] * gl_TessCoord.z;
	te_halfDir = tc_halfDir[0] * gl_TessCoord.x + tc_halfDir[1] * gl_TessCoord.y+ tc_halfDir[2] * gl_TessCoord.z;
	


}
