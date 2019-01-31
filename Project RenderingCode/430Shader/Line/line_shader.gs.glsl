#version 430 core
layout(triangles) in; 
layout(line_strip, max_vertices = 4) out;
in vec3 te_o_normal[3];
in vec3 te_lightDir[3];
in vec3 te_halfDir[3];


out vec3 g_o_normal;
out vec3 g_lightDir;
out vec3 g_halfDir;


void main(void)
{
	vec4 pos_x = gl_in[0].gl_Position;
	vec4 pos_y = gl_in[1].gl_Position;
	vec4 pos_z = gl_in[2].gl_Position;
    
	gl_Position = pos_x+vec4(normalize(te_o_normal[0])*0.01f,0.0f);
		g_o_normal = te_o_normal[0];
		g_lightDir = te_lightDir[0];
		g_halfDir = te_halfDir[0];
	EmitVertex();
		gl_Position = pos_y+vec4(normalize(te_o_normal[1])*0.01f,0.0f);
		g_o_normal = te_o_normal[1];
		g_lightDir = te_lightDir[1];
		g_halfDir = te_halfDir[1];
	EmitVertex();
		gl_Position = pos_z+vec4(normalize(te_o_normal[2])*0.01f,0.0f);
		g_o_normal = te_o_normal[2];
		g_lightDir = te_lightDir[2];
		g_halfDir = te_halfDir[2];
	EmitVertex();
		gl_Position = pos_x+vec4(normalize(te_o_normal[0])*0.01f,0.0f);
	EmitVertex();
	EndPrimitive();
}
