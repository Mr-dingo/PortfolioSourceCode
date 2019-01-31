#version 430 core
layout(triangles) in; 
layout(triangle_strip, max_vertices = 4) out;
in vec3 te_o_normal[3];
in vec3 te_lightDir[3];
in vec3 te_halfDir[3];


out vec3 g_o_normal;
out vec3 g_lightDir;
out vec3 g_halfDir;


void main(void)
{
	 	gl_Position = gl_in[0].gl_Position;
		g_o_normal = te_o_normal[0];
		g_lightDir = te_lightDir[0];
		g_halfDir = te_halfDir[0];
	EmitVertex();
		 gl_Position = gl_in[1].gl_Position;
		g_o_normal = te_o_normal[1];
		g_lightDir = te_lightDir[1];
		g_halfDir = te_halfDir[1];
	EmitVertex();
		gl_Position = gl_in[2].gl_Position;
		g_o_normal = te_o_normal[2];
		g_lightDir = te_lightDir[2];
		g_halfDir = te_halfDir[2];
	EmitVertex();
		gl_Position = gl_in[0].gl_Position;
	EmitVertex();
	EndPrimitive();
}
