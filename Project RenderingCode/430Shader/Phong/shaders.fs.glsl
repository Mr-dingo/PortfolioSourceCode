#version 430 core
layout(location = 0 ) out vec4 o_color;


in vec3 g_o_normal;
in vec3 g_lightDir;
in vec3 g_halfDir;


void main ()
{
	vec3 color = vec3(0,0,0);
	vec3 normal = normalize(g_o_normal);
	vec3 halfv = normalize(g_halfDir);
	vec3 lightv = normalize(g_lightDir);
	float NdotL, NdotH;
	NdotL = max(dot(normal,lightv),0.0f);
	if(NdotL > 0) {
		color += 0.5*vec3(NdotL,0,0);
		NdotH = max(dot(normal,halfv),0.0);
		color += 0.5*vec3(pow(NdotH,10));
	}
	o_color += vec4(color,1.0f);

}
