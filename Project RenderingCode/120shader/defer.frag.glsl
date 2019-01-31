#version 120
//render to texture shader ( frag )
//G-Buffer !!!!!!!

#define kernel_size 30
#define light_num 13
// NEED!!!!
uniform mat4 proj_mat;
uniform mat4 view_mat;
struct Light{
    mat4 view_mat;
    mat4 proj_mat;
    vec4 position;
    int turn;  //if turn on turn == 1; off == 0;
    sampler2D shadow_map;
};
uniform Light light[light_num];
uniform sampler2D ssao_norm_tex;
uniform sampler2D ssao_depth_tex;
uniform float u_radius;

uniform sampler2D defer_pos_tex;
uniform sampler2D defer_amb_tex;
uniform sampler2D defer_dif_tex;
uniform sampler2D defer_spe_tex;
uniform sampler2D defer_ori_tex;

varying vec2 texture_coord;

float tube_light_length = 150;

vec3 kernel[kernel_size];
mat4 half_mat;

vec4 world_vert = texture2D(defer_ori_tex , texture_coord).xyzw; //pre multiplied model_mat.
vec4 frag_vert = texture2D(defer_pos_tex , texture_coord).xyzw;
vec3 frag_normal = texture2D(ssao_norm_tex , texture_coord).xyz *2.0 - 1;
vec4 material_diffuse = texture2D(defer_dif_tex , texture_coord).xyzw;
vec4 material_specular = texture2D(defer_spe_tex , texture_coord).xyzw;
vec4 material_ambient = texture2D(defer_amb_tex , texture_coord).xyzw;


float rand(vec2 n)
{
    return fract(sin(dot(n.xy, vec2(12.9898, 78.233)))* 43758.5453);
}
vec3 compute_oc(){
    float occlusion = 0.0f;
    vec4 screen_space_vert = proj_mat * frag_vert;
    screen_space_vert /= screen_space_vert.w;
    screen_space_vert.xy = screen_space_vert.xy * 0.5+ 0.5; // [0,1]

    vec3 origin = frag_vert.xyz ; //[* , *]
    vec3 space_norm = texture2D( ssao_norm_tex , screen_space_vert.xy).xyz *2.0 -1.0 ;
    space_norm = normalize(space_norm);
    vec3 rvec = normalize (vec3(rand( frag_vert.xy ) , rand(frag_vert.xz) , 0 )) *2.0 -1.0;
    vec3 tangent = normalize(rvec - space_norm * dot(rvec, space_norm));
    vec3 bitangent = (cross(space_norm, tangent));
    mat3 tbn = mat3(tangent, bitangent, space_norm);

    for(int i = 0 ; i < kernel_size; i++){
        kernel[i] = vec3(
            rand(vec2(float(i) , frag_vert.x)),
            rand(vec2(frag_vert.y , float(i))),
            1.0f
            );
        kernel[i] = normalize(kernel[i]);
        kernel[i] *= pow((rand(vec2(frag_vert.y*frag_vert.x , float(i))) * 0.5 + 0.5) , 2);

        vec3 sample_point = tbn * kernel[i]; //[*,*]
        sample_point = sample_point * u_radius + origin;
        vec4 offset = vec4(sample_point, 1.0);
        offset = proj_mat * offset;
        offset.xy /= offset.w; // [*,*]
        offset.xy = offset.xy * 0.5 + 0.5;

        float sample_depth =texture2D( defer_pos_tex , offset.xy).b;  //[0 , 1]
        float range_check = abs( sample_point.z - sample_depth ) < u_radius ? 1.0: 0.0;
        occlusion += (sample_depth >= sample_point.z ? 1.0 : 0.0 ) * range_check;
    }
    occlusion /= float(kernel_size);
    occlusion = 1.0f - occlusion;

    return vec3(occlusion);

}




void main () {
    half_mat[0] = vec4(0.5 , 0 , 0 , 0);
    half_mat[1] = vec4(0 , 0.5 , 0 , 0);
    half_mat[2] = vec4(0 , 0 , 1 , 0);
    half_mat[3] = vec4(0.5 , 0.5 ,0,1);


    vec4 o_color = vec4(0,0,0,1);
    for(int i=0 ; i < light_num; i++)
    {
//in world position
        if( light[i].turn == 1){
            float visability = 1.0;
            vec4 tex_coord = (light[i].proj_mat * light[i].view_mat * world_vert);
            tex_coord = tex_coord / tex_coord[3];
            tex_coord = half_mat * tex_coord;
            float depth_at_this = texture2D(light[i].shadow_map , vec2(tex_coord.xy)).r;
            if( tex_coord[2] > depth_at_this+0.001)
                visability = 0;

            vec3 surfacePos = (frag_vert).xyz;
            vec3 surfaceToLight ;
            vec3 surfaceToCamera = normalize(-surfacePos);
            float distanceToLight = length((view_mat * vec4(light[i].position.xyz , 1)).xyz - surfacePos);
            float attenuation = 1.0 / (1.0 + 0.06 * pow(distanceToLight, 1));
            if( i > 9){
                //tube light starting point
                vec3 light_1_dir = (view_mat * vec4(light[i].position.xyz , 1)).xyz - surfacePos ;
                vec3 light_2_dir = (view_mat * vec4(light[i].position.x + tube_light_length , light[i].position.y , light[i].position.z  , 1.0f )).xyz - surfacePos;
                float upper_part = dot(normalize(frag_normal) , normalize(light_1_dir)) + dot(normalize(frag_normal) , normalize(light_2_dir));
                float down_part = 1.0;
                float diffuseCoefficient = max(upper_part/down_part , 0.0f );
                vec3 diffuse = diffuseCoefficient*material_diffuse.xyz * gl_LightSource[0].diffuse.xyz;

                float specularCoefficient = 0.0;
                if(diffuseCoefficient > 0 )
                {
                    vec3 reflected = normalize(reflect( -surfaceToCamera ,normalize(frag_normal)));
                    vec3 d_12 = light_2_dir - light_1_dir;
                    float t_upper = dot(light_1_dir , d_12)*dot(reflected , light_1_dir) - dot(light_1_dir , light_1_dir)*dot(reflected , d_12);
                    float t_down  = dot(light_1_dir , d_12)*dot(reflected , d_12) - dot(d_12, d_12 )*dot(reflected , light_1_dir);
                    vec3 L_smallest = light_1_dir + (t_upper/t_down)*d_12;
                    specularCoefficient = max(pow(dot(reflected , normalize(L_smallest)) , 30)  , 0.0f);
                }
                vec3 specular = specularCoefficient *material_specular.xyz * gl_LightSource[0].specular.xyz;
                vec3 linearColor = visability*attenuation*(diffuse + specular);
                o_color += vec4(linearColor,0);
            }
                    // point light or directional light
            else{
                if(light[i].position[3] > 0.5f)
                    surfaceToLight = normalize((view_mat * vec4(light[i].position.xyz,1)).xyz - surfacePos);
                else
                    surfaceToLight = normalize((view_mat * (light[i].position)).xyz);
                // diffuse term
                float diffuseCoefficient = max(0.0, dot(normalize(frag_normal), surfaceToLight));
                vec3 diffuse = diffuseCoefficient * material_diffuse.xyz * gl_LightSource[0].diffuse.xyz;

                //specular term
                float specularCoefficient = 0.0;
                if(diffuseCoefficient > 0.0)
                {
                    specularCoefficient = pow(max(0.0, dot(surfaceToCamera, reflect(-surfaceToLight, normalize(frag_normal)))), 20);
                }
                vec3 specular = specularCoefficient *material_specular.xyz * gl_LightSource[0].specular.xyz;
                vec3 linearColor = visability*attenuation*(diffuse + specular);
                o_color += vec4(linearColor,0);
            }
        }
    }
    vec3 ambient_color = compute_oc() * material_ambient.xyz;
    gl_FragColor = o_color + vec4(ambient_color,0);
}
