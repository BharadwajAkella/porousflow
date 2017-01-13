#version 430

layout (location=0) out vec4 FragColor;

subroutine vec3 shadeModelType(vec3 normal);
subroutine uniform shadeModelType shadeModel;

//in vec3 lightIntensity;
in vec4 color;
in vec2 texCoord;
in vec3 normal;
in vec3 position;

vec4 light_pos = vec4(10,10,10,1.0);
float light_intensity = 1.0;
// Material
vec3 Ka = vec3(0.1f, 0.1f, 0.1f);
vec3 Kd = vec3(0.8f, 0.4f, 0.2f);
vec3 Ks = vec3(0.9f, 0.9f, 0.9f);
//Light
vec3 La = vec3(0.4f, 0.4f, 0.4f);
vec3 Ld = vec3(1.0f, 1.0f, 1.0f);
vec3 Ls = vec3(1.0f, 1.0f, 1.0f);
float Shininess = 9.f ;

uniform mat4 mvpMatrix;
uniform mat4 normalMatrix;

uniform sampler2D textureSampler;


float rand(vec2 co){
    return fract(sin(dot(co.xy ,vec2(12.9898,78.233))) * 43758.5453);
}

subroutine(shadeModelType)
vec3 phongLighting( vec3 n)
{
	vec4 eyeCoordinates = mvpMatrix * vec4(position, 1.0);

    vec3 s = normalize((light_pos).xyz - eyeCoordinates.xyz);
    vec3 v = normalize(-eyeCoordinates.xyz);
    vec3 r = reflect( -s, n );
    float sDotN = max( abs(dot(s,n)), 0.0 );
	vec3 diffuse = Ld * Kd * sDotN;
	vec3 ambient =  La * Ka;
    vec3 spec = vec3(0.0);
    if( sDotN > 0.0 )
        spec = Ls * Ks * pow( max( dot(r,v), 0.0 ), Shininess );
    return diffuse + spec + ambient;
}

subroutine(shadeModelType)
vec3 diffuseLighting( vec3 n)
{
	vec4 eyeCoordinates = mvpMatrix * vec4(position, 1.0);

    vec3 s = normalize((light_pos).xyz - eyeCoordinates.xyz);
    vec3 v = normalize(-eyeCoordinates.xyz);
    vec3 r = reflect( -s, n );
    float sDotN = max( abs(dot(s,n)), 0.0 );
	vec3 diffuse = Ld * Kd * sDotN;
	vec3 ambient =  La * Ka;
    return diffuse + ambient;
}


void main() {
 //   FragColor = vec4(1.0, 0.0, 0.0, 1.0);
 //   vec4 fragmentTexColor = texture2D(textureSampler, texCoord.xy);
	//	//FragColor = fragmentTexColor;

	//vec3 n = normalize(vec3(normalMatrix) * vec3(normal));
	//FragColor = vec4(phongLighting(n), 1.0);
	//if( gl_FrontFacing ) {
	//	FragColor = vec4(frontColor, 1.0);
	//} else {
	//	FragColor = vec4(backColor, 1.0);
	//}
	//FragColor = vec4(1.f, 0.f, 0.f, 1.f);
	FragColor = color;
}
