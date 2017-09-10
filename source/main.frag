#version 330 core

// Interpolated values from the vertex shaders
in vec3 fragmentColor;
in vec3 Normal;
in vec4 shadowCoord;


// Ouput data
out vec3 color;

uniform vec3 LightColor;
uniform float LightPower;
uniform vec3 LightDirection;
uniform sampler2DShadow shadowMap;


void main(){


	vec3 MaterialDiffuseColor = fragmentColor;
	vec3 MaterialAmbientColor = vec3(0.3, 0.3, 0.3) * MaterialDiffuseColor;


	vec3 l = normalize(LightDirection);
	float cosTheta = clamp(dot(Normal, l), 0, 1);


	
	float bias = 0.0005*tan(acos(cosTheta)); // cosThetaはdot( n,l )で0と1の間にします。
	bias = clamp(bias, 0, 0.01);

	float visibility = 1.0f;
	if (0 <= shadowCoord.x && shadowCoord.x <= 1 && 0 <= shadowCoord.y && shadowCoord.y <= 1){
		if (texture( shadowMap, vec3(shadowCoord.xy, (shadowCoord.z - bias)/shadowCoord.w)) == 0){
			visibility = 0.5f;
		}
	}


	color = MaterialAmbientColor
		  + visibility * MaterialDiffuseColor * LightColor * LightPower * cosTheta;

}
