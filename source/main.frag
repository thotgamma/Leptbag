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
	vec3 MaterialAmbientColor = vec3(0.5, 0.5, 0.5) * MaterialDiffuseColor;


	vec3 l = normalize(LightDirection);
	float cosTheta = clamp(dot(Normal, l), 0, 1);


	
	float bias = 0.0005*tan(acos(cosTheta)); // cosThetaはdot( n,l )で0と1の間にします。
	bias = clamp(bias, 0, 0.01);

	float visibility = texture( shadowMap, vec3(shadowCoord.xy, (shadowCoord.z - bias)/shadowCoord.w));


	color = MaterialAmbientColor
		  + visibility * MaterialDiffuseColor * LightColor * LightPower * cosTheta;

}
