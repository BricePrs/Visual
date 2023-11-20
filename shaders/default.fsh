#version 430 core

uniform vec3 cameraPosition;
uniform vec3 objectColor;
uniform vec3 backgroundColor;
uniform bool bOverrideColor;
uniform vec3 overrideColor;

out vec4 FragColor;
in vec3 vertexWorldPos;

void main() {
    vec3 relativePosition = cameraPosition-vertexWorldPos;
    float attenuationDist = mix(0.1, 0.04, clamp(abs(relativePosition.y), 0., 20.)/20.);
    float fogAttenuation = exp(-attenuationDist*length(relativePosition));
    vec3 color = mix(objectColor, overrideColor, float(bOverrideColor));
    FragColor = vec4(mix(backgroundColor, color, fogAttenuation), 1.);
}