#version 430 core

uniform vec3 cameraPosition;
uniform vec3 backgroundColor;
uniform bool bOverrideColor;
uniform vec3 overrideColor;

out vec4 FragColor;

in vec3 vertexColor;
in vec3 vertexWorldPos;


void main() {
    float fogAttenuation = exp(-0.02*length(cameraPosition-vertexWorldPos));
    vec3 color = mix(vertexColor, overrideColor, float(bOverrideColor));
    FragColor = vec4(mix(backgroundColor, color, fogAttenuation), 1.);
}