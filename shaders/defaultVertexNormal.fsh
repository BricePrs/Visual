#version 430 core

uniform vec3 cameraPosition;
uniform vec3 backgroundColor;
uniform bool bOverrideColor;
uniform vec3 overrideColor;

out vec4 FragColor;

in vec3 vertexWorldPos;
in vec3 vertexNormal;


void main() {
    float fogAttenuation = exp(-0.02*length(cameraPosition-vertexWorldPos));

    float ambt = 0.2;
    float diff = max(0., dot(vec3(1.), vertexNormal));

    FragColor = vec4(mix(backgroundColor, vec3(ambt+0.7*diff), fogAttenuation), 1.);
}