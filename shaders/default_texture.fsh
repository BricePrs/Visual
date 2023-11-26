#version 430 core

uniform vec3 cameraPosition;
uniform vec3 backgroundColor;
uniform bool bOverrideColor;
uniform vec3 overrideColor;

uniform sampler2D objectTexture;

out vec4 FragColor;
in vec3 vertexWorldPos;
in vec2 uv;

void main() {
    vec3 relativePosition = cameraPosition-vertexWorldPos;
    float attenuationDist = mix(0.1, 0.04, clamp(abs(relativePosition.y), 0., 20.)/20.);
    float fogAttenuation = exp(-attenuationDist*length(relativePosition));
    vec3 objectColor = texture(objectTexture, uv).xyz;
    vec3 color = mix(objectColor, overrideColor, float(bOverrideColor));
    FragColor = vec4(mix(backgroundColor, color, fogAttenuation), 1.);
}