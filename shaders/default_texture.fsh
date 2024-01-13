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
    float fogAttenuation = exp2(-.1*length(relativePosition));
    ivec2 texSize = textureSize(objectTexture, 0);
    vec3 objectColor = texelFetch(objectTexture, ivec2(uv*vec2(texSize)), 0).xyz;
    vec3 color = mix(objectColor, overrideColor, float(bOverrideColor));

/*    float texColor = texelFetch(objectTexture, ivec2(uv*vec2(texSize)), 0).a;
    float meansq = dot(color.xyz, color.xyz);
    float stdev = sqrt(abs(texColor - meansq));*/
    FragColor = vec4(mix(backgroundColor, color, clamp(fogAttenuation, 0., 1.)), 1.);// + vec4(stdev, 0., 0., 0.);
}