#version 430 core

layout (location = 0) in vec3 aPosition;
layout (location = 1) in vec2 aUv;

uniform mat4 perspective;
uniform mat4 view;
uniform mat4 model;

out vec3 vertexWorldPos;
out vec2 uv;

void main() {
    gl_Position = perspective*view*model*vec4(aPosition, 1.);
    vertexWorldPos = vec3(model*vec4(aPosition, 1.));
    uv = aUv;
}