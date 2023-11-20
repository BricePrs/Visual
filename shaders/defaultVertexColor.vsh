#version 430 core

layout (location = 0) in vec3 aPosition;
layout (location = 1) in vec3 aColor;

uniform mat4 perspective;
uniform mat4 view;
uniform mat4 model;

out vec3 vertexColor;
out vec3 vertexWorldPos;

void main() {
    gl_Position = perspective*view*model*vec4(aPosition, 1.);
    vertexWorldPos = vec3(model*vec4(aPosition, 1.));
    vertexColor = aColor;
}