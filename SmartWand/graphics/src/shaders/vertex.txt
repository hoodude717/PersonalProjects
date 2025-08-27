#version 120

attribute vec2 vertexPos;
attribute vec3 vertexColor;

varying vec3 fragmentColor;

void main() {
    gl_Position = vec4(vertexPos, 0.0, 1.0);
    fragmentColor = vertexColor;
}