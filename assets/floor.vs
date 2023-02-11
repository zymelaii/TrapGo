#version 330

layout (location = 0) in vec2 Coord;
layout (location = 1) in float colorValue;

uniform mat4 WVP;
uniform vec3 rolePosition;

out vec3 color;
out vec3 pos;

void main() {
    const float scale = 6.0;
    vec2 origin = floor((rolePosition.xy + scale) / (scale * 2.0)) * scale * 2.0;
    gl_Position = WVP * vec4(Coord * scale + origin, 0.0, 1.0);
    pos = vec3(Coord, 0.0);
    color = vec3(colorValue, colorValue, colorValue);
}
