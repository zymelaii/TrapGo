#version 330
layout (location = 0) in vec3 Position;
uniform mat4 WVP;
out vec3 TexCoord;
void main() {
    vec4 pos = vec4(Position.xyz * 1000.0, 1.0);
    gl_Position = (WVP * pos).xyww;
    TexCoord = vec3(pos.x, pos.z, pos.y);
}
