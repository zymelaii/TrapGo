#version 330
uniform samplerCube sampleCubeTexture;
in vec3 TexCoord;
out vec4 FragColor;
void main() {
    FragColor = texture(sampleCubeTexture, TexCoord);
}