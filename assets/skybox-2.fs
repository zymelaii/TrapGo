#version 330

in vec3 TexCoord;

uniform samplerCube sampleCubeTexture;
uniform float time;

out vec4 FragColor;

void main() {
	vec3 lightColor = vec3(1.0, 1.0, 1.0);
    float intensity = (clamp(sin((time - 6) / 24.0 * 2 * 3.1415926), -0.9, 1.0) + 1.0) / 2.0;
	vec4 texColor = texture(sampleCubeTexture, TexCoord);
    FragColor = vec4(texColor.rgb * lightColor * intensity, texColor.a);
}
