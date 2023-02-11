#version 330

in vec3 color;

out vec4 FragColor;

void main() {
    vec3 lightColor = vec3(1.0, 1.0, 1.0);
    float intensity = 0.8;
    FragColor = vec4(color * lightColor * intensity, 1.0);
}
