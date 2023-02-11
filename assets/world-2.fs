#version 330

in vec3 color;
in vec3 pos;

uniform float time;
struct {
    vec3 pos;
    vec3 dir;
    float cutoff;
    float maxCutoff;
} SpotLight;

out vec4 FragColor;

void main() {
    vec3 lightColor = vec3(1.0, 1.0, 1.0);

    vec3 ambientLight = lightColor * (clamp(sin((time - 6) / 24.0 * 2 * 3.1415926), -0.9, 1.0) + 1.0) / 2.0;

    float range = cos(radians(SpotLight.cutoff)) - cos(radians(SpotLight.maxCutoff));
    float c_theta = dot(normalize(pos - SpotLight.pos), normalize(SpotLight.dir));
    float intensity = clamp((c_theta - cos(radians(SpotLight.maxCutoff))) / range, 0.0, 1.0);
    vec3 spotLight = lightColor * intensity;

    FragColor = vec4(color * (ambientLight), 1.0);
}
