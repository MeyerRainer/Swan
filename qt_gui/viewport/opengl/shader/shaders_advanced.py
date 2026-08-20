""" Advanced shaders. AI-generated.

Author: Rainer Meyer, r.meyer494@gmail.com
"""

# VERTEX_SHADER_SRC = """
# #version 330 core
#
# layout(location = 0) in vec3 aPos;
# layout(location = 1) in vec3 aNormal;
#
# out vec3 FragPos;
# out vec3 Normal;
#
# uniform mat4 model;
# uniform mat4 view;
# uniform mat4 projection;
# uniform mat3 normalMatrix;
#
# void main() {
#     vec4 worldPos = model * vec4(aPos, 1.0);
#     FragPos = worldPos.xyz;
#     Normal = normalMatrix * aNormal;
#     gl_Position = projection * view * worldPos;
# }
# """
#
# FRAGMENT_SHADER_SRC = """
# #version 330 core
#
# in vec3 FragPos;
# in vec3 Normal;
#
# out vec4 FragColor;
#
# uniform vec3 viewPos;
# uniform vec3 diffuseColor;
#
# void main() {
#     vec3 norm = normalize(Normal);
#     vec3 viewDir = normalize(viewPos - FragPos);
#
#     // 1. Primary Directional Light (Overhead/Top-down)
#     vec3 lightDir = normalize(vec3(0.0, 0.0, 0.8));
#
#     // 2. Hemispherical Ambient Lighting
#     float upFraction = norm.y * 0.5 + 0.5;
#     vec3 skyColor = vec3(0.95, 0.95, 0.98);
#     vec3 groundColor = vec3(0.30, 0.30, 0.35);
#     vec3 hemiAmbient = mix(groundColor, skyColor, upFraction) * 0.45;
#
#     // 3. Diffuse Shading
#     float diff = max(dot(norm, lightDir), 0.0);
#     vec3 diffuse = diff * vec3(0.55);
#
#     // 4. Specular Highlight (Blinn-Phong)
#     vec3 halfwayDir = normalize(lightDir + viewDir);
#     float spec = pow(max(dot(norm, halfwayDir), 0.0), 32.0);
#     vec3 specular = vec3(0.25) * spec;
#
#     // 5. Fresnel Rim Lighting
#     float fresnel = pow(1.0 - max(dot(norm, viewDir), 0.0), 3.0);
#     vec3 rim = vec3(0.15) * fresnel;
#
#     // Combine lighting with the material diffuse color
#     vec3 totalLighting = hemiAmbient + diffuse + specular + rim;
#     FragColor = vec4(diffuseColor * totalLighting, 1.0);
# }
# """
#
# VERTEX_SHADER_SRC = """
# #version 330 core
#
# layout(location = 0) in vec3 aPos;
# layout(location = 1) in vec3 aNormal;
# layout(location = 2) in vec4 aColor; // Changed to vec4 for RGBA
#
# out vec3 FragPos;
# out vec3 Normal;
# out vec4 VertColor;
#
# uniform mat4 model;
# uniform mat4 view;
# uniform mat4 projection;
# uniform mat3 normalMatrix;
#
# void main() {
#     vec4 worldPos = model * vec4(aPos, 1.0);
#     FragPos = worldPos.xyz;
#     Normal = normalMatrix * aNormal;
#     VertColor = aColor;
#     gl_Position = projection * view * worldPos;
# }
# """
#
# FRAGMENT_SHADER_SRC = """
# #version 330 core
#
# in vec3 FragPos;
# in vec3 Normal;
# in vec4 VertColor;
#
# out vec4 FragColor;
#
# uniform vec3 viewPos;
#
# void main() {
#     vec3 norm = normalize(Normal);
#     vec3 viewDir = normalize(viewPos - FragPos);
#
#     // Overhead lighting setup
#     vec3 lightDir = normalize(vec3(0.0, 0.0, 0.8));
#     float upFraction = norm.y * 0.5 + 0.5;
#     vec3 hemiAmbient = mix(vec3(0.30, 0.30, 0.35), vec3(0.95, 0.95, 0.98), upFraction) * 0.45;
#
#     float diff = max(dot(norm, lightDir), 0.0);
#     vec3 diffuse = diff * vec3(0.55);
#
#     vec3 halfwayDir = normalize(lightDir + viewDir);
#     float spec = pow(max(dot(norm, halfwayDir), 0.0), 32.0);
#     vec3 specular = vec3(0.25) * spec;
#
#     float fresnel = pow(1.0 - max(dot(norm, viewDir), 0.0), 3.0);
#     vec3 rim = vec3(0.15) * fresnel;
#
#     vec3 totalLighting = hemiAmbient + diffuse + specular + rim;
#
#     // Preserve the original alpha channel (VertColor.a)
#     FragColor = vec4(VertColor.rgb * totalLighting, VertColor.a);
# }
# """

VERTEX_SHADER_SRC = """
#version 330 core

layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aNormal;
layout(location = 2) in vec4 aColor; // Per-vertex color (RGBA)

out vec3 FragPos;
out vec3 Normal;
out vec4 VertColor;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;
uniform mat3 normalMatrix;

void main() {
    vec4 worldPos = model * vec4(aPos, 1.0);
    FragPos = worldPos.xyz;
    Normal = normalMatrix * aNormal;
    VertColor = aColor; // Passed down to fragment shader
    gl_Position = projection * view * worldPos;
}
"""

FRAGMENT_SHADER_SRC = """
#version 330 core

in vec3 FragPos;
in vec3 Normal;
in vec4 VertColor;

out vec4 FragColor;

uniform vec3 viewPos;

void main() {
    vec3 norm = normalize(Normal);
    vec3 viewDir = normalize(viewPos - FragPos);

    // 1. Primary Directional Light
    vec3 lightDir = normalize(vec3(0.0, 0.0, 0.8));

    // 2. AMBIENT: Hemispherical Ambient Lighting
    float upFraction = norm.y * 0.5 + 0.5;
    vec3 skyColor = vec3(0.95, 0.95, 0.98);
    vec3 groundColor = vec3(0.30, 0.30, 0.35);
    vec3 hemiAmbient = mix(groundColor, skyColor, upFraction) * 0.45;

    // 3. DIFFUSE: Lambertian Diffuse
    float diff = max(dot(norm, lightDir), 0.0);
    vec3 diffuse = diff * vec3(0.55);

    // 4. SPECULAR: Blinn-Phong Specular Highlight
    vec3 halfwayDir = normalize(lightDir + viewDir);
    float spec = pow(max(dot(norm, halfwayDir), 0.0), 32.0);
    vec3 specular = vec3(0.25) * spec; // Bright glossy highlight spot

    // 5. Rim / Fresnel
    float fresnel = pow(1.0 - max(dot(norm, viewDir), 0.0), 3.0);
    vec3 rim = vec3(0.15) * fresnel;

    // AMBIENT & DIFFUSE apply to the surface color (VertColor.rgb)
    // SPECULAR & RIM are added on top so glossy reflections remain white/bright
    vec3 surfaceColor = VertColor.rgb * (hemiAmbient + diffuse);
    vec3 finalRGB = surfaceColor + specular + rim;

    // Output with preserved vertex Alpha (translucency)
    FragColor = vec4(finalRGB, VertColor.a);
}
"""