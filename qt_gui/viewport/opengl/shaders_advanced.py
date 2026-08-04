""" Advanced shaders. AI-generated.

Author: Rainer Meyer, r.meyer494@gmail.com
"""

VERTEX_SHADER_SRC = """
#version 330 core

layout(location = 0) in vec3 aPos;
layout(location = 1) in vec4 aColor;
layout(location = 2) in vec3 aNormal; // Added vertex normals

out vec3 FragPos;      // World position for specular/view calculations
out vec3 Normal;       // World normal vector
out vec4 VertColor;    // Base color passed through

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;
uniform mat3 normalMatrix; // Inverse transpose of model matrix for correct non-uniform scaling

void main() {
    // Transform vertex position to world space
    vec4 worldPos = model * vec4(aPos, 1.0);
    FragPos = worldPos.xyz;

    // Transform normal to world space
    Normal = normalMatrix * aNormal;

    VertColor = aColor;
    gl_Position = projection * view * worldPos;
}
"""

FRAGMENT_SHADER_SRC = """
#version 330 core

in vec3 FragPos;
in vec3 Normal;
in vec4 VertColor;

out vec4 FragColor;

uniform vec3 viewPos; // Camera position in world space

void main() {
    vec3 norm = normalize(Normal);
    vec3 viewDir = normalize(viewPos - FragPos);

    // 1. Primary Directional Light (Overhead/Top-down with slight forward angle)
    vec3 lightDir = normalize(vec3(0.1, 1.0, 0.2)); 

    // 2. Hemispherical Lighting (CAD style: bright top, soft ground bounce)
    float upFraction = norm.y * 0.5 + 0.5; // Maps normal.y from [-1, 1] to [0, 1]
    vec3 skyColor = vec3(0.95, 0.95, 0.98); // Bright overhead daylight
    vec3 groundColor = vec3(0.30, 0.30, 0.35); // Subtle dark ground bounce
    vec3 hemiAmbient = mix(groundColor, skyColor, upFraction) * 0.45;

    // 3. Diffuse Shading (Lambertian)
    float diff = max(dot(norm, lightDir), 0.0);
    vec3 diffuse = diff * vec3(0.55);

    // 4. Specular Highlight (Blinn-Phong for smooth CAD highlights)
    vec3 halfwayDir = normalize(lightDir + viewDir);
    float spec = pow(max(dot(norm, halfwayDir), 0.0), 32.0); // 32.0 controlling shininess
    vec3 specular = vec3(0.25) * spec;

    // 5. Fresnel Rim Lighting (Enhances edge/silhouette visibility)
    float fresnel = pow(1.0 - max(dot(norm, viewDir), 0.0), 3.0);
    vec3 rim = vec3(0.15) * fresnel;

    // Combine all lighting components
    vec3 totalLighting = hemiAmbient + diffuse + specular + rim;

    // Apply lighting to the base color
    vec3 result = VertColor.rgb * totalLighting;

    FragColor = vec4(result, VertColor.a);
}
"""
