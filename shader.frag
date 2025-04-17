#version 450
#if defined(SPIRV)
layout(location=0) in vec2 C;
layout(location=0) out vec4 outColor;
layout(std140, binding=0) uniform X {
    vec2 SIZE;
};
#else
layout (location=0) in vec2 C;
out vec3 F;
layout (location=0) uniform vec2 SIZE;
#endif

// Shader minifier does not (currently) minimize structs, so use short names.
// Using a one-letter name for the struct itself seems to trigger a bug, so use two.
struct ma {
    float A; // ambient
    float D; // diffuse
    float P; // specular
    float S; // shininess
    float R; // reflection
    int T; // transparency index
    vec3 C; // RGB color
};

const vec3 background_color = vec3(0.8, 0.9, 1.0);

// Transparency parameters
struct tr {
    float T; // transparency amount
    float R; // refractive index (1 for vacuum)
    vec3 C; // fog color
    float E; // fog exponent
};

tr tr_air = tr(1.0, 1.0, background_color, 0.01);
tr tr_colored_glass = tr(0.8, 1.1, vec3(0.8, 0.6, 0.1), 0.4);
tr transparent_objects[2] = tr[2](tr_air, tr_colored_glass);

float DRAW_DISTANCE = 500.0;

float origin_sphere(vec3 p, float radius, bool inside) {
    float dist = length(p) - radius;
    return inside ? -dist : dist;
}

float horizontal_plane(vec3 p, float height) {
    return p.y - height;
}

float origin_box(vec3 p, vec3 dimensions, float corner_radius) {
    vec3 q = abs(p) - dimensions + corner_radius;
    return length(max(q, 0.0)) + min(max(q.x, max(q.y, q.z)), 0.0) - corner_radius;
}

void closest_material(inout float dist, inout ma mat, float new_dist, ma new_mat) {
    if (new_dist < dist) {
        dist = new_dist;
        mat = new_mat;
    }
}

float repeated_boxes_xz(vec3 p, vec3 dimensions, float corner_radius, float modulo) {
    p.xz = mod(p.xz - 0.5 * modulo, modulo) - 0.5 * modulo;
    return origin_box(p, dimensions, corner_radius);
}

float ground(vec3 p) {
    return min(
        horizontal_plane(p, -1),
        repeated_boxes_xz(vec3(p.x, p.y+2, p.z), vec3(1.1), 0.1, 5));
}

float window(vec3 p, bool inside) {
    float dist = origin_box(p, vec3(3.0, 3, 0.2), 0.5);
    return inside ? -dist : dist;
}

float scene(vec3 p, out ma mat, int inside) {
    float dist = origin_sphere(p + vec3(0.0, 0.0, 3.0), 1, false);
    mat = ma(0.1, 0.9, 0, 10, 0.5, 0, vec3(0.8));
    closest_material(dist, mat, window(p + vec3(0,-0.5,1), inside == 1), ma(0.1, 0.9, 0, 10, 0, 1, vec3(0.8)));
    closest_material(dist, mat, ground(p), ma(0.1, 0.9, 0, 10, 0.0, 0, vec3(0.8)));
    return dist;
}

bool ray_march(inout vec3 p, vec3 direction, out ma material, int inside) {
    float total_dist = 0.0;
    for (int i = 0; i < 5000; i++) {
        float dist = scene(p, material, inside);
        if (dist < 0.001) {
            return true;
        }
        total_dist += dist;
        if (total_dist > DRAW_DISTANCE) {
            return false;
        }
        p += direction * dist;
    }
    return false;
}

vec3 estimate_normal(vec3 p) {
    float epsilon = 0.001;
    ma m;
    return normalize(vec3(
        scene(vec3(p.x + epsilon, p.y, p.z), m, 0) - scene(vec3(p.x - epsilon, p.y, p.z), m, 0),
        scene(vec3(p.x, p.y + epsilon, p.z), m, 0) - scene(vec3(p.x, p.y - epsilon, p.z), m, 0),
        scene(vec3(p.x, p.y, p.z + epsilon), m, 0) - scene(vec3(p.x, p.y, p.z - epsilon), m, 0)
    ));
}

vec3 ray_reflection(vec3 direction, vec3 normal) {
    return 2.0 * dot(-direction, normal) * normal + direction;
}

float soft_shadow(vec3 p, vec3 light_direction, float sharpness) {
    ma m;
    p += light_direction * 0.1;
    float total_dist = 0.1;
    float res = 1.0;
    for (int i = 0; i < 20; i++) {
        float dist = scene(p, m, 0);
        if (dist < 0.01) {
            return 0.0;
        }
        total_dist += dist;
        res = min(res, sharpness * dist / total_dist);
        if (total_dist > DRAW_DISTANCE) {
            break;
        }
        p += light_direction * dist;
    }
    return res;
}


vec3 apply_fog(vec3 color, float total_distance, tr transparency) {
    return mix(color, transparency.C, 1.0 - exp(-transparency.E * total_distance));
}

vec3 phong_lighting(vec3 p, ma mat, vec3 ray_direction) {
    vec3 normal = estimate_normal(p);
    vec3 light_direction = normalize(vec3(-0.3, -1.0, -0.5));
    float shadow = soft_shadow(p, -light_direction, 20.0);
    float diffuse = max(0.0, mat.D * dot(normal, -light_direction)) * shadow;
    vec3 reflection = ray_reflection(ray_direction, normal);
    float specular = pow(max(0.0, mat.P * dot(reflection, -light_direction)), mat.S) * shadow;
    return min(mat.C * (diffuse + mat.A) + vec3(specular), vec3(1.0));
}

vec3 apply_reflections_and_transparency(vec3 color, ma mat, vec3 p, vec3 direction) {
    float blend_multiplier = 1.0;
    for (int i = 0; i < 3; i++) {
        // Apply either reflection or transparency in each step, not both.
        // Each involves ray marching steps that modify the position and material,
        // which makes it hard to support both in a single loop.
        int index = mat.T;
        float reflection = mat.R * blend_multiplier;
        if (index > 0) {
            tr tr_outside = tr_air;
            tr tr_inside = transparent_objects[index];
            float transparency = blend_multiplier * tr_inside.T;
            blend_multiplier *= transparency;
            // Offset the position to (hopefully) be inside the correct object
            vec3 normal = estimate_normal(p);
            p -= 0.02 * normal;
            vec3 start_pos = p;
            // apply incoming refraction
            direction = refract(direction, normal, tr_outside.R / tr_inside.R);
            // Ray march inside the object. For now, don't check the return value, assume hit
            ray_march(p, direction, mat, index);
            float inside_distance = length(p - start_pos);
            if (index == mat.T) {
                // We persumably hit the edge of the same object, so tweak the position to be outside and continue marching.
                // TODO: handle ending up in a different transparent object.
                normal = estimate_normal(p);
                p += 0.05 * normal;
                // apply outgoing refraction
                direction = refract(direction, -normal, tr_inside.R / tr_outside.R);
                if (ray_march(p, direction, mat, 0)) {
                    vec3 transparent_color = phong_lighting(p, mat, direction);
                    transparent_color = apply_fog(transparent_color, length(p - start_pos), tr_air);
                    color = mix(color, transparent_color, transparency);
                    color = apply_fog(color, inside_distance, tr_inside);
                } else {
                    color = mix(color, background_color, transparency);
                    color = apply_fog(color, inside_distance, tr_inside);
                    break;
                }
            } else {
                // hit another object inside the transparent one, assume that this one is not transparent
                vec3 transparent_color = phong_lighting(p, mat, direction);
                transparent_color = apply_fog(transparent_color, length(p - start_pos), tr_inside);
                color = mix(color, transparent_color, tr_inside.T);
            }
        } else if (reflection > 0.01) {
            blend_multiplier *= reflection;
            direction = ray_reflection(direction, estimate_normal(p));
            vec3 start_pos = p;
            p += 0.05 * direction;
            if (ray_march(p, direction, mat, 0)) {
                vec3 reflection_color = phong_lighting(p, mat, direction);
                reflection_color = apply_fog(reflection_color, length(p - start_pos), tr_air);
                color = mix(color, reflection_color, reflection);
            } else {
                color = mix(color, background_color, reflection);
                break;
            }
        } else {
            // No more transparency or reflections to apply
            break;
        }
    }
    return color;
}

vec3 render(float u, float v) {
    vec3 eye_position = vec3(0, 3, 4);
    vec3 forward = normalize(vec3(0, 0, -3) - eye_position);
    vec3 up = vec3(0.0, 1.0, 0.0);
    vec3 right = normalize(cross(up, forward));
    up = cross(-right, forward);
    float focal_length = 1.0;
    vec3 start_pos = eye_position + forward * focal_length + right * u + up * v;
    vec3 direction = normalize(start_pos - eye_position);
    vec3 p = start_pos;
    vec3 color = background_color;
    ma mat;
    if (ray_march(p, direction, mat, 0)) {
        color = phong_lighting(p, mat, direction);
        color = apply_reflections_and_transparency(color, mat, p, direction);
        color = apply_fog(color, length(p - start_pos), tr_air);
    }
    return color;
}

vec3 render_aa(float u, float v) {
    // Antialiasing: render and blend 2x2 points per pixel.
    // That means the distance between points is 1/2 pixel,
    // and the distance from the center (du, dv) is 1/4 pixel.
    // Each pixel size is (2.0 / W, 2.0 / H) since the full area is -1 to 1.
    float du = 2.0 / SIZE.x / 4.0;
    float dv = 2.0 / SIZE.y / 4.0;
    vec3 sum =
        render(u - du, v - dv) +
        render(u - du, v + dv) +
        render(u + du, v - dv) +
        render(u + du, v + dv);
    return sum / 4;
}

void main() {
    float u = C.x - 1.0;
    float v = (C.y - 1.0) * SIZE.y / SIZE.x;
#ifdef SPIRV
    vec3 F;
#endif
#if defined(DEBUG)
    F = render(u, v);
#else
    F = render_aa(u, v);
#endif
    // vignette
    float edge = abs(C.x - 1) + abs(C.y - 1);
    F = mix(F, vec3(0), min(1, max(0, edge*0.3 - 0.2)));
#ifdef SPIRV
    outColor = vec4(F, 0);
#endif
}
