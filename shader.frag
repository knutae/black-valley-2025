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

float HALF_PI = acos(0);

mat2 rotate(float degrees) {
    float a = degrees * HALF_PI / 90;
    return mat2(cos(a), -sin(a), sin(a), cos(a));
}

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

const vec3 background_color = vec3(0.1);

// Transparency parameters
struct tr {
    float T; // transparency amount
    float R; // refractive index (1 for vacuum)
    vec3 C; // fog color
    float E; // fog exponent
};

tr tr_air = tr(1.0, 1.0, background_color, 0.001);
tr tr_colored_glass = tr(0.8, 1.1, vec3(0.8, 0.6, 0.1), 0.4);

float DRAW_DISTANCE = 500.0;
float BATHROOM_WALL_DISTANCE = 15;
float BODY_Z_DISTANCE = BATHROOM_WALL_DISTANCE - 10;
float BODY_X_DISTANCE = 2;

float origin_sphere(vec3 p, float radius) {
    return length(p) - radius;
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

float bathroom_floor(vec3 p) {
    p.y += 2.5;
    return min(
        p.y,
        repeated_boxes_xz(vec3(p.x, p.y + 0.48, p.z), vec3(0.5), 0.12, 1));
}

float repeated_boxes_xy(vec3 p, vec3 dimensions, float corner_radius, float modulo) {
    p.xy = mod(p.xy - 0.5 * modulo, modulo) - 0.5 * modulo;
    return origin_box(p, dimensions, corner_radius);
}

float bathroom_wall(vec3 p) {
    p.z += BATHROOM_WALL_DISTANCE;
    return min(
        p.z,
        repeated_boxes_xy(vec3(p.x, p.y, p.z + 0.48), vec3(0.5), 0.12, 1));
}

vec3 bathroom_wall_color(vec3 p) {
    p.z += BATHROOM_WALL_DISTANCE;
    vec3 front = vec3(1);
    vec3 back = vec3(0.3);
    float a = clamp(p.z * 100, 0, 1);
    return mix(back, front, a);
}

vec3 bathroom_floor_color(vec3 p) {
    p.y += 2.5;
    vec3 front = vec3(0.8, 1, 0.8);
    vec3 back = vec3(0.3);
    float a = clamp(p.y * 100, 0, 1);
    return mix(back, front, a);
}

float repeated_spheres_xy(vec3 p, float radius, float modulo) {
    p.xy = mod(p.xy - 0.5 * modulo, modulo) - 0.5 * modulo;
    return origin_sphere(p, radius);
}

float window(vec3 p, bool inside) {
    vec3 q = p;
    q.xy *= rotate(-20);
    p.z += 0.005 * sin(q.x * 50 + 2 * sin(q.y * 15));
    p.y -= 9;
    float dist = origin_box(p, vec3(3.5, 8.5, 0.3), 0.1);
    q = p;
    q.z = abs(p.z - 0.6);
    float spheres = repeated_spheres_xy(q, 0.4, 0.5);
    dist = max(dist, -spheres);
    return inside ? -dist : dist;
}

float door_moldings(vec3 p, float width, float height) {
    float dist;
    vec3 q = p;
    q.y = abs(q.y) - height + 0.2;
    vec3 r = p;
    r.y = abs(r.y) - height - width;
    r.xy *= rotate(45);
    q.yz *= rotate(10);
    dist = max(
        origin_box(q, vec3(width + 0.2, 0.2, 0.5), 0.05),
        origin_box(r, vec3(sqrt(width*width*2)), 0));
    q = p;
    r = q;
    r.x = abs(r.x) - width - height;
    r.xy *= rotate(45);
    q.x = abs(q.x) - width + 0.2;
    q.xz *= rotate(10);
    dist = min(dist, max(
        origin_box(q, vec3(0.2, height + 0.2, 0.5), 0.05),
        origin_box(r, vec3(sqrt(height*height*2)), 0)));
    return dist;
}

float door(vec3 p) {
    p.y -= 9;
    float dist = origin_box(p, vec3(5, 10, 0.4), 0.1);
    dist = max(dist, -origin_box(p, vec3(3, 8, 1), 0.01));
    dist = min(dist, door_moldings(p, 3, 8));
    dist = min(dist, door_moldings(p, 5.2, 9.8));
    return dist;
}

float sdCappedCylinder( vec3 p, float h, float r )
{
  vec2 d = abs(vec2(length(p.xz),p.y)) - vec2(r,h);
  return min(max(d.x,d.y),0.0) + length(max(d,0.0));
}

float opSmoothIntersection( float d1, float d2, float k )
{
    float h = clamp( 0.5 - 0.5*(d2-d1)/k, 0.0, 1.0 );
    return mix( d2, d1, h ) + k*h*(1.0-h);
}

float opSmoothUnion( float d1, float d2, float k )
{
    float h = clamp( 0.5 + 0.5*(d2-d1)/k, 0.0, 1.0 );
    return mix( d2, d1, h ) - k*h*(1.0-h);
}

float keyhole(vec3 p) {
    p.x -= 4;
    p.y += 0.8;
    float dist = length(p.xy) - 0.13;
    p.y += 0.2;
    dist = min(dist, origin_box(p, vec3(0.07, 0.2, 2), 0));
    return dist;
}

float door_handle(vec3 p) {
    p.y -= 8.75;
    float hole = keyhole(p);
    p.z += 2;
    p.x -= 4;
    vec3 q = p;
    q.yz *= rotate(90);
    float dist = sdCappedCylinder(q, 2, 0.2);
    q = p;
    q.xy *= rotate(-90);
    q.z = abs(q.z) - 2;
    dist = opSmoothUnion(dist, origin_sphere(q, 0.28), 0.05);
    q.y -= 1;
    dist = opSmoothUnion(dist, sdCappedCylinder(q, 1, 0.2), 0.1);
    p.y += 0.5;
    dist = min(dist, origin_box(p, vec3(0.5, 1.3, 1.5), 0.15));
    dist = opSmoothIntersection(dist, -hole, 0.05);
    return dist;
}

float front_wall(vec3 p) {
    p.z += 1;
    if (p.z > -0.1) {
        p.z += 0.003 * sin(203 * p.x) + 0.005*sin(11*p.x);
    }
    float dist = abs(p.z) - 0.45;
    p.y -= 9;
    vec2 q = abs(p.xy) - vec2(5.1, 10.1);
    float doorway = length(max(q, 0.0)) + min(max(q.x, q.y), 0.0);
    return max(dist, -doorway);
}

vec3 wallpaper_color(vec3 p) {
    if (p.z < -1.1) {
        return vec3(1);
    }
    float modulo = 1.5;
    vec2 q = p.xy * rotate(45);
    q = mod(q - 0.5 * modulo, modulo) - 0.5 * modulo;
    vec3 col1 = vec3(0.3, 0.7, 0.3);
    vec3 col2 = vec3(0.6, 0.3, 0.15);
    return (q.x*q.y) < 0 ? col1 : col2;
}

float sink(vec3 p) {
    p.x += 5;
    p.z += BATHROOM_WALL_DISTANCE;
    p.y -= 2;
    float dist = origin_box(p, vec3(1, 5, 1), 0.5);
    p.y -= 5;
    p.z -= 2;
    dist = min(dist, origin_box(p, vec3(8, 2, 2), 0.5));
    return dist;
}

float mirror(vec3 p) {
    p.x += 5;
    p.z += BATHROOM_WALL_DISTANCE;
    p.y -= 16;
    return origin_box(p, vec3(8, 6, 0.1), 0.05);
}

float sdRoundCone( vec3 p, float r1, float r2, float h )
{
  // sampling independent computations (only depend on shape)
  float b = (r1-r2)/h;
  float a = sqrt(1.0-b*b);

  // sampling dependant computations
  vec2 q = vec2( length(p.xz), p.y );
  float k = dot(q,vec2(-b,a));
  if( k<0.0 ) return length(q) - r1;
  if( k>a*h ) return length(q-vec2(0.0,h)) - r2;
  return dot(q, vec2(a,b) ) - r1;
}

float sdEllipsoid( vec3 p, vec3 r )
{
  float k0 = length(p/r);
  float k1 = length(p/(r*r));
  return k0*(k0-1.0)/k1;
}

float lower_body(vec3 p) {
    p.x += BODY_X_DISTANCE;
    p.x = abs(p.x) - 0.8;
    p.z += BODY_Z_DISTANCE;
    float dist = sdRoundCone(p, 0.5, 0.6, 4);
    p.y -= 4;
    p.xy *= rotate(-5);
    dist = opSmoothUnion(dist, sdRoundCone(p, 0.6, 0.65, 3), 0.01);
    p.y -= 7;
    dist = opSmoothUnion(dist, origin_sphere(vec3(p.x + 0.4, p.y, p.z), 0.8), 0.1);
    return dist;
}

float upper_body(vec3 p) {
    p.x += BODY_X_DISTANCE;
    p.y -= 7;
    p.z += BODY_Z_DISTANCE;
    float dist = sdRoundCone(p, 1.15, 0.9, 1);
    p.y -= 1;
    dist = opSmoothUnion(dist, sdRoundCone(p, 0.9, 1.1, 3), 0.1);
    p.y -= 5;
    dist = opSmoothUnion(dist, sdEllipsoid(p, vec3(0.7, 1, 1)), 0.2);
    return dist;
}

float arms(vec3 p) {
    p.x += BODY_X_DISTANCE;
    p.z += BODY_Z_DISTANCE;
    p.y -= 11;
    p.x = abs(p.x) - 0.7;
    p.xy *= rotate(60);
    p.yz *= rotate(50);
    float dist = sdRoundCone(p, 0.6, 0.5, 3);
    p.y -= 3;
    p.yz *= rotate(70);
    p.xy *= rotate(-60);
    dist = opSmoothUnion(dist, sdRoundCone(p, 0.5, 0.35, 2.5), 0.1);
    return dist;
}

float body(vec3 p) {
    float dist = lower_body(p);
    dist = opSmoothUnion(dist, upper_body(p), 0.2);
    dist = opSmoothUnion(dist, arms(p), 0.1);
    return dist;
}

float hair(vec3 p) {
    p.x += BODY_X_DISTANCE;
    p.z += BODY_Z_DISTANCE - 0.5;
    p.y -= 7;
    float dist = sdRoundCone(p, 1.5, 1, 6);
    dist = max(dist, -p.y + 1);
    return dist;
}

float scene(vec3 p, out ma mat, int inside) {
    float dist = DRAW_DISTANCE;
    mat = ma(0, 0, 0, 10, 0, 0, vec3(0));
    closest_material(dist, mat, window(p + vec3(0,-0.5,1), inside == 1), ma(0.1, 0.9, 0, 10, 0, 1, vec3(0.8)));
    closest_material(dist, mat, door(p + vec3(0,-0.5,1)), ma(0.1, 0.9, 0, 10, 0, 0, vec3(0.5)));
    closest_material(dist, mat, door_handle(p), ma(0.1, 0.9, 0.8, 5, 0, 0, vec3(0.8, 0.8, 0.4)));
    closest_material(dist, mat, bathroom_floor(p), ma(0.1, 0.9, 0, 10, 0.0, 0, bathroom_floor_color(p)));
    closest_material(dist, mat, bathroom_wall(p), ma(0.1, 0.9, 0, 10, 0, 0, bathroom_wall_color(p)));
    closest_material(dist, mat, front_wall(p), ma(0.03, 0.97, 0, 10, 0, 0, wallpaper_color(p)));
    closest_material(dist, mat, sink(p), ma(0.1, 0.9, 0, 10, 0, 0, vec3(0.7, 1, 0.7)));
    closest_material(dist, mat, mirror(p), ma(0.1, 0.9, 0, 10, 1, 0, vec3(0)));
    closest_material(dist, mat, body(p), ma(0.15, 0.85, 0.8, 5, 0, 0, vec3(1, 0.8, 0.75)));
    closest_material(dist, mat, hair(p), ma(0.1, 0.9, 0, 10, 0, 0, vec3(0.1)));
    return dist;
}

bool ray_march(inout vec3 p, vec3 direction, out ma material, int inside) {
    float total_dist = 0.0;
    for (int i = 0; i < 4000; i++) {
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

float soft_shadow(vec3 p, vec3 light_direction, float max_distance, float sharpness) {
    ma m;
    p += light_direction * 0.1;
    float total_dist = 0.1;
    float res = 1.0;
    for (int i = 0; i < 50; i++) {
        float dist = scene(p, m, 0);
        if (dist < 0.01) {
            return 0.0;
        }
        total_dist += dist;
        res = min(res, sharpness * dist / total_dist);
        if (total_dist > max_distance) {
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
    vec3 diffuse_and_specular_sum = vec3(0);
    for (int i = 0; i < 5; i++) {
        vec3 light_pos = vec3(15, 15, 5);
        if (i == 1) {
            light_pos = vec3(5, 26, -BATHROOM_WALL_DISTANCE + 1);
        } else if (i == 2) {
            light_pos = vec3(-5, 26, -BATHROOM_WALL_DISTANCE + 1);
        } else if (i == 3) {
            light_pos = vec3(5, 26, -3);
        } else if (i == 4) {
            light_pos = vec3(-5, 26, -3);
        }
        vec3 light_color = i == 0 ? vec3(0.3) : vec3(1);
        vec3 light_direction = normalize(p - light_pos);
        float light_distance = length(p - light_pos);
        float light_dropoff = i == 0 ? -0.01 : -0.05;
        float light_intensity = exp(light_dropoff * light_distance);
        float shadow = soft_shadow(p, -light_direction, light_distance, 40.0);
        float diffuse = max(0.0, mat.D * dot(normal, -light_direction)) * shadow * light_intensity;
        vec3 reflection = ray_reflection(ray_direction, normal);
        float specular = pow(max(0.0, mat.P * dot(reflection, -light_direction)), mat.S) * shadow;
        diffuse_and_specular_sum += (mat.C * diffuse * light_color) + vec3(specular);
    }
    return min(mat.A + diffuse_and_specular_sum, vec3(1.0));
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
            tr tr_inside = tr_colored_glass; // the only transparent object
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
    vec3 eye_position = vec3(0, 10, 15);
    vec3 forward = normalize(vec3(0, 10, -3) - eye_position);
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

void main() {
    float u = C.x - 1.0;
    float v = (C.y - 1.0) * SIZE.y / SIZE.x;
#ifdef SPIRV
    vec3 F;
#endif
    F = render(u, v);
    // vignette
    float edge = abs(C.x - 1) + abs(C.y - 1);
    F = mix(F, vec3(0), min(1, max(0, edge*0.3 - 0.2)));
#ifdef SPIRV
    outColor = vec4(F, 0);
#endif
}
