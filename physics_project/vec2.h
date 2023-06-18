#ifndef VEC2_H
#define VEC2_H

#include <math.h>

typedef union {
	float r[2];
    struct {
        float x;
        float y;
    };
} vec2;

vec2 vec2_add(vec2 v1, vec2 v2)
{
    return (vec2){.x = v1.x + v2.x, .y = v1.y + v2.y};
}

vec2 vec2_sub(vec2 v1, vec2 v2)
{
    return (vec2){v1.x - v2.x, v1.y - v2.y};
}

vec2 vec2_scale(vec2 v1, float a)
{
    return (vec2){v1.x * a, v1.y * a};
}

vec2 vec2_rotate_rad(vec2 v1, float angle_rad)
{
    return (vec2){v1.x * cosf(angle_rad) - v1.y * sinf(angle_rad), v1.x * sinf(angle_rad) + v1.y * cosf(angle_rad)};
}

vec2 vec2_rotate_deg(vec2 v1, float angle_deg)
{
    float angle_rad = angle_deg / 180.0 * M_PI;
    return vec2_rotate_rad(v1, angle_rad);
}

float vec2_norm(vec2 v1)
{
    float norm_sq = v1.x * v1.x + v1.y * v1.y;
    return sqrtf(norm_sq);
}

float vec2_norm_squared(vec2 v1)
{
    return v1.x * v1.x + v1.y * v1.y;
}

vec2 vec2_unitvector(vec2 v1)
{
    float norm = vec2_norm(v1);
    if (norm > 0)
        return (vec2){v1.x / norm, v1.y / norm};
    else
        return (vec2){0, 0};
}

vec2 vec2_normal(vec2 v1)
{
    return vec2_unitvector((vec2){v1.y, -v1.x});
}

float vec2_dot(vec2 v1, vec2 v2)
{
    return v1.x * v2.x + v1.y * v2.y;
}

float vec2_cross(vec2 v1, vec2 v2)
{
    return (v1.x * v2.y) - (v1.y * v2.x);
}

#endif
