#ifndef VEC2_H
#define VEC2_H

#include <math.h>
#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif

typedef union {
	float r[2];
    struct {
        float x;
        float y;
    };
} Vec2;

extern inline Vec2 vec2_add(Vec2 v1, Vec2 v2)
{
    return (Vec2){.x = v1.x + v2.x, .y = v1.y + v2.y};
}

extern inline Vec2 vec2_sub(Vec2 v1, Vec2 v2)
{
    return (Vec2){v1.x - v2.x, v1.y - v2.y};
}

extern inline Vec2 vec2_scale(Vec2 v1, float a)
{
    return (Vec2){v1.x * a, v1.y * a};
}

extern inline Vec2 vec2_rotate_rad(Vec2 v1, float angle_rad)
{
    return (Vec2){v1.x * cosf(angle_rad) - v1.y * sinf(angle_rad), v1.x * sinf(angle_rad) + v1.y * cosf(angle_rad)};
}

extern inline Vec2 vec2_rotate_deg(Vec2 v1, float angle_deg)
{
    float angle_rad = angle_deg / 180.0 * M_PI;
    return vec2_rotate_rad(v1, angle_rad);
}

extern inline float vec2_norm(Vec2 v1)
{
    float norm_sq = v1.x * v1.x + v1.y * v1.y;
    return sqrtf(norm_sq);
}

extern inline float vec2_norm_squared(Vec2 v1)
{
    return v1.x * v1.x + v1.y * v1.y;
}

extern inline Vec2 vec2_unitvector(Vec2 v1)
{
    float norm = vec2_norm(v1);
    if (norm > 0)
        return (Vec2){v1.x / norm, v1.y / norm};
    else
        return (Vec2){0, 0};
}

extern inline Vec2 vec2_normal(Vec2 v1)
{
    return vec2_unitvector((Vec2){v1.y, -v1.x});
}

extern inline float vec2_dot(Vec2 v1, Vec2 v2)
{
    return v1.x * v2.x + v1.y * v2.y;
}

extern inline float vec2_cross(Vec2 v1, Vec2 v2)
{
    return (v1.x * v2.y) - (v1.y * v2.x);
}

#endif
