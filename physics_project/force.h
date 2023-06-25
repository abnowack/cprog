#ifndef FORCE_H
#define FORCE_H

#include "vec2.h"
#include "body.h"

Vec2 force_drag(Body *p, float k)
{
    Vec2 drag = (Vec2){0, 0};

    if (vec2_norm_squared(p->velocity) > 0)
    {
        Vec2 direction = vec2_scale(vec2_unitvector(p->velocity), -1);

        float magnitude = k * vec2_norm_squared(p->velocity);

        drag = vec2_scale(direction, magnitude);
    }

    return drag;
}

Vec2 force_gravity(Body *p1, Body *p2, float G, float min_distance, float max_distance)
{
    Vec2 d = vec2_sub(p2->position, p1->position);
    float distance = vec2_norm(d);

    if (distance < min_distance)
        distance = min_distance;
    if (distance > max_distance)
        distance = max_distance;

    Vec2 force_direction = vec2_unitvector(d);
    float force_magnitude = G * p1->mass * p2->mass / (distance * distance);
    Vec2 gravity = vec2_scale(force_direction, force_magnitude);

    return gravity;
}

Vec2 force_spring(Body *p, Vec2 anchor, float rest_length, float k)
{
    Vec2 d = vec2_sub(p->position, anchor);
    float displacement = vec2_norm(d) - rest_length;

    Vec2 spring_direction = vec2_unitvector(d);
    float spring_magnitude = -k * displacement;

    return vec2_scale(spring_direction, spring_magnitude);
}

#endif