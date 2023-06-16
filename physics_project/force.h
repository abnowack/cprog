#ifndef FORCE_H
#define FORCE_H

#include "vec2.h"
#include "particle.h"

vec2 force_drag(Particle *p, float k)
{
    vec2 drag = (vec2){0, 0};

    if (vec2_norm_squared(p->velocity) > 0)
    {
        vec2 direction = vec2_scale(vec2_unitvector(p->velocity), -1);

        float magnitude = k * vec2_norm_squared(p->velocity);

        drag = vec2_scale(direction, magnitude);
    }

    return drag;
}

vec2 force_friction(Particle *p, float k)
{
    vec2 direction = vec2_scale(vec2_unitvector(p->velocity), -1);

    vec2 friction = vec2_scale(direction, k);

    return friction;
}

#endif