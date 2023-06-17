#ifndef COLLISION_H
#define COLLISION_H

#include <stdbool.h>
#include "body.h"
#include "vec2.h"

#include <stdio.h>

bool collision_circle_circle(Body *, Body *);

bool collision(Body *a, Body *b)
{
    if (a->shape_type == CIRCLE && b->shape_type == CIRCLE)
    {
        return collision_circle_circle(a, b);
    }
    else
    {
        return false;
    }
}

bool collision_circle_circle(Body *a, Body *b)
{
    Circle *c_a = (Circle *)a->shape;
    Circle *c_b = (Circle *)b->shape;

    float distance = vec2_norm(vec2_sub(b->position, a->position));

    if (distance <= (c_a->radius + c_b->radius))
    {
        return true;
    }
    else
    {
        return false;
    }
}

#endif