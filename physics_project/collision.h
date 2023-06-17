#ifndef COLLISION_H
#define COLLISION_H

#include <stdbool.h>
#include "body.h"
#include "vec2.h"

#include <stdio.h>

typedef struct 
{
    Body *a;
    Body *b;

    vec2 start;
    vec2 end;
    vec2 normal;
    float depth;
} Collision_Info;


bool collision_circle_circle(Body *, Body *, Collision_Info *);

bool collision(Body *a, Body *b, Collision_Info *info)
{
    if (a->shape_type == CIRCLE && b->shape_type == CIRCLE)
    {
        return collision_circle_circle(a, b, info);
    }
    else
    {
        return false;
    }
}

bool collision_circle_circle(Body *a, Body *b, Collision_Info *info)
{
    Circle *c_a = (Circle *)a->shape;
    Circle *c_b = (Circle *)b->shape;

    float distance = vec2_norm(vec2_sub(b->position, a->position));

    if (distance <= (c_a->radius + c_b->radius))
    {
        info->a = a;
        info->b = b;

        info->normal = vec2_unitvector(vec2_sub(b->position, a->position));

        vec2 v1 = vec2_scale(info->normal, c_b->radius);
        info->start = vec2_sub(b->position, v1);

        vec2 foo = vec2_scale(info->normal, c_a->radius);
        info->end = vec2_add(a->position, foo);

        info->depth = vec2_norm(vec2_sub(info->end, info->start));

        return true;
    }
    else
    {
        return false;
    }
}

void collision_info_resolve_penetration(Collision_Info *info)
{
    if (info->a->inv_mass == 0 && info->b->inv_mass == 0)
        return;
        
    float da = info->depth / (info->a->inv_mass + info->b->inv_mass) * info->a->inv_mass;
    float db = info->depth / (info->a->inv_mass + info->b->inv_mass) * info->b->inv_mass;

    info->a->position = vec2_sub(info->a->position, vec2_scale(info->normal, da));
    info->b->position = vec2_add(info->b->position, vec2_scale(info->normal, db));
}

#endif