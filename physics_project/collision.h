#ifndef COLLISION_H
#define COLLISION_H

#include <stdbool.h>
#include "body.h"
#include "vec2.h"

#include <stdio.h>
#include <float.h>

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
bool collision_polygon_polygon(Body *, Body *, Collision_Info *);

bool collision(Body *a, Body *b, Collision_Info *info)
{
    if (a->shape_type == CIRCLE && b->shape_type == CIRCLE)
    {
        return collision_circle_circle(a, b, info);
    }
    else if ((a->shape_type == POLYGON || a->shape_type == BOX) && (b->shape_type == POLYGON || b->shape_type == BOX))
    {
        return collision_polygon_polygon(a, b, info);
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

float collision_find_minimum_separation(Polygon *a, Polygon *b)
{
    float separation = -FLT_MAX;

    for (unsigned int i = 0; i < a->n_vertices; i++) {
        vec2 va = a->global_vertices[i];
        vec2 normal = vec2_normal(polygon_edge_at(a, i));

        float min_separation = FLT_MAX;

        for (int j = 0; j < b->n_vertices; j++)
        {
            vec2 vb = b->global_vertices[j];
            
            float this_separation = vec2_dot(vec2_sub(vb, va), normal);
            if (this_separation < min_separation)
                min_separation = this_separation;
        }
        if (min_separation > separation)
            separation = min_separation;
    }
    return separation;
}

bool collision_polygon_polygon(Body *a, Body *b, Collision_Info *info)
{
    float sep_ab = collision_find_minimum_separation((Polygon*)a->shape, (Polygon*)b->shape);
    float sep_ba = collision_find_minimum_separation((Polygon*)b->shape, (Polygon*)a->shape);

    if (sep_ab >= 0)
    {
        return false;
    }
    if (sep_ba >= 0)
    {
        return false;
    }
    return true;
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

void collision_info_resolve_collision(Collision_Info *info)
{
    collision_info_resolve_penetration(info);

    float e = info->a->restitution;
    if (info->b->restitution < info->a->restitution)
        e = info->b->restitution;
    
    vec2 v_rel = vec2_sub(info->a->velocity, info->b->velocity);

    float dot = vec2_dot(v_rel, info->normal);

    float impulse_magnitude = -(1.0 + e) * dot / (info->a->inv_mass + info->b->inv_mass);
    vec2 impulse_direction = info->normal;

    vec2 j = vec2_scale(impulse_direction, impulse_magnitude);

    body_apply_impulse(info->a, j);
    body_apply_impulse(info->b, vec2_scale(j, -1.0));
}

#endif