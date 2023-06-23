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
bool collision_polygon_circle(Body *, Body *, Collision_Info *);

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
    else if ((a->shape_type == POLYGON || a->shape_type == BOX) && b->shape_type == CIRCLE)
    {
        return collision_polygon_circle(a, b, info);
    }
    else if (a->shape_type == CIRCLE && (b->shape_type == POLYGON || b->shape_type == BOX))
    {
        return collision_polygon_circle(b, a, info);
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

float collision_find_minimum_separation(Polygon *a, Polygon *b, vec2 *axis, vec2 *point)
{
    float separation = -FLT_MAX;

    for (unsigned int i = 0; i < a->n_vertices; i++)
    {
        vec2 va = a->global_vertices[i];
        vec2 normal = vec2_normal(polygon_edge_at(a, i));

        float min_separation = FLT_MAX;
        vec2 min_vertex;

        for (int j = 0; j < b->n_vertices; j++)
        {
            vec2 vb = b->global_vertices[j];

            float projection = vec2_dot(vec2_sub(vb, va), normal);

            if (projection < min_separation)
            {
                min_separation = projection;
                min_vertex = vb;
            }
        }
        if (min_separation > separation)
        {
            separation = min_separation;
            *axis = polygon_edge_at(a, i);
            *point = min_vertex;
        }
    }
    return separation;
}

bool collision_polygon_polygon(Body *a, Body *b, Collision_Info *info)
{
    vec2 a_axis, b_axis;
    vec2 a_point, b_point;
    float sep_ab = collision_find_minimum_separation((Polygon *)a->shape, (Polygon *)b->shape, &a_axis, &a_point);
    float sep_ba = collision_find_minimum_separation((Polygon *)b->shape, (Polygon *)a->shape, &b_axis, &b_point);

    if (sep_ab >= 0)
    {
        return false;
    }
    if (sep_ba >= 0)
    {
        return false;
    }

    info->a = a;
    info->b = b;

    if (sep_ab > sep_ba)
    {
        info->depth = -sep_ab;
        info->normal = vec2_normal(a_axis);
        info->start = a_point;
        info->end = vec2_add(a_point, vec2_scale(info->normal, info->depth));
    }
    else
    {
        info->depth = -sep_ba;
        info->normal = vec2_scale(vec2_normal(b_axis), -1);
        info->start = vec2_sub(b_point, vec2_scale(info->normal, info->depth));
        info->end = b_point;
    }

    return true;
}

void collision_info_resolve_penetration(Collision_Info *info)
{
    if (info->a->inv_mass == 0 && info->b->inv_mass == 0)
        return;

    float da = info->depth / (info->a->inv_mass + info->b->inv_mass) * info->a->inv_mass;
    float db = info->depth / (info->a->inv_mass + info->b->inv_mass) * info->b->inv_mass;

    float corr = 1.0;
    info->a->position = vec2_sub(info->a->position, vec2_scale(info->normal, da * corr));
    info->b->position = vec2_add(info->b->position, vec2_scale(info->normal, db * corr));

    shape_update_vertices(info->a->theta, info->a->position, info->a->shape_type, info->a->shape);
    shape_update_vertices(info->b->theta, info->b->position, info->b->shape_type, info->b->shape);
}

bool collision_polygon_circle(Body *a, Body *b, Collision_Info *info)
{
    Polygon *p = (Polygon *)a->shape;
    Circle *c = (Circle *)b->shape;

    bool is_outside = false;
    vec2 min_current_vertex;
    vec2 min_next_vertex;
    float distance_circle_edge = FLT_MIN;

    for (int i = 0; i < p->n_vertices; i++)
    {
        int i_next = (i + 1) % p->n_vertices;
        vec2 edge = polygon_edge_at(p, i);
        vec2 normal = vec2_normal(edge);

        vec2 circle_center = vec2_sub(b->position, p->global_vertices[i]);
        float projection = vec2_dot(circle_center, normal);

        if (projection > 0)
        {
            distance_circle_edge = projection;
            min_current_vertex = p->global_vertices[i];
            min_next_vertex = p->global_vertices[i_next];
            is_outside = true;
            break;
        }
        else
        {
            if (projection > distance_circle_edge)
            {
                distance_circle_edge = projection;
                min_current_vertex = p->global_vertices[i];
                min_next_vertex = p->global_vertices[i_next];
            }
        }
    }

    if (is_outside)
    {
        vec2 v1 = vec2_sub(b->position, min_current_vertex);
        vec2 v2 = vec2_sub(min_next_vertex, min_current_vertex);

        if (vec2_dot(v1, v2) < 0)
        {
            if (vec2_norm(v1) > c->radius)
            {
                return false;
            }
            else
            {
                info->a = a;
                info->b = b;
                info->depth = c->radius - vec2_norm(v1);
                info->normal = vec2_unitvector(v1);
                info->start = vec2_add(b->position, vec2_scale(info->normal, -1.0 * c->radius));
                info->end = vec2_add(info->start, vec2_scale(info->normal, info->depth));
            }
        }
        else
        {
            v1 = vec2_sub(b->position, min_next_vertex);
            v2 = vec2_sub(min_current_vertex, min_next_vertex);
            if (vec2_dot(v1, v2) < 0)
            {
                if (vec2_norm(v1) > c->radius)
                {
                    return false;
                }
                else
                {
                    info->a = a;
                    info->b = b;
                    info->depth = c->radius - vec2_norm(v1);
                    info->normal = vec2_unitvector(v1);
                    info->start = vec2_add(b->position, vec2_scale(info->normal, -1.0 * c->radius));
                    info->end = vec2_add(info->start, vec2_scale(info->normal, info->depth));
                }
            }
            else
            {
                if (distance_circle_edge > c->radius)
                {
                    return false;
                }
                else
                {
                    info->a = a;
                    info->b = b;
                    info->depth = c->radius - distance_circle_edge;
                    info->normal = vec2_normal(vec2_sub(min_next_vertex, min_current_vertex));
                    info->start = vec2_add(b->position, vec2_scale(info->normal, -1.0 * c->radius));
                    info->end = vec2_add(info->start, vec2_scale(info->normal, info->depth));
                }
            }
        }
    }
    else
    {
        info->a = a;
        info->b = b;
        info->depth = c->radius - distance_circle_edge;
        info->normal = vec2_normal(vec2_sub(min_next_vertex, min_current_vertex));
        info->start = vec2_add(b->position, vec2_scale(info->normal, -1.0 * c->radius));
        info->end = vec2_add(info->start, vec2_scale(info->normal, info->depth));
    }

    return true;
}

void collision_info_resolve_collision(Collision_Info *info)
{
    collision_info_resolve_penetration(info);

    float e = info->a->restitution;
    if (info->b->restitution < info->a->restitution)
        e = info->b->restitution;

    float f = info->a->friction;
    if (info->b->friction < info->a->friction)
        e = info->b->friction;

    // vec2 v_rel = vec2_sub(info->a->velocity, info->b->velocity);
    vec2 ra = vec2_sub(info->end, info->a->position);
    vec2 rb = vec2_sub(info->start, info->b->position);
    vec2 va = vec2_add(info->a->velocity, (vec2){-1.0 * info->a->omega * ra.y, info->a->omega * ra.x});
    vec2 vb = vec2_add(info->b->velocity, (vec2){-1.0 * info->b->omega * rb.y, info->b->omega * rb.x});
    vec2 v_rel = vec2_sub(va, vb);

    float v_rel_dot_normal = vec2_dot(v_rel, info->normal);
    vec2 impulse_N_direction = info->normal;
    float impulse_N_magnitude = -(1.0 + e) * v_rel_dot_normal;
    impulse_N_magnitude /= ((info->a->inv_mass + info->b->inv_mass) + vec2_cross(ra, info->normal) * vec2_cross(ra, info->normal) * info->a->inv_inertia + vec2_cross(rb, info->normal) * vec2_cross(rb, info->normal) * info->b->inv_inertia);

    vec2 tangent = vec2_normal(info->normal);
    float v_rel_dot_tangent = vec2_dot(v_rel, tangent);
    vec2 impulse_T_direction = tangent;
    float impulse_T_magnitude = f * -(1.0 + e) * v_rel_dot_tangent;
    impulse_T_magnitude /= ((info->a->inv_mass + info->b->inv_mass) + vec2_cross(ra, tangent) * vec2_cross(ra, tangent) * info->a->inv_inertia + vec2_cross(rb, tangent) * vec2_cross(rb, tangent) * info->b->inv_inertia);

    vec2 j_N = vec2_scale(impulse_N_direction, impulse_N_magnitude);
    vec2 j_T = vec2_scale(impulse_T_direction, impulse_T_magnitude);

    vec2 j_total = vec2_add(j_N, j_T);

    body_apply_impulse_at_r(info->a, j_total, ra);
    body_apply_impulse_at_r(info->b, vec2_scale(j_total, -1.0), rb);
}

#endif