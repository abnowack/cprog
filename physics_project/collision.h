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

bool collision_circle_circle(Body *, Body *, Collision_Info[], unsigned int *);
bool collision_polygon_polygon(Body *, Body *, Collision_Info[], unsigned int *);
bool collision_polygon_circle(Body *, Body *, Collision_Info[], unsigned int *);

bool collision(Body *a, Body *b, Collision_Info info[], unsigned int *n_collisions)
{
    if (a->shape_type == CIRCLE && b->shape_type == CIRCLE)
    {
        return collision_circle_circle(a, b, info, n_collisions);
    }
    else if ((a->shape_type == POLYGON || a->shape_type == BOX) && (b->shape_type == POLYGON || b->shape_type == BOX))
    {
        return collision_polygon_polygon(a, b, info, n_collisions);
    }
    else if ((a->shape_type == POLYGON || a->shape_type == BOX) && b->shape_type == CIRCLE)
    {
        return collision_polygon_circle(a, b, info, n_collisions);
    }
    else if (a->shape_type == CIRCLE && (b->shape_type == POLYGON || b->shape_type == BOX))
    {
        return collision_polygon_circle(b, a, info, n_collisions);
    }
    else
    {
        return false;
    }
}

bool collision_circle_circle(Body *a, Body *b, Collision_Info info[], unsigned int *n_collisions)
{
    Collision_Info *contact = &info[*n_collisions];

    Circle *c_a = (Circle *)a->shape;
    Circle *c_b = (Circle *)b->shape;

    float distance = vec2_norm(vec2_sub(b->position, a->position));

    if (distance <= (c_a->radius + c_b->radius))
    {
        contact->a = a;
        contact->b = b;

        contact->normal = vec2_unitvector(vec2_sub(b->position, a->position));

        vec2 v1 = vec2_scale(contact->normal, c_b->radius);
        contact->start = vec2_sub(b->position, v1);

        vec2 foo = vec2_scale(contact->normal, c_a->radius);
        contact->end = vec2_add(a->position, foo);

        contact->depth = vec2_norm(vec2_sub(contact->end, contact->start));

        (*n_collisions)++;
        return true;
    }
    else
    {
        return false;
    }
}

float collision_find_minimum_separation(Polygon *a, Polygon *b, unsigned int *index_reference_edge, vec2 *support_point)
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
            *index_reference_edge = i;
            *support_point = min_vertex;
        }
    }
    return separation;
}

unsigned int polygon_find_incident_edge(Polygon *shape, vec2 normal)
{
    unsigned int incident_edge;
    float min_projection = FLT_MAX;
    for (unsigned int i = 0; i < shape->n_vertices; ++i)
    {
        vec2 edge_normal = vec2_normal(polygon_edge_at(shape, i));
        float projection = vec2_dot(edge_normal, normal);
        if (projection < min_projection)
        {
            min_projection = projection;
            incident_edge = i;
        }
    }
    return incident_edge;
}

int polygon_clip_segment_to_line(Polygon *shape, vec2 contacts_in[2], vec2 contacts_out[2], vec2 *c0, vec2 *c1)
{
    unsigned int num_out = 0;

    vec2 normal = vec2_unitvector(vec2_sub(*c1, *c0));
    float dist0 = vec2_cross(vec2_sub(contacts_in[0], *c0), normal);
    float dist1 = vec2_cross(vec2_sub(contacts_in[1], *c0), normal);

    if (dist0 <= 0)
    {
        contacts_out[num_out++] = contacts_in[0];
    }
    if (dist1 <= 0)
    {
        contacts_out[num_out++] = contacts_in[1];
    }

    if (dist0 * dist1 < 0)
    {
        float total_dist = dist0 - dist1;

        float t = dist0 / total_dist;
        vec2 contact = vec2_add(contacts_in[0], vec2_scale(vec2_sub(contacts_in[1], contacts_in[0]), t));
        contacts_out[num_out] = contact;
        num_out++;
    }
    return num_out;
}

bool collision_polygon_polygon(Body *a, Body *b, Collision_Info info[], unsigned int *n_collisions)
{

    unsigned int a_index_reference_edge, b_index_reference_edge;
    vec2 a_support_point, b_support_point;
    float sep_ab = collision_find_minimum_separation((Polygon *)a->shape, (Polygon *)b->shape, &a_index_reference_edge, &a_support_point);
    float sep_ba = collision_find_minimum_separation((Polygon *)b->shape, (Polygon *)a->shape, &b_index_reference_edge, &b_support_point);

    if (sep_ab >= 0)
    {
        return false;
    }
    if (sep_ba >= 0)
    {
        return false;
    }

    Polygon *reference_shape;
    Polygon *incident_shape;
    unsigned int index_reference_edge;
    if (sep_ab > sep_ba)
    {
        reference_shape = (Polygon *)a->shape;
        incident_shape = (Polygon *)b->shape;
        index_reference_edge = a_index_reference_edge;
    }
    else
    {
        reference_shape = (Polygon *)b->shape;
        incident_shape = (Polygon *)a->shape;
        index_reference_edge = b_index_reference_edge;
    }

    vec2 reference_edge = polygon_edge_at(reference_shape, index_reference_edge);

    unsigned int incident_index = polygon_find_incident_edge(incident_shape, vec2_normal(reference_edge));
    unsigned int incident_next_index = (incident_index + 1) % (incident_shape->n_vertices);
    vec2 v0 = incident_shape->global_vertices[incident_index];
    vec2 v1 = incident_shape->global_vertices[incident_next_index];

    vec2 contact_points[2] = {v0, v1};
    vec2 clipped_points[2] = {v0, v1};

    for (unsigned int i = 0; i < reference_shape->n_vertices; i++)
    {
        if (i == index_reference_edge)
            continue;

        vec2 c0 = reference_shape->global_vertices[i];
        vec2 c1 = reference_shape->global_vertices[(i + 1) % reference_shape->n_vertices];

        int num_clipped = polygon_clip_segment_to_line(reference_shape, contact_points, clipped_points, &c0, &c1);
        if (num_clipped < 2)
        {
            break;
        }

        contact_points[0] = clipped_points[0];
        contact_points[1] = clipped_points[1];
    }

    vec2 *vref = &reference_shape->global_vertices[index_reference_edge];

    for (unsigned int i = 0; i < 2; i++)
    {
        vec2 vclip = clipped_points[i];
        float separation = vec2_dot(vec2_sub(vclip, *vref), vec2_normal(reference_edge));
        if (separation <= 0)
        {
            Collision_Info *contact = &info[*n_collisions];
            contact->a = a;
            contact->b = b;
            contact->normal = vec2_normal(reference_edge);
            contact->start = vclip;
            contact->end = vec2_add(vclip, vec2_scale(contact->normal, -1.0 * separation));
            if (sep_ba >= sep_ab)
            {
                vec2 temp_start = contact->start;
                vec2 temp_end = contact->end;
                contact->start = temp_end;
                contact->end = temp_start;
                contact->normal = vec2_scale(contact->normal, -1.0);
            }

            (*n_collisions)++;
        }
    }

    return true;
}

bool collision_polygon_circle(Body *a, Body *b, Collision_Info info[], unsigned int *n_collisions)
{
    Collision_Info *contact = &info[*n_collisions];

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
                contact->a = a;
                contact->b = b;
                contact->depth = c->radius - vec2_norm(v1);
                contact->normal = vec2_unitvector(v1);
                contact->start = vec2_add(b->position, vec2_scale(contact->normal, -1.0 * c->radius));
                contact->end = vec2_add(contact->start, vec2_scale(contact->normal, contact->depth));
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
                    contact->a = a;
                    contact->b = b;
                    contact->depth = c->radius - vec2_norm(v1);
                    contact->normal = vec2_unitvector(v1);
                    contact->start = vec2_add(b->position, vec2_scale(contact->normal, -1.0 * c->radius));
                    contact->end = vec2_add(contact->start, vec2_scale(contact->normal, contact->depth));
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
                    contact->a = a;
                    contact->b = b;
                    contact->depth = c->radius - distance_circle_edge;
                    contact->normal = vec2_normal(vec2_sub(min_next_vertex, min_current_vertex));
                    contact->start = vec2_add(b->position, vec2_scale(contact->normal, -1.0 * c->radius));
                    contact->end = vec2_add(contact->start, vec2_scale(contact->normal, contact->depth));
                }
            }
        }
    }
    else
    {
        contact->a = a;
        contact->b = b;
        contact->depth = c->radius - distance_circle_edge;
        contact->normal = vec2_normal(vec2_sub(min_next_vertex, min_current_vertex));
        contact->start = vec2_add(b->position, vec2_scale(contact->normal, -1.0 * c->radius));
        contact->end = vec2_add(contact->start, vec2_scale(contact->normal, contact->depth));
    }

    (*n_collisions)++;

    return true;
}

#endif