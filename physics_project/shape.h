#ifndef SHAPE_H
#define SHAPE_H

#include "vec2.h"

#define MAX_VERTICES 20

typedef enum
{
    BOX,
    POLYGON,
    CIRCLE
} ShapeType;

typedef struct
{
    float radius;
} Circle;

typedef struct
{
    vec2 local_vertices[MAX_VERTICES];
    vec2 global_vertices[MAX_VERTICES];
    unsigned int n_vertices;
} Polygon;

typedef struct
{
    float width;
    float height;

    vec2 local_vertices[4];
    vec2 global_vertices[4];
    unsigned int n_vertices;
} Box;

Circle circle_create(float radius)
{
    Circle c;
    c.radius = radius;
    return c;
}

Box box_create(float width, float height)
{
    Box b;
    b.width = width;
    b.height = height;

    b.n_vertices = 4;
    b.local_vertices[0] = (vec2){-b.width / 2.0, -height / 2.0};
    b.local_vertices[1] = (vec2){b.width / 2.0, -height / 2.0};
    b.local_vertices[2] = (vec2){b.width / 2.0, height / 2.0};
    b.local_vertices[3] = (vec2){-b.width / 2.0, height / 2.0};
    b.global_vertices[0] = (vec2){-b.width / 2.0, -height / 2.0};
    b.global_vertices[1] = (vec2){b.width / 2.0, -height / 2.0};
    b.global_vertices[2] = (vec2){b.width / 2.0, height / 2.0};
    b.global_vertices[3] = (vec2){-b.width / 2.0, height / 2.0};

    return b;
}

Polygon polygon_create(vec2 *vertices, unsigned int n_vertices)
{
    Polygon p;
    p.n_vertices = n_vertices;
    for (unsigned int i = 0; i < n_vertices; i++)
    {
        p.local_vertices[i] = vertices[i];
    }
    return p;
}

void polygon_update_vertices(float theta, vec2 position, Polygon *p)
{
    for (unsigned int i = 0; i < p->n_vertices; i++)
    {
        p->global_vertices[i] = vec2_rotate_rad(p->local_vertices[i], theta);
        p->global_vertices[i] = vec2_add(p->global_vertices[i], position);
    }
}

void box_update_vertices(float theta, vec2 position, Box *b)
{
    for (unsigned int i = 0; i < b->n_vertices; i++)
    {
        b->global_vertices[i] = vec2_rotate_rad(b->local_vertices[i], theta);
        b->global_vertices[i] = vec2_add(b->global_vertices[i], position);
    }
}

float shape_moment_of_inertia(ShapeType shape_type, void *shape)
{
    float inertia = 0;
    switch (shape_type)
    {
    case CIRCLE:;
        Circle *c = (Circle *)shape;
        inertia = 0.5 * c->radius * c->radius;
        break;
    case BOX:;
        Box *b = (Box *)shape;
        inertia = (1.0 / 12.0) * (b->width * b->width + b->height * b->height);
        break;
    case POLYGON:
        break;
    }

    return inertia;
}

#endif