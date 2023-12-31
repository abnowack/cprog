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
    Vec2 local_vertices[MAX_VERTICES];
    Vec2 global_vertices[MAX_VERTICES];
    unsigned int n_vertices;
} Polygon;

Circle circle_create(float radius)
{
    Circle c;
    c.radius = radius;
    return c;
}

Polygon box_create(float width, float height)
{
    Polygon b;

    b.n_vertices = 4;
    b.local_vertices[0] = (Vec2){-width / 2.0, -height / 2.0};
    b.local_vertices[1] = (Vec2){width / 2.0, -height / 2.0};
    b.local_vertices[2] = (Vec2){width / 2.0, height / 2.0};
    b.local_vertices[3] = (Vec2){-width / 2.0, height / 2.0};
    b.global_vertices[0] = (Vec2){-width / 2.0, -height / 2.0};
    b.global_vertices[1] = (Vec2){width / 2.0, -height / 2.0};
    b.global_vertices[2] = (Vec2){width / 2.0, height / 2.0};
    b.global_vertices[3] = (Vec2){-width / 2.0, height / 2.0};

    return b;
}

Polygon polygon_create(Vec2 *vertices, unsigned int n_vertices)
{
    Polygon p;
    p.n_vertices = n_vertices;
    for (unsigned int i = 0; i < n_vertices; i++)
    {
        p.local_vertices[i] = vertices[i];
        p.global_vertices[i] = vertices[i];
    }
    return p;
}

void shape_update_vertices(float theta, Vec2 position, ShapeType shape_type, void *shape)
{
    if (shape_type == CIRCLE)
        return;
    else if (shape_type == BOX || shape_type == POLYGON)
    {
        Polygon *p = (Polygon*)shape;
        for (unsigned int i = 0; i < p->n_vertices; i++)
        {
            p->global_vertices[i] = vec2_rotate_rad(p->local_vertices[i], theta);
            p->global_vertices[i] = vec2_add(p->global_vertices[i], position);
        }
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
        Polygon *b = (Polygon *)shape;
        float width = b->local_vertices[1].x - b->local_vertices[0].x;
        float height = b->local_vertices[2].y - b->local_vertices[1].y;
        inertia = (1.0 / 12.0) * (width * width + height * height);
        break;
    case POLYGON:
        inertia = 500;
        break;
    }

    return inertia;
}

Vec2 polygon_edge_at(Polygon *p, unsigned int index)
{
    unsigned int next_index = (index + 1) % (p->n_vertices);
    return vec2_sub(p->global_vertices[next_index], p->global_vertices[index]);
}

#endif