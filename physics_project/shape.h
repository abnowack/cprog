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
    ShapeType type;
} Shape;

typedef struct
{
    Shape shape;
    float radius;
} Circle;

typedef struct
{
    Shape shape;
    float width;
    float height;
} Box;

typedef struct
{
    Shape shape;
    vec2 vertices[MAX_VERTICES];
    unsigned int n_vertices;
} Polygon;

Circle circle_create(float radius)
{
    Circle c;
    c.shape.type = CIRCLE;
    c.radius = radius;
    return c;
}

Box box_create(float width, float height)
{
    Box b;
    b.shape.type = BOX;
    b.width = width;
    b.height = height;
    return b;
}

Polygon polygon_create(vec2 *vertices, unsigned int n_vertices)
{
    Polygon p;
    p.shape.type = POLYGON;
    p.n_vertices = n_vertices;
    for (unsigned int i = 0; i < n_vertices; i++)
    {
        p.vertices[i] = vertices[i];
    }
    return p;
}

float shape_moment_of_inertia(Shape *s)
{
    switch (s->type)
    {
        case CIRCLE:
            break;
        case BOX:
            break;
        case POLYGON:
            break;
    }
}

#endif