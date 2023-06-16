#ifndef BODY_H
#define BODY_H

#include "vec2.h"
#include "shape.h"

typedef struct {
    vec2 position;
    vec2 velocity;
    vec2 acceleration;
    float mass;
    float inv_mass;
    vec2 force;

    Shape *shape;
} Body;

void body_clear_force(Body*);

Body body_create(Shape *shape, float x_pos, float y_pos, float mass)
{
    Body b;
    b.position = (vec2){x_pos, y_pos};
    b.velocity = (vec2){0, 0};
    b.acceleration = (vec2){0, 0};
    b.mass = mass;
    if (b.mass != 0.0)
    {
        b.inv_mass = 1.0 / b.mass;
    }
    b.force = (vec2){0, 0};

    b.shape = shape;

    return b;
}

void body_integrate(Body *b, float delta_time)
{
    b->acceleration = vec2_scale(b->force, b->inv_mass);

    vec2 dv = vec2_scale(b->acceleration, delta_time);
    b->velocity = vec2_add(b->velocity, dv);
    vec2 dx = vec2_scale(b->velocity, delta_time);
    b->position = vec2_add(b->position, dx);

    body_clear_force(b);
}

void body_add_force(Body *b, vec2 force)
{
    b->force = vec2_add(b->force, force);
}

void body_clear_force(Body *b)
{
    b->force = (vec2){0, 0};
}

#endif