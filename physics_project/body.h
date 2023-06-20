#ifndef BODY_H
#define BODY_H

#include "vec2.h"
#include "shape.h"

typedef struct {
    vec2 position;
    vec2 velocity;
    vec2 acceleration;

    float theta;
    float omega;
    float alpha;

    float mass;
    float inv_mass;
    vec2 force;

    float inertia;
    float inv_inertia;
    float torque;

    ShapeType shape_type;
    void *shape;

    bool is_colliding;
    float restitution;
    float friction;
} Body;

void body_clear_force(Body*);
void body_clear_torque(Body*);

Body body_create(ShapeType shape_type, void *shape, float x_pos, float y_pos, float mass)
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
    else
    {
        b.inv_mass = 0.0;
    }
    b.force = (vec2){0, 0};

    b.theta = 0;
    b.omega = 0;
    b.alpha = 0;

    b.shape_type = shape_type;
    b.shape = shape;
    b.inertia = shape_moment_of_inertia(b.shape_type, b.shape) * b.mass;
    if (b.inertia != 0.0)
    {
        b.inv_inertia = 1.0 / b.inertia;
    }
    else
    {
        b.inv_inertia = 0.0;
    }
    b.torque = 0.0;

    b.is_colliding = false;
    b.restitution = 1.0;
    b.friction = 0.7;

    return b;
}

void body_integrate_position(Body *b, float delta_time)
{
    if (b->inv_mass == 0)
        return;

    b->acceleration = vec2_scale(b->force, b->inv_mass);

    vec2 dv = vec2_scale(b->acceleration, delta_time);
    b->velocity = vec2_add(b->velocity, dv);
    vec2 dx = vec2_scale(b->velocity, delta_time);
    b->position = vec2_add(b->position, dx);

    body_clear_force(b);
}

void body_integrate_angle(Body *b, float delta_time)
{
    if (b->inv_mass == 0)
        return;

    b->alpha = b->torque * b->inv_inertia;
    b->omega += b->alpha * delta_time;
    b->theta += b->omega * delta_time;

    b->theta = (b->theta + 2.0 * M_PI);
    b->theta = fmodf(b->theta, 2.0 * M_PI);

    body_clear_torque(b);
}

void body_add_force(Body *b, vec2 force)
{
    b->force = vec2_add(b->force, force);
}

void body_clear_force(Body *b)
{
    b->force = (vec2){0, 0};
}

void body_add_torque(Body *b, float torque)
{
    b->torque += torque;
}

void body_clear_torque(Body *b)
{
    b->torque = 0.0f;
}

void body_apply_impulse(Body *b, vec2 j)
{
    if (b->inv_mass == 0)
        return;
    
    b->velocity = vec2_add(b->velocity, vec2_scale(j, b->inv_mass));
}

void body_apply_impulse_at_r(Body *b, vec2 j, vec2 r)
{
    if (b->inv_mass == 0)
        return;
    
    b->velocity = vec2_add(b->velocity, vec2_scale(j, b->inv_mass));
    b->omega += vec2_cross(r, j) * b->inv_inertia;
}

void body_update(Body *b, float delta_time)
{
    body_integrate_position(b, delta_time);
    body_integrate_angle(b, delta_time);

    if (b->shape_type == BOX || b->shape_type == POLYGON)
    {
        polygon_update_vertices(b->theta, b->position, (Polygon*)(b->shape));
    }
}

#endif