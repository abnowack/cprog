#ifndef BODY_H
#define BODY_H

#include "vec2.h"
#include "shape.h"

typedef struct
{
    Vec2 position;
    Vec2 velocity;
    Vec2 acceleration;

    float theta;
    float omega;
    float alpha;

    float mass;
    float inv_mass;
    Vec2 force;

    float inertia;
    float inv_inertia;
    float torque;

    ShapeType shape_type;
    void *shape;

    float restitution;
    float friction;
} Body;

void body_clear_force(Body *);
void body_clear_torque(Body *);

Body body_create(ShapeType shape_type, void *shape, float x_pos, float y_pos, float mass)
{
    Body b;

    b.position = (Vec2){x_pos, y_pos};
    b.velocity = (Vec2){0, 0};
    b.acceleration = (Vec2){0, 0};

    b.mass = mass;
    if (b.mass != 0.0)
    {
        b.inv_mass = 1.0 / b.mass;
    }
    else
    {
        b.inv_mass = 0.0;
    }
    b.force = (Vec2){0, 0};

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

    b.restitution = 1.0;
    b.friction = 0.0;

    shape_update_vertices(b.theta, b.position, b.shape_type, b.shape);

    return b;
}

Vec2 body_local_to_global_space(Body *b, Vec2 point)
{
    Vec2 rotated = vec2_rotate_rad(point, b->theta);
    return vec2_add(rotated, b->position);
}

Vec2 body_global_to_local_space(Body *b, Vec2 point)
{
    float translated_dx = point.x - b->position.x;
    float translated_dy = point.y - b->position.y;
    float rotated_x = cosf(-b->theta) * translated_dx - sinf(-b->theta) * translated_dy;
    float rotated_y = cosf(-b->theta) * translated_dy + sinf(-b->theta) * translated_dx;

    return (Vec2){rotated_x, rotated_y};
}

void body_integrate_forces(Body *b, float delta_time)
{
    if (b->inv_mass == 0)
        return;

    b->acceleration = vec2_scale(b->force, b->inv_mass);
    Vec2 dv = vec2_scale(b->acceleration, delta_time);
    b->velocity = vec2_add(b->velocity, dv);

    b->alpha = b->torque * b->inv_inertia;
    b->omega += b->alpha * delta_time;

    body_clear_force(b);
    body_clear_torque(b);
}

void body_integrate_velocities(Body *b, float delta_time)
{
    if (b->inv_mass == 0)
        return;

    Vec2 dx = vec2_scale(b->velocity, delta_time);
    b->position = vec2_add(b->position, dx);

    b->theta += b->omega * delta_time;
    b->theta = (b->theta + 2.0 * M_PI);
    b->theta = fmodf(b->theta, 2.0 * M_PI);

    shape_update_vertices(b->theta, b->position, b->shape_type, b->shape);
}

void body_add_force(Body *b, Vec2 force)
{
    b->force = vec2_add(b->force, force);
}

void body_clear_force(Body *b)
{
    b->force = (Vec2){0, 0};
}

void body_add_torque(Body *b, float torque)
{
    b->torque += torque;
}

void body_clear_torque(Body *b)
{
    b->torque = 0.0f;
}

void body_apply_impulse_linear(Body *b, Vec2 j)
{
    if (b->inv_mass == 0)
        return;

    Vec2 dv = vec2_scale(j, b->inv_mass);
    b->velocity = vec2_add(b->velocity, dv);
}

void body_apply_impulse_angular(Body *b, float j)
{
    if (b->inv_mass == 0)
        return;

    b->omega += j * b->inv_inertia;
}

void body_apply_impulse_at_r(Body *b, Vec2 j, Vec2 r)
{
    if (b->inv_mass == 0)
        return;

    b->velocity = vec2_add(b->velocity, vec2_scale(j, b->inv_mass));
    b->omega += vec2_cross(r, j) * b->inv_inertia;
}

#endif