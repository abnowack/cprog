#ifndef PARTICLE_H
#define PARTICLE_H

#include "vec2.h"

typedef struct {
    vec2 position;
    vec2 velocity;
    vec2 acceleration;
    float mass;
    float inv_mass;
    int radius;
    vec2 force;
    bool frozen;
} Particle;

void particle_clear_force(Particle*);

Particle particle_create(float x_pos, float y_pos, float mass)
{
    Particle p;
    p.position = (vec2){x_pos, y_pos};
    p.velocity = (vec2){0, 0};
    p.acceleration = (vec2){0, 0};
    p.mass = mass;
    p.frozen = false;
    if (p.mass != 0.0)
    {
        p.inv_mass = 1.0 / p.mass;
    }
    p.radius = 1;
    p.force = (vec2){0, 0};
    return p;
}

void particle_integrate(Particle *p, float delta_time)
{
    p->acceleration = vec2_scale(p->force, p->inv_mass);

    vec2 dv = vec2_scale(p->acceleration, delta_time);
    p->velocity = vec2_add(p->velocity, dv);
    vec2 dx = vec2_scale(p->velocity, delta_time);
    p->position = vec2_add(p->position, dx);

    particle_clear_force(p);
}

void particle_add_force(Particle *p, vec2 force)
{
    p->force = vec2_add(p->force, force);
}

void particle_clear_force(Particle *p)
{
    p->force = (vec2){0, 0};
}

#endif