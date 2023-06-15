#ifndef PARTICLE_H
#define PARTICLE_H

#include "vec2.h"

typedef struct {
    vec2 position;
    vec2 velocity;
    vec2 acceleration;
    float mass;
} Particle;

Particle particle_create(float x_pos, float y_pos, float mass)
{
    Particle p;
    p.position = (vec2){x_pos, y_pos};
    p.velocity = (vec2){0, 0};
    p.acceleration = (vec2){0, 0};
    p.mass = mass;
    return p;
}

#endif