#ifndef WORLD_H
#define WORLD_H

#include "body.h"
#include "collision.h"
#include "constraint.h"

#define MAX_BODIES 40
#define MAX_FORCES 20
#define MAX_TORQUES 20
#define MAX_CONSTRAINTS 100

#define PIXELS_PER_METER 50

typedef struct
{
    float G;

    Body b[MAX_BODIES];
    unsigned int n_bodies;

    vec2 forces[MAX_FORCES];
    unsigned int n_forces;

    float torques[MAX_TORQUES];
    unsigned int n_torques;

    JointConstraint *joint_constraints[MAX_CONSTRAINTS];
    unsigned int n_joint_constraints;
} World;

void world_create(World *w, float gravity)
{
    w->G = -gravity;
    w->n_bodies = 0;
    w->n_forces = 0;
    w->n_torques = 0;
    w->n_joint_constraints = 0;
}

void world_add_body(World *w, Body *b)
{
    w->b[w->n_bodies] = *b;
    shape_update_vertices(w->b[w->n_bodies].theta, w->b[w->n_bodies].position, w->b[w->n_bodies].shape_type, w->b[w->n_bodies].shape);
    w->n_bodies++;
}

void world_add_force(World *w, vec2 force)
{
    w->forces[w->n_forces] = force;
    w->n_forces++;
}

void world_add_torque(World *w, float torque)
{
    w->torques[w->n_torques] = torque;
    w->n_torques++;
}

void world_add_joint_constraints(World *w, JointConstraint *jc)
{
    w->joint_constraints[w->n_joint_constraints] = jc;
    w->n_joint_constraints++;
}

void world_update(World *w, float delta_time)
{
    PenetrationConstraint pc_list[MAX_CONSTRAINTS];
    unsigned int n_penetration_constraints = 0;

    for (unsigned int i = 0; i < w->n_bodies; i++)
    {
        vec2 weight = (vec2){0.0, w->b[i].mass * w->G * PIXELS_PER_METER};
        body_add_force(&w->b[i], weight);

        for (unsigned int j = 0; j < w->n_forces; j++)
        {
            body_add_force(&w->b[i], w->forces[j]);
        }

        for (unsigned int j = 0; j < w->n_torques; j++)
        {
            body_add_torque(&w->b[i], w->torques[j]);
        }
    }

    for (unsigned int i = 0; i < w->n_bodies; i++)
    {
        body_integrate_forces(&w->b[i], delta_time);
    }

    // collision detection
    for (unsigned int i = 0; i < w->n_bodies; i++)
    {
        for (unsigned int j = i + 1; j < w->n_bodies; j++)
        {
            Body *a = &(w->b[i]);
            Body *b = &(w->b[j]);
            Collision_Info info[10];
            unsigned int n_collisions = 0;

            if (collision(a, b, info, &n_collisions))
            {
                for (unsigned int coll_iter = 0; coll_iter < n_collisions; coll_iter++)
                {
                    gfx_draw_filled_circle(info[coll_iter].start.x, info[coll_iter].start.y, 5, (uint8_t[3]){255, 0, 0});

                    penetration_constraint_create(&pc_list[n_penetration_constraints], info[coll_iter].a, info[coll_iter].b, info[coll_iter].start, info[coll_iter].end, info[coll_iter].normal);
                    n_penetration_constraints++;
                }
            }
        }
    }

    for (unsigned int i = 0; i < w->n_joint_constraints; i++)
    {
        joint_constraint_pre_solve(w->joint_constraints[i], delta_time);
    }

    for (unsigned int i = 0; i < n_penetration_constraints; i++)
    {
        penetration_constraint_pre_solve(&pc_list[i], delta_time);
    }

    for (unsigned int iter = 0; iter < 10; iter++)
    {
        for (unsigned int i = 0; i < w->n_joint_constraints; i++)
        {
            joint_constraint_solve(w->joint_constraints[i]);
        }

        for (unsigned int i = 0; i < n_penetration_constraints; i++)
        {
            penetration_constraint_solve(&pc_list[i]);
        }
    }

    for (unsigned int i = 0; i < w->n_joint_constraints; i++)
    {
        joint_constraint_post_solve(w->joint_constraints[i]);
    }

    for (unsigned int i = 0; i < n_penetration_constraints; i++)
    {
        penetration_constraint_post_solve(&pc_list[i]);
    }

    for (unsigned int i = 0; i < w->n_bodies; i++)
    {
        body_integrate_velocities(&w->b[i], delta_time);
    }
}

#endif