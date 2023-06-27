#ifndef WORLD_H
#define WORLD_H

#include "body.h"
#include "collision.h"
#include "constraint.h"
#include "linked_list.h"
#include "mem.h"

#define MAX_CONSTRAINTS 100

#define PIXELS_PER_METER 50

typedef struct
{
    float G;
    List bodies;
    List joint_constraints;
} World;

void world_create(World *w, float gravity)
{
    w->G = -gravity;
    w->bodies = list_create_empty();
    w->joint_constraints = list_create_empty();
}

void world_update(World *w, float delta_time)
{
    List pc_list = list_create_empty();
    // PenetrationConstraint pc_list[MAX_CONSTRAINTS];
    // unsigned int n_penetration_constraints = 0;

    for (Node *n = w->bodies.start, *next; n != NULL; n = next)
    {
        Body *b = (Body *)n->data;
        Vec2 weight = (Vec2){0.0, b->mass * w->G * PIXELS_PER_METER};
        body_add_force(b, weight);
        next = n->next;
    }

    for (Node *n = w->bodies.start, *next; n != NULL; n = next)
    {
        Body *b = (Body *)n->data;
        body_integrate_forces(b, delta_time);
        next = n->next;
    }

    // collision detection
    for (Node *n_i = w->bodies.start, *next_i; n_i != NULL; n_i = next_i)
    {
        for (Node *n_j = n_i->next, *next_j; n_j != NULL; n_j = next_j)
        {
            Body *a = (Body *)n_i->data;
            Body *b = (Body *)n_j->data;
            Collision_Info info[10];
            unsigned int n_collisions = 0;

            if (collision(a, b, info, &n_collisions))
            {
                for (unsigned int coll_iter = 0; coll_iter < n_collisions; coll_iter++)
                {
                    gfx_draw_filled_circle(info[coll_iter].start.x, info[coll_iter].start.y, 5, (uint8_t[3]){255, 0, 0});

                    PenetrationConstraint *pc = (PenetrationConstraint *)mem_malloc(sizeof(PenetrationConstraint));
                    penetration_constraint_create(pc, info[coll_iter].a, info[coll_iter].b, info[coll_iter].start, info[coll_iter].end, info[coll_iter].normal);
                    List_push(&pc_list, pc); // calling malloc here
                }
            }
            next_j = n_j->next;
        }
        next_i = n_i->next;
    }

    for (Node *n = w->joint_constraints.start, *next; n; n = next)
    {
        joint_constraint_pre_solve((JointConstraint *)n->data, delta_time);
        next = n->next;
    }

    // for (unsigned int i = 0; i < n_penetration_constraints; i++)
    for (Node *n = pc_list.start, *next; n; n = next)
    {
        penetration_constraint_pre_solve((PenetrationConstraint *)n->data, delta_time);
        next = n->next;
    }

    for (unsigned int iter = 0; iter < 10; iter++)
    {
        for (Node *n = w->joint_constraints.start, *next; n; n = next)
        {
            joint_constraint_solve((JointConstraint *)n->data);
            next = n->next;
        }

        for (Node *n = pc_list.start, *next; n; n = next)
        {
            penetration_constraint_solve((PenetrationConstraint *)n->data);
            next = n->next;
        }
    }

    for (Node *n = w->joint_constraints.start, *next; n; n = next)
    {
        joint_constraint_post_solve((JointConstraint *)n->data);
        next = n->next;
    }

    for (Node *n = pc_list.start, *next; n; n = next)
    {
        penetration_constraint_post_solve((PenetrationConstraint *)n->data);
        next = n->next;
    }

    for (Node *n = w->bodies.start, *next; n != NULL; n = next)
    {
        Body *b = (Body *)n->data;
        body_integrate_velocities(b, delta_time);
        next = n->next;
    }

    list_destroy(&pc_list);
}

#endif