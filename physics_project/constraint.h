#ifndef CONSTRAINT_H
#define CONSTRAINT_H

#include "body.h"
#include "vec2.h"
#include "vecn.h"
#include "matmn.h"

#define MAX(x, y) (((x) > (y)) ? (x) : (y))
#define MIN(x, y) (((x) < (y)) ? (x) : (y))

void matmn_print(MatMN *a)
{
    for (unsigned int i = 0; i < a->m; i++)
    {
        for (unsigned int j = 0; j < a->n; j++)
        {
            printf("%f ", MATMN_AT(*a, i, j));
        }
        printf("\n");
    }
}

void vecn_print(VecN *a)
{
    for (unsigned int i = 0; i < a->n; i++)
    {
        printf("%f ", a->data[i]);
    }
    printf("\n");
}

typedef struct
{
    Body *a;
    Body *b;
    Vec2 a_local_anchor;
    Vec2 b_local_anchor;
    MatMN jacobian;
    VecN cached_lambda;
    float bias;
} JointConstraint;

typedef struct
{
    Body *a;
    Body *b;
    Vec2 a_collision;
    Vec2 b_collision;
    MatMN jacobian;
    VecN cached_lambda;
    float bias;
    Vec2 normal;
    float friction;
} PenetrationConstraint;


void joint_constraint_create(JointConstraint *jc, Body *a, Body *b, Vec2 anchor)
{
    jc->a = a;
    jc->b = b;
    jc->a_local_anchor = body_global_to_local_space(a, anchor);
    jc->b_local_anchor = body_global_to_local_space(b, anchor);
    jc->jacobian = matmn_create(1, 6);
    jc->cached_lambda = vecn_create(1);
    jc->bias = 0;
}

void penetration_constraint_create(PenetrationConstraint *pc, Body *a, Body *b, Vec2 a_collision, Vec2 b_collision, Vec2 normal)
{
    pc->a = a;
    pc->b = b;
    pc->a_collision = body_global_to_local_space(a, a_collision);
    pc->b_collision = body_global_to_local_space(b, b_collision);
    pc->normal = body_global_to_local_space(a, normal);
    pc->jacobian = matmn_create(2, 6);
    pc->cached_lambda = vecn_create(2);
    pc->bias = 0;
    pc->friction = 0.0;
}

MatMN constraint_get_inv_m(Body *a, Body *b)
{
    MatMN invM = matmn_create(6, 6);
    MATMN_AT(invM, 0, 0) = a->inv_mass;
    MATMN_AT(invM, 1, 1) = a->inv_mass;
    MATMN_AT(invM, 2, 2) = a->inv_inertia;
    MATMN_AT(invM, 3, 3) = b->inv_mass;
    MATMN_AT(invM, 4, 4) = b->inv_mass;
    MATMN_AT(invM, 5, 5) = b->inv_inertia;
    return invM;
}

VecN constraint_velocities(Body *a, Body *b)
{
    VecN v = vecn_create(6);
    v.data[0] = a->velocity.x;
    v.data[1] = a->velocity.y;
    v.data[2] = a->omega;
    v.data[3] = b->velocity.x;
    v.data[4] = b->velocity.y;
    v.data[5] = b->omega;
    return v;
}

void joint_constraint_pre_solve(JointConstraint *c, float delta_time)
{
    Vec2 pa = body_local_to_global_space(c->a, c->a_local_anchor);
    Vec2 pb = body_local_to_global_space(c->b, c->b_local_anchor);

    Vec2 j1 = vec2_scale(vec2_sub(pa, pb), 2.0);
    MATMN_AT(c->jacobian, 0, 0) = j1.x;
    MATMN_AT(c->jacobian, 0, 1) = j1.y;

    Vec2 ra = vec2_sub(pa, c->a->position);
    Vec2 rb = vec2_sub(pb, c->b->position);

    float j2 = 2.0 * vec2_cross(ra, vec2_sub(pa, pb));
    MATMN_AT(c->jacobian, 0, 2) = j2;

    Vec2 j3 = vec2_scale(vec2_sub(pb, pa), 2.0);
    MATMN_AT(c->jacobian, 0, 3) = j3.x;
    MATMN_AT(c->jacobian, 0, 4) = j3.y;

    float j4 = 2.0 * vec2_cross(rb, vec2_sub(pb, pa));
    MATMN_AT(c->jacobian, 0, 5) = j4;

    MatMN jacobian_T = matmn_transpose(&(c->jacobian));

    VecN impulses = matmn_vec_mul(&jacobian_T, &c->cached_lambda);

    body_apply_impulse_linear(c->a, (Vec2){impulses.data[0], impulses.data[1]});
    body_apply_impulse_angular(c->a, impulses.data[2]);

    body_apply_impulse_linear(c->b, (Vec2){impulses.data[3], impulses.data[4]});
    body_apply_impulse_angular(c->b, impulses.data[5]);

    float beta = 0.2;
    float C = vec2_dot(vec2_sub(pb, pa), vec2_sub(pb, pa));
    C = MAX(0.0, C - 0.01f);
    c->bias = beta / delta_time * C;
}

void joint_constraint_post_solve(JointConstraint *c)
{
}

void joint_constraint_solve(JointConstraint *c)
{
    VecN v = constraint_velocities(c->a, c->b);
    MatMN invM = constraint_get_inv_m(c->a, c->b);

    MatMN jacobian_T = matmn_transpose(&(c->jacobian));

    MatMN aa_2 = matmn_mat_mul(&(c->jacobian), &invM);
    MatMN lhs = matmn_mat_mul(&aa_2, &jacobian_T);
    VecN aa_1 = matmn_vec_mul(&(c->jacobian), &v);
    VecN rhs = vecn_scale(&aa_1, -1.0);
    rhs.data[0] -= c->bias;

    VecN lambda = matmn_solve_gauss_seidel(&lhs, &rhs);
    c->cached_lambda = vecn_add(&c->cached_lambda, &lambda);

    VecN impulses = matmn_vec_mul(&jacobian_T, &lambda);

    body_apply_impulse_linear(c->a, (Vec2){impulses.data[0], impulses.data[1]});
    body_apply_impulse_angular(c->a, impulses.data[2]);

    body_apply_impulse_linear(c->b, (Vec2){impulses.data[3], impulses.data[4]});
    body_apply_impulse_angular(c->b, impulses.data[5]);
}

void penetration_constraint_pre_solve(PenetrationConstraint *c, float delta_time)
{
    Vec2 pa = body_local_to_global_space(c->a, c->a_collision);
    Vec2 pb = body_local_to_global_space(c->b, c->b_collision);
    Vec2 n = body_local_to_global_space(c->a, c->normal);

    Vec2 ra = vec2_sub(pa, c->a->position);
    Vec2 rb = vec2_sub(pb, c->b->position);

    Vec2 j1 = vec2_scale(n, -1.0);
    MATMN_AT(c->jacobian, 0, 0) = j1.x;
    MATMN_AT(c->jacobian, 0, 1) = j1.y;

    float j2 = vec2_cross(vec2_scale(ra, -1.0), n);
    MATMN_AT(c->jacobian, 0, 2) = j2;

    Vec2 j3 = n;
    MATMN_AT(c->jacobian, 0, 3) = j3.x;
    MATMN_AT(c->jacobian, 0, 4) = j3.y;

    float j4 = vec2_cross(rb, n);
    MATMN_AT(c->jacobian, 0, 5) = j4;

    c->friction = MAX(c->a->friction, c->b->friction);
    if (c->friction > 0.0)
    {
        Vec2 t = vec2_normal(n);
        MATMN_AT(c->jacobian, 1, 0) = -(t.x);
        MATMN_AT(c->jacobian, 1, 1) = -(t.y);
        MATMN_AT(c->jacobian, 1, 2) = -(vec2_cross(ra, t));

        MATMN_AT(c->jacobian, 1, 3) = t.x;
        MATMN_AT(c->jacobian, 1, 4) = t.y;
        MATMN_AT(c->jacobian, 1, 5) = vec2_cross(rb, t);
    }

    MatMN jacobian_T = matmn_transpose(&(c->jacobian));

    VecN impulses = matmn_vec_mul(&jacobian_T, &c->cached_lambda);

    body_apply_impulse_linear(c->a, (Vec2){impulses.data[0], impulses.data[1]});
    body_apply_impulse_angular(c->a, impulses.data[2]);

    body_apply_impulse_linear(c->b, (Vec2){impulses.data[3], impulses.data[4]});
    body_apply_impulse_angular(c->b, impulses.data[5]);

    float beta = 0.2;
    float C = vec2_dot(vec2_sub(pb, pa), vec2_scale(n, -1.0));
    C = MIN(0.0, C + 0.01f);
    
    Vec2 va = vec2_add(c->a->velocity, (Vec2){-(c->a->omega) * ra.y, c->a->omega * ra.x});
    Vec2 vb = vec2_add(c->b->velocity, (Vec2){-(c->b->omega) * rb.y, c->b->omega * rb.x});
    float vrel_dot_normal = vec2_dot(vec2_sub(va, vb), n);

    float e = MIN(c->a->restitution, c->b->restitution);
    c->bias = beta / delta_time * C + (e * vrel_dot_normal);
}

void penetration_constraint_post_solve(PenetrationConstraint *c)
{
}

void penetration_constraint_solve(PenetrationConstraint *c)
{
    VecN v = constraint_velocities(c->a, c->b);
    MatMN invM = constraint_get_inv_m(c->a, c->b);

    MatMN jacobian_T = matmn_transpose(&(c->jacobian));

    MatMN aa_2 = matmn_mat_mul(&(c->jacobian), &invM);
    MatMN lhs = matmn_mat_mul(&aa_2, &jacobian_T);
    VecN aa_1 = matmn_vec_mul(&(c->jacobian), &v);
    VecN rhs = vecn_scale(&aa_1, -1.0);
    rhs.data[0] -= c->bias;

    VecN lambda = matmn_solve_gauss_seidel(&lhs, &rhs);

    VecN old_lambda = vecn_copy(&c->cached_lambda);
    c->cached_lambda = vecn_add(&c->cached_lambda, &lambda);
    
    if (c->cached_lambda.data[0] < 0.0)
        c->cached_lambda.data[0] = 0.0;
    
    if (c->friction > 0)
    {
        float max_friction = c->cached_lambda.data[0] * c->friction;
        if (c->cached_lambda.data[1] < -max_friction)
        {
            c->cached_lambda.data[1] = -max_friction;
        } 
        else if (c->cached_lambda.data[1] > max_friction)
        {
            c->cached_lambda.data[1] = max_friction;
        }
    }
    
    lambda = vecn_sub(&c->cached_lambda, &old_lambda);

    VecN impulses = matmn_vec_mul(&jacobian_T, &lambda);

    body_apply_impulse_linear(c->a, (Vec2){impulses.data[0], impulses.data[1]});
    body_apply_impulse_angular(c->a, impulses.data[2]);

    body_apply_impulse_linear(c->b, (Vec2){impulses.data[3], impulses.data[4]});
    body_apply_impulse_angular(c->b, impulses.data[5]);
}

#endif