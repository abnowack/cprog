#ifndef CONSTRAINT_H
#define CONSTRAINT_H

#include "body.h"
#include "vec2.h"
#include "vecN.h"
#include "MatMN.h"

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
    vec2 a_local_anchor;
    vec2 b_local_anchor;
    MatMN jacobian;
    VecN cached_lambda;
    float bias;
} JointConstraint;

void joint_constraint_create(JointConstraint *jc, Body *a, Body *b, vec2 anchor)
{
    jc->a = a;
    jc->b = b;
    jc->a_local_anchor = body_global_to_local_space(a, anchor);
    jc->b_local_anchor = body_global_to_local_space(b, anchor);
    jc->jacobian = matmn_create(1, 6);
    jc->cached_lambda = vecN_create(1);
    jc->bias = 0;
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
    VecN v = vecN_create(6);
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
    vec2 pa = body_local_to_global_space(c->a, c->a_local_anchor);
    vec2 pb = body_local_to_global_space(c->b, c->b_local_anchor);

    vec2 j1 = vec2_scale(vec2_sub(pa, pb), 2.0);
    MATMN_AT(c->jacobian, 0, 0) = j1.x;
    MATMN_AT(c->jacobian, 0, 1) = j1.y;

    vec2 ra = vec2_sub(pa, c->a->position);
    vec2 rb = vec2_sub(pb, c->b->position);

    float j2 = 2.0 * vec2_cross(ra, vec2_sub(pa, pb));
    MATMN_AT(c->jacobian, 0, 2) = j2;

    vec2 j3 = vec2_scale(vec2_sub(pb, pa), 2.0);
    MATMN_AT(c->jacobian, 0, 3) = j3.x;
    MATMN_AT(c->jacobian, 0, 4) = j3.y;

    float j4 = 2.0 * vec2_cross(rb, vec2_sub(pb, pa));
    MATMN_AT(c->jacobian, 0, 5) = j4;

    MatMN jacobian_T = matmn_transpose(&(c->jacobian));

    VecN impulses = matmn_vec_mul(&jacobian_T, &c->cached_lambda);

    body_apply_impulse_linear(c->a, (vec2){impulses.data[0], impulses.data[1]});
    body_apply_impulse_angular(c->a, impulses.data[2]);

    body_apply_impulse_linear(c->b, (vec2){impulses.data[3], impulses.data[4]});
    body_apply_impulse_angular(c->b, impulses.data[5]);

    float beta = 0.1;
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
    VecN rhs = vecN_scale(&aa_1, -1.0);
    rhs.data[0] -= c->bias;

    VecN lambda = matmn_solve_gauss_seidel(&lhs, &rhs);
    c->cached_lambda = vecN_add(&c->cached_lambda, &lambda);

    VecN impulses = matmn_vec_mul(&jacobian_T, &lambda);

    body_apply_impulse_linear(c->a, (vec2){impulses.data[0], impulses.data[1]});
    body_apply_impulse_angular(c->a, impulses.data[2]);

    body_apply_impulse_linear(c->b, (vec2){impulses.data[3], impulses.data[4]});
    body_apply_impulse_angular(c->b, impulses.data[5]);
}

#endif