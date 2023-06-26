#ifndef CONSTRAINT_H
#define CONSTRAINT_H

#include "body.h"
#include "vec2.h"
#include "matmn.h"
#include "util.h"

#define MAX(x, y) (((x) > (y)) ? (x) : (y))
#define MIN(x, y) (((x) < (y)) ? (x) : (y))

typedef struct
{
    Body *a;
    Body *b;
    Vec2 a_local_anchor;
    Vec2 b_local_anchor;
    MatMN jacobian;
    MatMN cached_lambda;
    float bias;
} JointConstraint;

typedef struct
{
    Body *a;
    Body *b;
    Vec2 a_collision;
    Vec2 b_collision;
    MatMN jacobian;
    MatMN cached_lambda;
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
    jc->cached_lambda = matmn_create(1, 1);
    jc->bias = 0;
}

void joint_constraint_destroy(JointConstraint *jc)
{
    matmn_destroy(&jc->jacobian);
    matmn_destroy(&jc->cached_lambda);
}

void penetration_constraint_create(PenetrationConstraint *pc, Body *a, Body *b, Vec2 a_collision, Vec2 b_collision, Vec2 normal)
{
    pc->a = a;
    pc->b = b;
    pc->a_collision = body_global_to_local_space(a, a_collision);
    pc->b_collision = body_global_to_local_space(b, b_collision);
    pc->normal = body_global_to_local_space(a, normal);
    pc->jacobian = matmn_create(2, 6);
    pc->cached_lambda = matmn_create(2, 1);
    pc->bias = 0;
    pc->friction = 0.0;
}

void penetration_constraint_destroy(PenetrationConstraint *pc)
{
    matmn_destroy(&pc->jacobian);
    matmn_destroy(&pc->cached_lambda);
}

void constraint_get_inv_m(Body *a, Body *b, MatMN *z)
{
    assert(z->m == 6);
    assert(z->n == 6);

    memset(z->data, 0, sizeof(z->data[0]) * z->m * z->n);

    MATMN_AT(*z, 0, 0) = a->inv_mass;
    MATMN_AT(*z, 1, 1) = a->inv_mass;
    MATMN_AT(*z, 2, 2) = a->inv_inertia;
    MATMN_AT(*z, 3, 3) = b->inv_mass;
    MATMN_AT(*z, 4, 4) = b->inv_mass;
    MATMN_AT(*z, 5, 5) = b->inv_inertia;
}

void constraint_velocities(Body *a, Body *b, MatMN *z)
{
    assert(z->m == 6);
    assert(z->n == 1);

    MATMN_AT(*z, 0, 0) = a->velocity.x;
    MATMN_AT(*z, 0, 1) = a->velocity.y;
    MATMN_AT(*z, 0, 2) = a->omega;
    MATMN_AT(*z, 0, 3) = b->velocity.x;
    MATMN_AT(*z, 0, 4) = b->velocity.y;
    MATMN_AT(*z, 0, 5) = b->omega;
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

    MatMN jacobian_T = matmn_create(c->jacobian.n, c->jacobian.m);
    matmn_transpose(&(c->jacobian), &jacobian_T);

    MatMN impulses = matmn_create(jacobian_T.m, c->cached_lambda.n);
    matmn_mul(&jacobian_T, &c->cached_lambda, &impulses);

    body_apply_impulse_linear(c->a, (Vec2){MATMN_AT(impulses, 0, 0), MATMN_AT(impulses, 1, 0)});
    body_apply_impulse_angular(c->a, MATMN_AT(impulses, 2, 0));

    body_apply_impulse_linear(c->b, (Vec2){MATMN_AT(impulses, 3, 0), MATMN_AT(impulses, 4, 0)});
    body_apply_impulse_angular(c->b, MATMN_AT(impulses, 5, 0));

    float beta = 0.2;
    float C = vec2_dot(vec2_sub(pb, pa), vec2_sub(pb, pa));
    C = MAX(0.0, C - 0.01f);
    c->bias = beta / delta_time * C;

    matmn_destroy(&jacobian_T);
    matmn_destroy(&impulses);
}

void joint_constraint_post_solve(JointConstraint *c)
{
}

void joint_constraint_solve(JointConstraint *c)
{
    MatMN v = matmn_create(6, 1);
    constraint_velocities(c->a, c->b, &v);
    MatMN invM = matmn_create(6, 6);
    constraint_get_inv_m(c->a, c->b, &invM);

    MatMN jacobian_T = matmn_create(c->jacobian.n, c->jacobian.m);
    matmn_transpose(&(c->jacobian), &jacobian_T);

    MatMN aa_2 = matmn_create(c->jacobian.m, invM.n);
    matmn_mul(&c->jacobian, &invM, &aa_2);
    MatMN lhs = matmn_create(aa_2.m, jacobian_T.n);
    matmn_mul(&aa_2, &jacobian_T, &lhs);
    MatMN aa_1 = matmn_create(c->jacobian.m, v.n);
    matmn_mul(&c->jacobian, &v, &aa_1);
    MatMN rhs = matmn_create_zero_like(&aa_1);
    matmn_scale(&aa_1, -1.0, &rhs);
    MATMN_AT(rhs, 0, 0) -= c->bias;

    MatMN lambda = matmn_create(1, 1);
    // printf("%d %d - %d %d - %d %d\n", lhs.m, lhs.n, rhs.m, rhs.n, lambda.m, lambda.n);
    matmn_solve_gauss_seidel(&lhs, &rhs, &lambda);
    matmn_add(&c->cached_lambda, &lambda, &c->cached_lambda);

    MatMN impulses = matmn_create(jacobian_T.m, lambda.n);
    matmn_mul(&jacobian_T, &lambda, &impulses);

    body_apply_impulse_linear(c->a, (Vec2){MATMN_AT(impulses, 0, 0), MATMN_AT(impulses, 1, 0)});
    body_apply_impulse_angular(c->a, MATMN_AT(impulses, 2, 0));

    body_apply_impulse_linear(c->b, (Vec2){MATMN_AT(impulses, 3, 0), MATMN_AT(impulses, 4, 0)});
    body_apply_impulse_angular(c->b, MATMN_AT(impulses, 5, 0));

    matmn_destroy(&impulses);
    matmn_destroy(&lambda);
    matmn_destroy(&rhs);
    matmn_destroy(&aa_1);
    matmn_destroy(&lhs);
    matmn_destroy(&aa_2);
    matmn_destroy(&v);
    matmn_destroy(&invM);
    matmn_destroy(&jacobian_T);
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

    MatMN jacobian_T = matmn_create(c->jacobian.n, c->jacobian.m);
    matmn_transpose(&(c->jacobian), &jacobian_T);

    MatMN impulses = matmn_create(jacobian_T.m, c->cached_lambda.n);
    // printf("%d %d - %d %d - %d %d\n", jacobian_T.m, jacobian_T.n, c->cached_lambda.m, c->cached_lambda.n, impulses.m, impulses.n);
    matmn_mul(&jacobian_T, &c->cached_lambda, &impulses);

    body_apply_impulse_linear(c->a, (Vec2){MATMN_AT(impulses, 0, 0), MATMN_AT(impulses, 1, 0)});
    body_apply_impulse_angular(c->a, MATMN_AT(impulses, 2, 0));

    body_apply_impulse_linear(c->b, (Vec2){MATMN_AT(impulses, 3, 0), MATMN_AT(impulses, 4, 0)});
    body_apply_impulse_angular(c->b, MATMN_AT(impulses, 5, 0));

    float beta = 0.2;
    float C = vec2_dot(vec2_sub(pb, pa), vec2_scale(n, -1.0));
    C = MIN(0.0, C + 0.01f);

    Vec2 va = vec2_add(c->a->velocity, (Vec2){-(c->a->omega) * ra.y, c->a->omega * ra.x});
    Vec2 vb = vec2_add(c->b->velocity, (Vec2){-(c->b->omega) * rb.y, c->b->omega * rb.x});
    float vrel_dot_normal = vec2_dot(vec2_sub(va, vb), n);

    float e = MIN(c->a->restitution, c->b->restitution);
    c->bias = beta / delta_time * C + (e * vrel_dot_normal);

    matmn_destroy(&jacobian_T);
    matmn_destroy(&impulses);
}

void penetration_constraint_post_solve(PenetrationConstraint *c)
{
}

void penetration_constraint_solve(PenetrationConstraint *c)
{
    MatMN v = matmn_create(6, 1);
    constraint_velocities(c->a, c->b, &v);
    MatMN invM = matmn_create(6, 6);
    constraint_get_inv_m(c->a, c->b, &invM);

    MatMN jacobian_T = matmn_create(c->jacobian.n, c->jacobian.m);
    matmn_transpose(&(c->jacobian), &jacobian_T);

    MatMN aa_2 = matmn_create(c->jacobian.m, invM.n);
    matmn_mul(&c->jacobian, &invM, &aa_2);
    MatMN lhs = matmn_create(aa_2.m, jacobian_T.n);
    matmn_mul(&aa_2, &jacobian_T, &lhs);
    MatMN aa_1 = matmn_create(c->jacobian.m, v.n);
    matmn_mul(&c->jacobian, &v, &aa_1);
    MatMN rhs = matmn_create_zero_like(&aa_1);
    matmn_scale(&aa_1, -1.0, &rhs);
    MATMN_AT(rhs, 0, 0) -= c->bias;

    MatMN lambda = matmn_create(2, 1);
    matmn_solve_gauss_seidel(&lhs, &rhs, &lambda);

    MatMN old_lambda = matmn_create_zero_like(&c->cached_lambda);
    matmn_copy(&c->cached_lambda, &old_lambda);
    matmn_add(&c->cached_lambda, &lambda, &c->cached_lambda);

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

    matmn_sub(&c->cached_lambda, &old_lambda, &lambda);

    MatMN impulses = matmn_create(jacobian_T.m, lambda.n);
    matmn_mul(&jacobian_T, &lambda, &impulses);

    body_apply_impulse_linear(c->a, (Vec2){MATMN_AT(impulses, 0, 0), MATMN_AT(impulses, 1, 0)});
    body_apply_impulse_angular(c->a, MATMN_AT(impulses, 2, 0));

    body_apply_impulse_linear(c->b, (Vec2){MATMN_AT(impulses, 3, 0), MATMN_AT(impulses, 4, 0)});
    body_apply_impulse_angular(c->b, MATMN_AT(impulses, 5, 0));

    matmn_print(&impulses);
    printf("\n");

    matmn_destroy(&impulses);
    matmn_destroy(&old_lambda);
    matmn_destroy(&lambda);
    matmn_destroy(&rhs);
    matmn_destroy(&aa_1);
    matmn_destroy(&lhs);
    matmn_destroy(&aa_2);
    matmn_destroy(&jacobian_T);
    matmn_destroy(&v);
    matmn_destroy(&invM);
}

#endif