#ifndef MATMN_H
#define MATMN_H

#include "vecn.h"

#define MATMN_AT(MAT, I, J) ((MAT).data[((MAT).n * I) + J])

typedef struct
{
    unsigned int m; // rows
    unsigned int n; // cols
    float *data;
} MatMN;

MatMN matmn_create(unsigned int m, unsigned int n)
{
    MatMN a;
    a.m = m;
    a.n = n;
    a.data = (float *)calloc(m * n, sizeof(float));

    return a;
}

MatMN matmn_copy(MatMN *a)
{
    MatMN b = matmn_create(a->m, a->n);
    for (unsigned int i = 0; i < (a->m * a->n); i++)
    {
        b.data[i] = a->data[i];
    }
    return b;
}

MatMN matmn_transpose(MatMN *a)
{
    MatMN b = matmn_create(a->n, a->m);
    for (unsigned int i = 0; i < a->m; i++)
    {
        for (unsigned int j = 0; j < a->n; j++)
        {
            MATMN_AT(b, j, i) = MATMN_AT(*a, i, j);
        }
    }
    return b;
}

VecN matmn_vec_mul(MatMN *a, VecN *b)
{
    assert(b->n == a->n);
    VecN c = vecn_create(a->m);

    for (unsigned int i = 0; i < a->m; i++)
    {
        c.data[i] = 0;
        for (unsigned int j = 0; j < a->n; j++)
        {
            c.data[i] += b->data[j] * MATMN_AT(*a, i, j);
        }
    }
    return c;
}

MatMN matmn_mat_mul(MatMN *a, MatMN *b)
{
    // assert(a->n == b->m);
    // assert(a->m == b->n);

    if (b->m != a->n && b->n != a->m)
        return matmn_copy(a);

    MatMN c = matmn_create(a->m, b->n);

    for (unsigned int i = 0; i < a->m; i++)
    {
        for (unsigned int j = 0; j < b->n; j++)
        {
            MATMN_AT(c, i, j) = 0;
            for (unsigned int k = 0; k < a->n; k++)
                MATMN_AT(c, i, j) += MATMN_AT(*a, i, k) * MATMN_AT(*b, k, j);
        }
    }
    return c;
}

MatMN matmn_scale(MatMN *a, float b)
{
    MatMN c = matmn_create(a->m, a->n);
    for (unsigned int i = 0; i < (a->m * a->n); i++)
    {
        c.data[i] = a->data[i] * b;
    }
    return c;
}

VecN matmn_solve_gauss_seidel(MatMN *a, VecN *b)
{
    VecN X = vecn_create(b->n);

    for (unsigned int iter = 0; iter < b->n; iter++)
    {
        for (unsigned int i = 0; i < b->n; i++)
        {
            float dot_prod = 0;
            for (unsigned int j = 0; j < b->n; j++)
            {
                dot_prod += MATMN_AT(*a, i, j) * X.data[j];
            }

            float dx = (b->data[i] / MATMN_AT(*a, i, i)) - dot_prod / MATMN_AT(*a, i, i);
            if (dx == dx)
                X.data[i] += dx;
        }
    }
    return X;
}

#endif