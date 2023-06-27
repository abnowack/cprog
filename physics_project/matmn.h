#ifndef MATMN_H
#define MATMN_H

#include <stdlib.h>
#include <assert.h>

#define MATMN_AT(MAT, I, J) ((MAT).data[(((MAT).n) * (I)) + (J)])

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

void matmn_destroy(MatMN *a)
{
    free(a->data);
}

void matmn_copy(MatMN *a, MatMN *z)
{
    assert(a->m == z->m);
    assert(a->n == z->n);

    for (unsigned int i = 0; i < (a->m * a->n); i++)
    {
        z->data[i] = a->data[i];
    }
}

MatMN matmn_create_zero_like(MatMN *a)
{
    return matmn_create(a->m, a->n);
}

// MatMN matmn_row(MatMN *a, unsigned int row)
// {
//     MatMN z;
//     z.m = 1;
//     z.n = a->n;
//     z.data = &MATMN_AT(*a, row, 0);
//     return z;
// }

// MatMN matmn_col(MatMN *a, unsigned int col)
// {
//     MatMN z;
//     z.m = a->m;
//     z.n = 1;
//     z.data = &MATMN_AT(*a, 0, col);
//     return z;
// }

void matmn_transpose(MatMN *a, MatMN *z)
{
    assert(a->m == z->n);
    assert(a->n == z->m);
    assert(a != z);

    for (unsigned int i = 0; i < a->m; i++)
    {
        for (unsigned int j = 0; j < a->n; j++)
        {
            MATMN_AT(*z, j, i) = MATMN_AT(*a, i, j);
        }
    }
}

void matmn_scale(MatMN *a, float b, MatMN *z)
{
    assert(a->m == z->m);
    assert(a->n == z->n);

    for (unsigned int i = 0; i < (a->m * a->n); i++)
    {
        z->data[i] = a->data[i] * b;
    }
}

void matmn_set(float a, MatMN *z)
{
    for (unsigned int i = 0; i < (z->m * z->n); i++)
    {
        z->data[i] = a;
    }
}

void matmn_add(MatMN *a, MatMN *b, MatMN *z)
{
    assert(a->m == b->m);
    assert(a->n == b->n);

    assert(a->m == z->m);
    assert(a->n == z->n);

    for (unsigned int i = 0; i < (a->m * a->n); i++)
    {
        z->data[i] = a->data[i] + b->data[i];
    }
}

void matmn_sub(MatMN *a, MatMN *b, MatMN *z)
{
    assert(a->m == b->m);
    assert(a->n == b->n);

    assert(a->m == z->m);
    assert(a->n == z->n);

    for (unsigned int i = 0; i < (a->m * a->n); i++)
    {
        z->data[i] = a->data[i] - b->data[i];
    }
}

void matmn_mul(MatMN *a, MatMN *b, MatMN *z)
{
    assert(a->n == b->m);
    assert(z->m == a->m);
    assert(z->n == b->n);

    for (unsigned int i = 0; i < a->m; i++)
    {
        for (unsigned int j = 0; j < b->n; j++)
        {
            MATMN_AT(*z, i, j) = 0;
            for (unsigned int k = 0; k < a->n; k++)
                MATMN_AT(*z, i, j) += MATMN_AT(*a, i, k) * MATMN_AT(*b, k, j);
        }
    }
}

float matmn_dot_prod(MatMN *a, MatMN *b)
{
    assert(a->m == 1);
    assert(b->n == 1);
    assert(a->n == b->m);

    float dot_prod = 0;
    for (unsigned int i = 0; i < a->n; i++)
    {
        dot_prod += MATMN_AT(*a, 0, i) * MATMN_AT(*b, i, 0);
    }
    return dot_prod;
}

void matmn_solve_gauss_seidel(MatMN *a, MatMN *b, MatMN *x)
{
    assert(b->n == 1);
    assert(x->n == 1);
    assert(x->m == b->m);

    matmn_set(0.0, x);

    for (unsigned int iter = 0; iter < 10; iter++)
    {
        for (unsigned int i = 0; i < b->m; i++)
        {
            float dot_prod = 0;
            for(unsigned int j = 0; j < x->m; j++)
            {
                dot_prod += MATMN_AT(*x, j, 0) * MATMN_AT(*a, i, j);
            }

            if (MATMN_AT(*a, i, i) != 0.0)
                MATMN_AT(*x, i, 0) = MATMN_AT(*x, i, 0) + MATMN_AT(*b, i, 0) / MATMN_AT(*a, i, i) - dot_prod / MATMN_AT(*a, i, i);
        }
    }
}

#endif