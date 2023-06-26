#ifndef MATMN_H
#define MATMN_H

#include <stdlib.h>
#include <assert.h>

#define MATMN_AT(MAT, I, J) ((MAT).data[(((MAT).n) * ((MAT).s) * (I)) + (J)])

typedef struct
{
    unsigned int m; // rows
    unsigned int n; // cols
    unsigned int s; // stride
    float *data;
} MatMN;

MatMN matmn_create(unsigned int m, unsigned int n)
{
    MatMN a;
    a.m = m;
    a.n = n;
    a.s = 1;
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

MatMN matmn_row(MatMN *a, unsigned int row)
{
    MatMN z;
    z.m = 1;
    z.n = a->n;
    z.s = 1;
    z.data = &MATMN_AT(*a, row, 0);
    return z;
}

MatMN matmn_col(MatMN *a, unsigned int col)
{
    MatMN z;
    z.m = a->m;
    z.n = 1;
    z.s = a->n;
    z.data = &MATMN_AT(*a, 0, col);
    return z;
}

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

void matmn_solve_gauss_seidel(MatMN *a, MatMN *b, MatMN *z)
{
    assert(b->m == 1);
    assert(z->n == b->n);

    for (unsigned int iter = 0; iter < b->n; iter++)
    {
        for (unsigned int i = 0; i < b->n; i++)
        {
            float dot_prod = 0;
            for (unsigned int j = 0; j < b->n; j++)
            {
                dot_prod += MATMN_AT(*a, i, j) * MATMN_AT(*z, 0, j);
            }
            
            float dx = (MATMN_AT(*b, 0, i) / MATMN_AT(*a, i, i)) - dot_prod / MATMN_AT(*a, i, i);
            if (dx == dx)
                MATMN_AT(*z, 0, i) += dx;
        }
    }
}

#endif