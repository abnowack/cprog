#ifndef VECN_H
#define VECN_H

#include <stdlib.h>
#include <assert.h>

typedef struct
{
    unsigned int n;
    float *data;
} VecN;

VecN vecn_create(unsigned int n)
{
    VecN a;
    a.n = n;
    a.data = (float*)calloc(n, sizeof(float));
    return a;
}

VecN vecn_destroy(VecN *a)
{
    a->n = 0;
    free(a->data);
    a->data = NULL;
}

VecN vecn_copy(VecN *a)
{
    VecN b = vecn_create(a->n);
    
    for (unsigned int i = 0; i < b.n; i++)
        b.data[i] = a->data[i];
    
    return b;
}

VecN vecn_add(VecN *a, VecN *b)
{
    assert(a->n == b->n);
    
    VecN c = vecn_create(a->n);
    
    for (unsigned int i = 0; i < a->n; i++)
        c.data[i] = a->data[i] + b->data[i];
    
    return c;
}

VecN vecn_sub(VecN *a, VecN *b)
{
    assert(a->n == b->n);
    
    VecN c = vecn_create(a->n);
    
    for (unsigned int i = 0; i < a->n; i++)
        c.data[i] = a->data[i] - b->data[i];
    
    return c;
}

VecN vecn_scale(VecN *a, float b)
{
    VecN c = vecn_create(a->n);
    
    for (unsigned int i = 0; i < a->n; i++)
        c.data[i] = a->data[i] * b;
    
    return c;
}

float vecn_dot(VecN *a, VecN *b)
{
    assert(a->n == b->n);

    float sum = 0.0;
    for (unsigned int i = 0; i < a->n; i++)
        sum += a->data[i] * b->data[i];
    
    return sum;
}

float vecn_at(VecN *a, unsigned int i)
{
    assert(i < a->n);
    return a->data[i];
}

#endif