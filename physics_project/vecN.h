#ifndef VECN_H
#define VECN_H

#include <stdlib.h>
#include <assert.h>

typedef struct
{
    unsigned int n;
    float *data;
} VecN;

VecN vecN_create(unsigned int n)
{
    VecN a;
    a.n = n;
    a.data = (float*)calloc(n, sizeof(float));
    return a;
}

VecN vecN_destroy(VecN *a)
{
    a->n = 0;
    free(a->data);
    a->data = NULL;
}

VecN vecN_copy(VecN *a)
{
    VecN b = vecN_create(a->n);
    
    for (unsigned int i = 0; i < b.n; i++)
        b.data[i] = a->data[i];
    
    return b;
}

VecN vecN_add(VecN *a, VecN *b)
{
    assert(a->n == b->n);
    
    VecN c = vecN_create(a->n);
    
    for (unsigned int i = 0; i < a->n; i++)
        c.data[i] = a->data[i] + b->data[i];
    
    return c;
}

VecN vecN_sub(VecN *a, VecN *b)
{
    assert(a->n == b->n);
    
    VecN c = vecN_create(a->n);
    
    for (unsigned int i = 0; i < a->n; i++)
        c.data[i] = a->data[i] - b->data[i];
    
    return c;
}

VecN vecN_scale(VecN *a, float b)
{
    VecN c = vecN_create(a->n);
    
    for (unsigned int i = 0; i < a->n; i++)
        c.data[i] = a->data[i] * b;
    
    return c;
}

float vecN_dot(VecN *a, VecN *b)
{
    assert(a->n == b->n);

    float sum = 0.0;
    for (unsigned int i = 0; i < a->n; i++)
        sum += a->data[i] * b->data[i];
    
    return sum;
}

float vecN_at(VecN *a, unsigned int i)
{
    assert(i < a->n);
    return a->data[i];
}

#endif