#include <stdio.h>
#include "../matmn.h"
#include "../vecn.h"

void matmn_print(MatMN *a)
{
    for (unsigned int i = 0; i < a->m; i++)
    {
        for (unsigned int j = 0; j < a->n; j++)
        {
            printf("%.3f ", MATMN_AT(*a, i, j));
        }
        printf("\n");
    }
}

void vecn_print(VecN *a)
{
    for (unsigned int i = 0; i < a->n; i++)
    {
        printf("%.3f ", a->data[i]);
    }
    printf("\n");
}

int main(void)
{
    MatMN a = matmn_create(3, 4);

    for (unsigned int i = 0; i < a.m; i++)
    {
        for (unsigned int j = 0; j < a.n; j++)
        {
            MATMN_AT(a, i, j) = i + 0.1 * j;
        }
    }

    matmn_print(&a);
    printf("\n");

    MatMN b = matmn_transpose(&a);

    matmn_print(&b);
    printf("\n");

    MatMN c = matmn_copy(&a);

    matmn_print(&c);
    printf("\n");

    MatMN d = matmn_scale(&a, -2.0);

    matmn_print(&d);
    printf("\n");

    VecN v1 = vecn_create(4);
    v1.data[0] = 2;
    v1.data[1] = 3;
    v1.data[2] = 4;
    v1.data[3] = 5;

    VecN v2 = matmn_vec_mul(&a, &v1);

    vecn_print(&v1);
    vecn_print(&v2);
    printf("\n");

    MatMN e = matmn_mat_mul(&a, &b);
    matmn_print(&e);

    return 0;
}