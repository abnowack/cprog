#include <stdio.h>
#include "../matmn.h"

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

    MatMN a_row = matmn_row(&a, 1);
    matmn_print(&a_row);
    printf("\n");

    MatMN a_col = matmn_col(&a, 1);
    matmn_print(&a_col);
    printf("\n");

    MatMN a_T = matmn_create(a.n, a.m);
    matmn_transpose(&a, &a_T);
    matmn_print(&a_T);
    printf("\n");
    matmn_destroy(&a_T);

    MatMN a_scale = matmn_create_zero_like(&a);
    matmn_scale(&a, 2.0, &a_scale);
    matmn_print(&a_scale);
    printf("\n");
    matmn_destroy(&a_scale);

    MatMN b = matmn_create(4, 3);
    for (unsigned int i = 0; i < b.m; i++)
    {
        for (unsigned int j = 0; j < b.n; j++)
        {
            MATMN_AT(b, i, j) = i + 0.1 * j;
        }
    }
    MatMN a_mul_b = matmn_create(a.m, b.n);
    matmn_mul(&a, &b, &a_mul_b);
    matmn_print(&a_mul_b);
    printf("\n");
    matmn_destroy(&b);
    matmn_destroy(&a_mul_b);

    return 0;
}