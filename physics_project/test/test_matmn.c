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
    // MatMN a = matmn_create(3, 4);
    // for (unsigned int i = 0; i < a.m; i++)
    // {
    //     for (unsigned int j = 0; j < a.n; j++)
    //     {
    //         MATMN_AT(a, i, j) = i + 0.1 * j;
    //     }
    // }
    // matmn_print(&a);
    // printf("\n");

    // MatMN a_row = matmn_row(&a, 1);
    // matmn_print(&a_row);
    // printf("\n");

    // MatMN a_col = matmn_col(&a, 1);
    // matmn_print(&a_col);
    // printf("\n");

    // MatMN a_T = matmn_create(a.n, a.m);
    // matmn_transpose(&a, &a_T);
    // matmn_print(&a_T);
    // printf("\n");
    // matmn_destroy(&a_T);

    // MatMN a_scale = matmn_create_zero_like(&a);
    // matmn_scale(&a, 2.0, &a_scale);
    // matmn_print(&a_scale);
    // printf("\n");
    // matmn_destroy(&a_scale);

    // MatMN b = matmn_create(4, 3);
    // for (unsigned int i = 0; i < b.m; i++)
    // {
    //     for (unsigned int j = 0; j < b.n; j++)
    //     {
    //         MATMN_AT(b, i, j) = i + 0.1 * j;
    //     }
    // }
    // MatMN a_mul_b = matmn_create(a.m, b.n);
    // matmn_mul(&a, &b, &a_mul_b);
    // matmn_print(&a_mul_b);
    // printf("\n");
    // matmn_destroy(&b);
    // matmn_destroy(&a_mul_b);

    // matmn_destroy(&a);

    MatMN lhs = matmn_create(4, 4);
    MATMN_AT(lhs, 0, 0) = 10.0;
    MATMN_AT(lhs, 0, 1) = -1.0;
    MATMN_AT(lhs, 0, 2) = 2.0;
    MATMN_AT(lhs, 0, 3) = 0.0;
    MATMN_AT(lhs, 1, 0) = -1.0;
    MATMN_AT(lhs, 1, 1) = 11.0;
    MATMN_AT(lhs, 1, 2) = -1.0;
    MATMN_AT(lhs, 1, 3) = 3.0;
    MATMN_AT(lhs, 2, 0) = 2.0;
    MATMN_AT(lhs, 2, 1) = -1.0;
    MATMN_AT(lhs, 2, 2) = 10.0;
    MATMN_AT(lhs, 2, 3) = -1.0;
    MATMN_AT(lhs, 3, 0) = 0.0;
    MATMN_AT(lhs, 3, 1) = 3.0;
    MATMN_AT(lhs, 3, 2) = -1.0;
    MATMN_AT(lhs, 3, 3) = 8.0;
    printf("lhs\n");
    matmn_print(&lhs);
    printf("\n");

    MatMN rhs = matmn_create(4, 1);
    MATMN_AT(rhs, 0, 0) = 6.0;
    MATMN_AT(rhs, 1, 0) = 25.0;
    MATMN_AT(rhs, 2, 0) = -11.0;
    MATMN_AT(rhs, 3, 0) = 15.0;
    printf("rhs\n");
    matmn_print(&rhs);
    printf("\n");

    MatMN sol = matmn_create_zero_like(&rhs);
    matmn_solve_gauss_seidel(&lhs, &rhs, &sol);
    printf("sol \n");
    matmn_print(&sol);
    printf("\n");

    // MatMN a_mul_sol = matmn_create(a.m, sol.n);
    // matmn_mul(&a, &sol, &a_mul_sol);
    // printf("a * sol\n");
    // matmn_print(&a_mul_sol);
    // printf("\n");

    return 0;
}