#ifndef UTIL_H
#define UTIL_H

#include "matmn.h"

void matmn_print_label(MatMN *a, char* label)
{
    if (label)
        printf(" - %s - \n", label);
    
    for (unsigned int i = 0; i < a->m; i++)
    {
        for (unsigned int j = 0; j < a->n; j++)
        {
            printf("%f ", MATMN_AT(*a, i, j));
        }
        printf("\n");
    }
    printf("\n");
}

void matmn_print(MatMN *a)
{
    matmn_print_label(a, "");
}

#endif