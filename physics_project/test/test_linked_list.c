#include <stdio.h>
#include "../linked_list.h"

int main(void)
{
    List int_list = list_create_empty();
    
    printf("%d\n", list_length(&int_list));

    int a = 10, b = 20, c = 30, d = 40;

    List_push(&int_list, &a);
    printf("%d\n", list_length(&int_list));

    List_push(&int_list, &b);
    printf("%d\n", list_length(&int_list));

    List_push(&int_list, &c);
    printf("%d\n", list_length(&int_list));

    List_push(&int_list, &d);
    printf("%d\n", list_length(&int_list));

    for(Node *n = int_list.start, *next; n != NULL; n = next)
    {
        printf("%d \n", *((int*)n->data));
        next = n->next;
    }

    return 0;
}