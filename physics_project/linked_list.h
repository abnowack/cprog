#ifndef LINKED_LIST_H
#define LINKED_LIST_H

#include <stdlib.h>

struct Node_T
{
    void *data;
    struct Node_T *next;
};

typedef struct Node_T Node;

typedef struct
{
    Node *start;
    Node *end;
} List;

List list_create_empty()
{
    List l;
    l.start = NULL;
    l.end = NULL;
    return l;
}

unsigned int list_length(List *list)
{
    Node *n = list->start;
    unsigned int len = 0;

    for(Node *n = list->start, *next; n; n = next)
    {
        len++;
        next = n->next;
    }
    return len;
}

void List_push(List *list, void *data)
{
    Node *new_node = (Node *)malloc(sizeof(Node));
    new_node->data = data;
    new_node->next = NULL;

    if (list->start == NULL)
    {
        list->start = new_node;
        list->end = new_node;
    }
    else
    {
        list->end->next = new_node;
        list->end = new_node;
    }
}

#endif