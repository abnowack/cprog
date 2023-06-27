#ifndef MEM_H
#define MEM_H

#include <stdlib.h>

#define DEBUG_MEM

#include <stdio.h>

struct MemoryLog
{
    unsigned long memory_allocated;
    unsigned long memory_calls;
};

struct MemoryLog mem_log = {.memory_allocated = 0, .memory_calls = 0};

struct ScratchPool
{
    char data[100000];
    unsigned int index;
};

struct ScratchPool scratch_pool = {.index = 0};

typedef enum
{
    MEM_HEAP,
    MEM_SCRATCH_POOL
} MEMORY_TAG;

void mem_reset_scratch_pool()
{
    scratch_pool.index = 0;
}

void *mem_calloc(size_t n, size_t size, MEMORY_TAG tag)
{

    if (tag == MEM_HEAP)
    {
#ifdef DEBUG_MEM
        // printf("[MEM] Allocating %zu objects of size %zu\n", n, size);
        mem_log.memory_allocated += n * size;
        mem_log.memory_calls++;
#endif
        return calloc(n, size);
    }
    else if (tag == MEM_SCRATCH_POOL)
    {
        void *data_ptr = (void *)&scratch_pool.data[scratch_pool.index];
        scratch_pool.index += n * size;
        return data_ptr;
    }
}

void *mem_malloc(size_t size)
{
#ifdef DEBUG_MEM
    // printf("[MEM] Allocating %zu objects of size %zu\n", n, size);
    mem_log.memory_allocated += size;
    mem_log.memory_calls++;
#endif

    return malloc(size);
}

void mem_free(void *a)
{
#ifdef DEBUG_MEM
    // printf("[MEM] Freeing memory at %p\n", a);
#endif
    free(a);
}

#endif