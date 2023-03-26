#pragma once

#include <stdint.h>
#include <stddef.h>

typedef struct arrayList
{
    size_t len;
    size_t capacity;
    size_t itemSize;
    void *items;
} ArrayList;

ArrayList *arrayListCreate(size_t itemSize, size_t initialCapasity);
size_t arrayListAppend(ArrayList *list, void* item);
void *arrayListGet(ArrayList *list, size_t index);
uint8_t arrayListRemove(ArrayList *list, size_t index);
