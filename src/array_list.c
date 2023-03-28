#include <stdlib.h>
#include <stdio.h>
#include "array_list.h"
#include <string.h>

ArrayList* arrayListCreate(size_t itemSize, size_t initialCapacity)
{
    ArrayList* list = (ArrayList*) malloc(sizeof(ArrayList));
    if (!list)
    {
        printf("Could not allocate memory for array list\n");
        return NULL;
    }
    list->itemSize = itemSize;
    list->capacity = initialCapacity;
    list->len = 0;
    list->items = (void*) malloc(itemSize * initialCapacity);

    if (!list->items)
    {
        printf("Could not allocate memory for array list\n");
        return NULL;
    }

    return list;
}

size_t arrayListAppend(ArrayList* list, void* item)
{
    if (list->len == list->capacity) {
        list->capacity = list->capacity > 0 ? list->capacity * 2 : 1;
        void *items = realloc(list->items, list->itemSize * list->capacity);
        if (!items)
        {
            printf("Could not allocate memory for array list\n");
            return -1;
        }
        list->items = items;
    }
    size_t index = list->len++;
    memcpy((void*)list->items + index * list->itemSize, item, list->itemSize);
    return index;
}

void *arrayListGet(ArrayList *list, size_t index)
{
    if (index >= list->len)
    {
        printf("Index out of bounds\n");
        return NULL;
    }
    // return (uint8_t*)list->items + index * list->itemSize;
    return (void*)list->items + index * list->itemSize;
}

uint8_t arrayListRemove(ArrayList *list, size_t index)
{
    if (list->len == 0)
    {
        printf("Error List is empty\n");
        return 1;
    }
    if (index >= list->len)
    {
        printf("Index out of bounds\n");
        return 1;
    }
    if (list->len == 1) {
        list->len = 0;
        return 0;
    }

    --list->len;

    uint8_t *itemPtr = (uint8_t*)list->items + index * list->itemSize;
    uint8_t *endPtr = (uint8_t*)list->items + list->len * list->itemSize;
    memcpy(itemPtr, endPtr, list->itemSize);

    return 0;
}
