#ifndef IO_H
#define IO_H
#include <stdlib.h>
#include <stdbool.h>

typedef struct file {
    char *data;
    size_t len;
    bool isValid;
} File;

File ioReadFile(const char *path);
int ioFileWrite(void *buffer, size_t size, const char *path);
#endif // IO_H
