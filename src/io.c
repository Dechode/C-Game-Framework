#include <stdio.h>
#include <stdlib.h>
#include <errno.h>

#include "io.h"

#define IO_READ_CHUNK_SIZE 2097152
#define IO_READ_ERROR_GENERAL "Error reading file: %s. errno: %d\n"
#define IO_READ_ERROR_MEMORY "Not enough memory to read the file: %s\n"

File ioReadFile(const char *path){
    File file = { .isValid = false};

    FILE *fp = fopen(path, "rb");
    if (!fp || ferror(fp)) {
        printf(IO_READ_ERROR_GENERAL, path, errno);
    }

    char *data = NULL;
    char *tmp;
    size_t used = 0;
    size_t size = 0;
    size_t n;

    while (true) {
        if (used + IO_READ_CHUNK_SIZE + 1 > size) {
            size = used + IO_READ_CHUNK_SIZE + 1;

            if (size <= used){
                free(data);
                printf("ERROR Input file too large: %s\n", path);
            }
            tmp = realloc(data, size);
            if (!tmp) {
                free(data);
                printf(IO_READ_ERROR_MEMORY, path);
            }
            data = tmp;
        }
        n = fread(data + used, 1, IO_READ_CHUNK_SIZE, fp);
        if (n == 0) {
            break;
        }
        used += n;
    }
    if (ferror(fp)) {
        free(data);
        printf(IO_READ_ERROR_GENERAL, path, errno);
    }
    tmp = realloc(data, used +1);
    if (!tmp) {
        free(data);
        printf(IO_READ_ERROR_MEMORY, path);
    }
    data = tmp;
    data[used] = 0;
    file.data = data;
    file.len = used;
    file.isValid = true;

    return file;
}

int ioFileWrite(void *buffer, size_t size, const char* path) {
    FILE *fp = fopen(path, "wb");
    if (!fp || ferror(fp)) {
        printf("Cannot write file: %s.\n", path);
    }
    size_t chunks_written = fwrite(buffer, size, 1, fp);

    fclose(fp);

    if (chunks_written != 1) {
        printf("Write error. Expected 1 chunk, got %zu.\n ", chunks_written);
    }
    return 0;
}
