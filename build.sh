#!/bin/bash

set -xe

gcc -Wall -Wextra -o example examples/example.c src/event.c src/input.c src/physics.c src/renderer.c src/glad.c src/io.c -lSDL2main -lSDL2 -lSDL2_image -lm
