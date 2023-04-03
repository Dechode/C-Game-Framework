#!/bin/bash

set -xe

echo 'Building breakout clone example'
# gcc -Wall -Wextra -o example examples/example.c src/event.c src/input.c src/physics.c src/array_list.c src/renderer.c src/glad.c src/io.c -lSDL2main -lSDL2 -lSDL2_image -lm

echo '----Building physics test----'
# gcc -Wall -Wextra -o physics_test examples/physics_test.c src/glad.c src/event.c src/input.c src/io.c src/physics.c src/array_list.c src/renderer.c -lSDL2main -lSDL2 -lSDL2_image -lm 

