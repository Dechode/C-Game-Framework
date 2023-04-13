#include "../src/event.h"
#include "../src/physics.h"
#include "../src/renderer.h"
#include <SDL2/SDL.h>
#include <SDL2/SDL_events.h>
#include <SDL2/SDL_keycode.h>
#include <SDL2/SDL_version.h>
#include <math.h>
#include <stdio.h>
#include <sys/time.h>

#define WIDTH 800
#define HEIGHT 600

#define vec2ZERO                                                               \
(vec2) { 0.0f, 0.0f }

#define BLOCK_AMOUNT 36

static int shouldQuit = 0;

typedef struct {
	PhysicsBody2D body;
	int enabled;
	vec4 color;
} Block;

void initBlock(Block *block, const vec4 color, const vec2 size,
			   const vec2 position, const int enabled) {
	block->color[0] = color[0];
	block->color[1] = color[1];
	block->color[2] = color[2];
	block->color[3] = color[3];

	initPhysicsBody2D(&block->body, size, position, vec2ZERO, 0.0f, 100.0f, 1.0f,
				   1.0, 1, CIRCLE);

	block->enabled = enabled;
}

void run(SDL_Window *window) {
	// Initialization
	int rows = 4;
	int cols = BLOCK_AMOUNT / rows;
	int blockGap = 20;
	vec4 color = {1.0f, 1.0f, 1.0f, 1.0f};
	vec2 blockSize = {60.0f, 20.0f};

	Block blocks[BLOCK_AMOUNT];

	for (int y = 0; y < rows; y++) {
		for (int x = 0; x < cols; x++) {
			float positionY =
				HEIGHT - (2 * blockGap + (blockSize[1] + blockGap) * y + 1);
			float positionX =
				blockSize[0] + 0.5 * blockGap + (blockSize[0] + blockGap) * x;
			vec2 pos = {positionX, positionY};
			int i = x + (y * cols);
			initBlock(&blocks[i], color, blockSize, pos, 1);
		}
	}

	Block pad;
	initBlock(&pad, (vec4){1.0f, 1.0f, 1.0f, 1.0f}, (vec2){100.0f, 20.0f},
		   (vec2){0.5f * WIDTH, 20.0f}, 1);

	Block ball;
	initBlock(&ball, (vec4){1.0f, 1.0f, 1.0f, 1.0f}, (vec2){10.0f, 10.0f},
		   (vec2){0.5f * WIDTH, 60.0f}, 1);

	InputState inputState;
	SDL_Event event;

	struct timeval t0, t1;
	float deltaTime = 0.0f;
	float fps = 0.0f;

	// Gameloop
	while (!shouldQuit) {
		// Start time of the frame
		gettimeofday(&t0, 0);

		// Event handling
		handleEvents(&event, &inputState, &shouldQuit);
		shouldQuit = shouldQuit || inputState.escape;

		// Update game entities
		// moveKinematicBody2D(&pad.body, (vec2){0.0f, 0.0f}, 0.0f);
		if (inputState.left == PRESSED)
			movePhysicsBody2D(&pad.body, (vec2){-1.0f * 500.0f, 0.0f * 500.0f});
		if (inputState.right == PRESSED)
			movePhysicsBody2D(&pad.body, (vec2){1.0f * 500.0f, 0.0f * 500.0f});

		updatePhysicsBody2D(&pad.body, deltaTime);
		updatePhysicsBody2D(&ball.body, deltaTime);

		// Rendering
		renderBegin();

		renderQuad(0, 0, 0.0f, pad.body.position, pad.body.size, pad.color);
		renderQuad(0, 0, 0.0f, ball.body.position, ball.body.size, ball.color);

		for (int i = 0; i < BLOCK_AMOUNT; i++) {
			if (!blocks[i].enabled)
				continue;
			renderQuad(0, blocks[i].body.position, blocks[i].body.size,
			  blocks[i].color);
		}

		// End time of the frame and delta time
		gettimeofday(&t1, 0);
		deltaTime =
			(t1.tv_sec - t0.tv_sec) * 1000.0f + (t1.tv_usec - t0.tv_usec) / 1000.0f;
		deltaTime *= 0.001f;

		fps = 1 / deltaTime;
		//		printf("FPS = %2.2f\n", fps);

		renderEnd(window);
	}

	// Cleanup
	glUseProgram(0);
}

int main(int argc, char **argv) {

	SDL_Window *window;
	window = initWindow(WIDTH, HEIGHT, "Breakout clone");
	initRenderer();

	run(window);

	SDL_DestroyWindow(window);
	SDL_Quit();

	return 0;
}
