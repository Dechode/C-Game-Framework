#include "../src/renderer.h"
#include <SDL2/SDL_events.h>
#include <SDL2/SDL_keycode.h>
#include <SDL2/SDL_version.h>
#include <math.h>
#include <sys/time.h>
#include <stdio.h>

#define WIDTH 800
#define HEIGHT 600

#define BLOCK_AMOUNT 36

static int shouldQuit = 0;

typedef enum 
{
	UNPRESSED,
	PRESSED,
	HELD,
}KeyState;

typedef struct
{
	KeyState Left;
	KeyState right;
} InputState;

void handleInputEvents(SDL_Event* event, InputState* inputState)
{
	while (SDL_PollEvent(event))
	{
		switch (event->type)
		{
			case SDL_QUIT:
				shouldQuit = 1;
				break;
			case SDL_KEYDOWN:
				 switch( event->key.keysym.sym ){
                    case SDLK_LEFT:
                        inputState->Left = PRESSED;
                        break;
                    case SDLK_RIGHT:
                        inputState->right = PRESSED;
                        break;
					case SDLK_ESCAPE:
						shouldQuit = 1;
						break;
                    default:
                        break;
					}
				break;

			case SDL_KEYUP:
				switch( event->key.keysym.sym ){
					case SDLK_LEFT:
                        inputState->Left = UNPRESSED;
                        break;
                    case SDLK_RIGHT:
                        inputState->right = UNPRESSED;
                        break;
                    default:
                        break;
					}
				break;

			
			default:
				break;
		}
    }
}

typedef struct
{
	vec2 position;
	vec2 size;
	int enabled;
	vec4 color;
} Block;

void initBlock(Block* block, vec4 color, vec2 size, vec2 position, int enabled)
{
	block->color[0] = color[0];
	block->color[1] = color[1];
	block->color[2] = color[2];
	block->color[3] = color[3];

	block->size[0] =size[0] ;
	block->size[1] =size[1] ;

	block->position[0] = position[0]; 
	block->position[1] = position[1]; 

	block->enabled = enabled;

}

void run(SDL_Window* window)
{
	// Initialization
	int rows = 4;
	int cols = BLOCK_AMOUNT / rows;
	int blockGap = 20;
	vec4 color = {1.0f, 1.0f, 1.0f, 1.0f};
	vec2 blockSize = {60.0f, 20.0f};
	
	Block blocks[BLOCK_AMOUNT];
	
	for (int y = 0; y < rows; y++)
	{
		for (int x = 0; x < cols; x++)
		{
			float positionY = HEIGHT - ( 40.0f + (blockSize[1] + 20.0f) * y +1);
			float positionX = blockSize[0] + 0.5 * blockGap + (blockSize[0] + blockGap) * x;
			int i = x + (y * cols);
			initBlock(&blocks[i], color, blockSize, (vec2){positionX, positionY}, 1);
		}
	}

	Block pad;
	initBlock(&pad, (vec4){1.0f, 1.0f, 1.0f, 1.0f}, (vec2){100.0f, 20.0f},(vec2){0.5f * WIDTH, 20.0f}, 1);

	Block ball;
	initBlock(&ball, (vec4){1.0f, 1.0f, 1.0f, 1.0f}, (vec2){10.0f, 10.0f},(vec2){0.5f * WIDTH, 60.0f}, 1);

	InputState inputState;
	SDL_Event event;

	struct timeval t0, t1;
	float deltaTime = 0.0f;

	while (!shouldQuit)
	{
		gettimeofday(&t0,0);

		// Event handling
		handleInputEvents(&event, &inputState);
		
		// Update game entities
		if (inputState.Left == PRESSED)
			pad.position[0] -= 500.0f * deltaTime;
		if (inputState.right == PRESSED)
			pad.position[0] += 500.0f * deltaTime;

		ball.position[1] += 5.0f;

		// Collision checking
		

		// Rendering
		renderBegin();

		renderQuad(0, pad.position, pad.size, pad.color);
		renderQuad(0, ball.position, ball.size, ball.color);

		for (int i = 0; i < BLOCK_AMOUNT; i++)
		{
			if (!blocks[i].enabled)
				continue;
			renderQuad(0, blocks[i].position, blocks[i].size, blocks[i].color);
		}

		renderEnd(window);
		gettimeofday(&t1,0);
		deltaTime = (t1.tv_sec - t0.tv_sec) * 1000.0f + (t1.tv_usec - t0.tv_usec) / 1000.0f;
		deltaTime *= 0.001f;
		printf("deltaTime = %f\n", deltaTime);
	}

	// Cleanup
	glUseProgram(0);
}

int main(int argc, char** argv)
{

	SDL_Window* window;
	window = initWindow(WIDTH, HEIGHT, "Breakout clone");
	initRenderer();

	run(window);

	SDL_DestroyWindow(window);
}

