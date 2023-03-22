#include "../src/renderer.h"
#include <SDL2/SDL_events.h>
#include <SDL2/SDL_keycode.h>
#include <SDL2/SDL_version.h>
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
	Block blocks[BLOCK_AMOUNT];
	vec4 color = {1.0f, 1.0f, 1.0f, 1.0f};
	vec2 blockSize = {60.0f, 20.0f};
	
	for (int y = 0; y < rows; y++)
	{
		for (int x = 0; x < cols; x++)
		{
			float positionY = HEIGHT - ( 40.0f + (blockSize[1] + 20.0f) * y +1);
			float positionX = blockSize[0] + (blockSize[0] + blockGap) * x;
			int i = x + (y * cols);
			initBlock(&blocks[i], color,blockSize, (vec2) {positionX, positionY}, 1);
		}
	}

	Block pad;
	pad.position[0] = 0.5f * WIDTH;
	pad.position[1] = 20.0f;
	pad.color[0] = 1.0f;
	pad.color[1] = 1.0f;
	pad.color[2] = 1.0f;
	pad.color[3] = 1.0f;

	pad.size[0] = 100.0f;
	pad.size[1] = 20.0f;
	pad.enabled = 1;


	// Event handling
	InputState inputState;
	SDL_Event event;

	while (!shouldQuit)
	{
		handleInputEvents(&event, &inputState);
		
		// Update game entities
		if (inputState.Left == PRESSED)
			pad.position[0] -= 5;
		if (inputState.right == PRESSED)
			pad.position[0] += 5;

		// Rendering
		renderBegin();

		renderQuad(0, pad.position, pad.size, pad.color);

		for (int i = 0; i < BLOCK_AMOUNT; i++)
		{
			renderQuad(0, blocks[i].position, blocks[i].size, blocks[i].color);
		}

		renderEnd(window);
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
