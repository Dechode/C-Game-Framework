#pragma once
#include "SDL2/SDL.h"

typedef enum 
{
	UNPRESSED,
	PRESSED,
	HELD,
}KeyState;

typedef struct
{
	KeyState left;
	KeyState right;
	KeyState up;
	KeyState down;
	KeyState escape;
} InputState;

void handleInputEvents(SDL_Event* event, InputState* inputState, int* shouldQuit);
void initInputState(InputState* inputState);

