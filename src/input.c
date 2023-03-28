#include "input.h"

void initInputState(InputState* inputState)
{
	inputState->escape = UNPRESSED;
	inputState->left =UNPRESSED;
	inputState->right = UNPRESSED;
	inputState->up = UNPRESSED;
	inputState->down = UNPRESSED;
}

void handleInputEvents(SDL_Event* event, InputState* inputState, int* shouldQuit)
{
//	while (SDL_PollEvent(event))
//	{
switch (event->type)
{
	case SDL_KEYDOWN:
		switch( event->key.keysym.sym )
		{
			case SDLK_LEFT:
				inputState->left = PRESSED;
				break;
			case SDLK_RIGHT:
				inputState->right = PRESSED;
				break;
			case SDLK_ESCAPE:
				inputState->escape = PRESSED;
				*shouldQuit = 1;
				break;
			case SDLK_DOWN:
				inputState->down = PRESSED;
				break;
			case SDLK_UP:
				inputState->up = PRESSED;
				break;
			default:
				break;
		}
		break;

	case SDL_KEYUP:
		switch( event->key.keysym.sym )
		{
			case SDLK_LEFT:
				inputState->left = UNPRESSED;
				break;
			case SDLK_RIGHT:
				inputState->right = UNPRESSED;
				break;

			case SDLK_DOWN:
				inputState->down = UNPRESSED;
				break;
			case SDLK_UP:
				inputState->up = UNPRESSED;
				break;
			default:
				break;
		}
		break;
}
//    }
}
