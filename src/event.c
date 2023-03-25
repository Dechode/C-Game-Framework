#include "event.h"
#include "input.h"

void handleEvents(SDL_Event* event, InputState* inputState, int* shouldQuit)
{	
	while (SDL_PollEvent(event))
	{
		switch (event->type)
		{
			case SDL_QUIT:
				*shouldQuit = 1;
				break;

			default:
				break;
		}
		handleInputEvents(event, inputState);
	}

}

