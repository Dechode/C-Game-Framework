#include "input.h"

void handleInputEvents(SDL_Event* event, InputState* inputState)
{
	while (SDL_PollEvent(event))
	{
		switch (event->type)
		{
			case SDL_QUIT:
				inputState->escape = PRESSED;
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
						inputState->escape = PRESSED;
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
