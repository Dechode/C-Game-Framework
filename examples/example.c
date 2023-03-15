#include "../src/renderer.h"
#include <SDL2/SDL_version.h>

#define WIDTH 800
#define HEIGHT 600

static int shouldQuit = 0;

void handleEvents(SDL_Event* event)
{
	while (SDL_PollEvent(event))
	{
		switch (event->type)
		{
			case SDL_QUIT:
				shouldQuit = 1;
				break;
			
			default:
				break;
		}
    }
}

void gameLoop(SDL_Window* window)
{
	// Initialization
	Sprite sprite;
	initSprite(&sprite, "textures/car16x16.png", (vec3){0.5f * WIDTH, 0.5f * HEIGHT, 0.0f}, (vec2){10.0, 10.0});
	
	// Event handling
	SDL_Event event;
	handleEvents(&event);
	
	// Rendering
	renderBegin();

	drawSprite(&sprite);
	renderTriangle((vec2){WIDTH * 0.7, HEIGHT * 0.6}, (vec2){60,60}, (vec4){1.0, 1.0, 0.4, 1.0});

	renderEnd(window);

	// Cleanup
}

int main(int argc, char** argv)
{

	SDL_Window* window;
	window = initWindow(WIDTH, 600, "Example");
	initRenderer();

	while (!shouldQuit)
	{
		gameLoop(window);
	}
	SDL_DestroyWindow(window);
}
