#include "../src/event.h"
#include "../src/physics.h"
#include "../src/renderer.h"
#include <sys/time.h>

static int width = 800;
static int height = 600;

#define BODY_COUNT 10

void printVec2(vec2 a) { printf("{%2.2f, %2.2f}\n", a[0], a[1]); }

void printPhysicsBody2D(PhysicsBody2D *body) {
  printf("PhysicsBody2D info:\n");
  puts("Position:");
  printVec2(body->position);
  puts("Acceleration:");
  printVec2(body->acceleration);
  puts("Velocity:");
  printVec2(body->velocity);
}
void drawPhysicsBody2D(PhysicsBody2D *body, float z) {
  if (body->collisionShape == RECTANGLE)
  {
    vec3 pos = {body->position[0], body->position[1], z};
    renderQuad(-1, -1, body->rotation, pos, body->size,
               (vec4){1.0f, 1.0f, 1.0f, 1.0f});
  }
  if (body->collisionShape == CIRCLE)
  {
    drawCircle(body->size[0] * 0.5f, body->position);
  }
}

void run(SDL_Window *window) {
  static int shouldQuit = 0;
  InputState inputState;
  SDL_Event event;

  initInputState(&inputState);
  initPhysicsState2D((vec2){0.0f, -10.0f}, 10000.0f, 144, 4);
  // initPhysicsState2D((vec2){0.0f, 0.0f}, 1000.0f, 144, 4);

  struct timeval t0, t1;
  float deltaTime = 0.007f;
  float fps = 144.0f;

  size_t physicsBodiesList[BODY_COUNT] = {-1};

  for (int i = 0; i < BODY_COUNT; i++) {
    physicsBodiesList[i] = createPhysicsBody2D(
        (vec2){10.0f, 10.0f}, (vec2){140.0f + (i * 20), (1 + i) * 20.0f},
        (vec2){30.0f, 33.0f}, 0.0f, 100.0f, 1.0f, 1.0f, 1, RECTANGLE);
    printf("Body id %d = %zu\n", i, physicsBodiesList[i]);
  }
  while (!shouldQuit) {

    // Beginning of frame
    gettimeofday(&t0, 0);

    handleEvents(&event, &inputState, &shouldQuit);

    // Update
    vec2 vel = {0};
    vec2 input = {0};
    input[0] = ((float)inputState.right - inputState.left);
    input[1] = ((float)inputState.up - inputState.down);
    if (vec2_len(input) > 1.0f) {
      vec2_norm(input, input);
    }
    vel[0] = input[0] * 500.0f;
    vel[1] = input[1] * 500.0f;

    physicsUpdate(deltaTime);

    // Rendering
    renderBegin();

    // Draw here
    for (size_t i = 0; i < BODY_COUNT; i++)
    {
      PhysicsBody2D* b = getPhysicsBody2D(i);
      windowCollision(b, width, height, deltaTime);
      drawPhysicsBody2D(b, 0.0f);
    }

    renderEnd(window);

    // End of frame
    gettimeofday(&t1, 0);
    deltaTime =
        (t1.tv_sec - t0.tv_sec) * 1000.0f + (t1.tv_usec - t0.tv_usec) / 1000.0f;
    deltaTime *= 0.001f;
    fps = 1 / deltaTime;
  }
}

int main(int argc, char **argv) {
  SDL_Window *window;
  window = initWindow(width, height, "Physics test");
  initRenderer();

  run(window);

  SDL_DestroyWindow(window);
  return 0;
}
