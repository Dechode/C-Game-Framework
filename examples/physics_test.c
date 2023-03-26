#include "../src/event.h"
#include "../src/physics.h"
#include "../src/renderer.h"
#include <sys/time.h>

static int width = 800;
static int height = 600;

void printVec2(vec2 a) { printf("Position = {%2.2f, %2.2f}\n", a[0], a[1]); }

void run(SDL_Window *window) {
  static int shouldQuit = 0;
  InputState inputState;
  SDL_Event event;

  initInputState(&inputState);

  struct timeval t0, t1;
  float deltaTime = 0.007f;
  float fps = 144.0f;

  RigidBody2D body;
  initRigidBody2D(&body, (vec2){20.0f, 20.0f},
                  (vec2){0.5f * width, 0.5f * height}, (vec2){-70.0f, 90.0f},
                  0.0f, 30.0f, 1.0f, 0.0f);

  while (!shouldQuit) {
    printVec2(body.physicsBody.velocity);
    // Beginning of frame
    gettimeofday(&t0, 0);

    handleEvents(&event, &inputState, &shouldQuit);
    shouldQuit = shouldQuit || inputState.escape;

    // Update
    applyForceRigidBody2D(&body, (vec2){0.0f, -100.0f});
    updateRigidBody2D(&body, deltaTime);
    // movePhysicsBody2D(&body.physicsBody, (vec2){10.0f, 10.0f});
    // updatePhysicsBody2D(&body.physicsBody, deltaTime);
    // windowCollision(&body.physicsBody, width, height, deltaTime);

    // Rendering
    renderBegin();

    // Draw here
    vec3 pos = {body.physicsBody.position[0], body.physicsBody.position[1],
                0.0f};
    renderQuad(-1, -1, body.physicsBody.rotation, pos, body.physicsBody.size,
               (vec4){1.0f, 1.0f, 1.0f, 1.0f});

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
