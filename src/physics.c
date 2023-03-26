#include "physics.h"
#include "array_list.h"
#include "linmath.h"
#include <math.h>
#include <stdint.h>
#include <stdio.h>

#define INITIAL_PHYSICS_BODY_COUNT 10
static PhysicsState2D physicsState;

float dotProduct(const vec2 a, const vec2 b, int n) {
  float result = 0;
  for (int i = 0; i < n; i++) {
    result += a[i] * b[i];
  }
  return result;
}

void initPhysicsState2D(const vec2 gravity, const float terminalVelocity,
                        const int FPS, const int iterations) {
  physicsState.physicsBodies = arrayListCreate(sizeof(PhysicsBody2D), 0);
  physicsState.collidingBodies =
      arrayListCreate(sizeof(CollisionPair), INITIAL_PHYSICS_BODY_COUNT * 2);
  physicsState.collidingBodyCount = 0;
  physicsState.physicsFPS = FPS;
  physicsState.iterations = iterations;
  physicsState.terminalVelocity = terminalVelocity;
  vec2_dup(physicsState.gravity, gravity);
}

void initPhysicsBody2D(PhysicsBody2D *body, const vec2 size,
                       const vec2 position, const vec2 velocity,
                       const float rotation) {
  vec2_dup(body->aabb.position, position);
  vec2_dup(body->position, position);
  vec2_dup(body->size, size);
  vec2_dup(body->velocity, (vec2){0.0f, 0.0f});
  // vec2_dup(body->acceleration, (vec2){0.0f, 0.0f});
  // CollisionInfo colInfo = {0};
  // body->collisionInfo = colInfo;
  body->aabb.halfSize[0] = size[0] * 0.5f;
  body->aabb.halfSize[1] = size[1] * 0.5f;
  body->rotation = rotation;
  movePhysicsBody2D(body, velocity);
}

void initRigidBody2D(RigidBody2D *body, const vec2 size, const vec2 position,
                     const vec2 velocity, const float rotation,
                     const float mass, const float density,
                     const float restitution) {
  initPhysicsBody2D(&body->physicsBody, size, position, velocity, rotation);
  vec2_dup(body->acceleration, (vec2){0.0f, 0.0f});
  body->mass = mass;
  body->invMass = 1 / mass;
  body->density = density;
  body->inertia = 0.75f * size[0] * size[1] *
                  ((size[0] * size[0]) + (size[1] * size[1])) * body->density;
  body->restitution = restitution;
  body->angularVelocity = 0.0f;
  movePhysicsBody2D(&body->physicsBody, velocity);
}

// RigidBody2D* createRigidBody2D(const vec2 size, const vec2 position, const
// vec2 velocity,
//                                const float rotation, const float mass, const
//                                float density, float restitution)
// {
//   size_t id = physicsState.rigidBodies->len;
//   for (size_t i = 0; i < physicsState.rigidBodies->len; ++i)
//   {
//     RigidBody2D* body = arrayListGet(physicsState.rigidBodies, i);
//     if (!body->physicsBody.active)
//     {
//       id = i;
//       break;
//     }
//   }
//   if (id == physicsState.rigidBodies->len)
//   {
//     if (arrayListAppend(physicsState.physicsBodies, &(RigidBody2D){0}) ==
//     (size_t) - 1)
//     {
//       printf("Error! Could not append rigid body to array list\n");
//       return NULL;
//     }
//   }
//   RigidBody2D* body = arrayListGet(physicsState.rigidBodies, id);
//   initRigidBody2D(body, size, position, velocity, rotation, mass, density,
//   restitution); return body;
// }

PhysicsBody2D *createPhysicsBody2D(const vec2 size, const vec2 position,
                                   const vec2 velocity, const float rotation) {
  size_t id = physicsState.physicsBodies->len;
  for (size_t i = 0; i < physicsState.physicsBodies->len; ++i) {
    PhysicsBody2D *body = arrayListGet(physicsState.physicsBodies, i);
    if (!body->active) {
      id = i;
      break;
    }
  }
  if (id == physicsState.physicsBodies->len) {
    if (arrayListAppend(physicsState.physicsBodies, &(PhysicsBody2D){0}) ==
        (size_t)-1) {
      printf("Error! Could not append physics body to array list\n");
      return NULL;
    }
  }
  PhysicsBody2D *body = arrayListGet(physicsState.physicsBodies, id);
  initPhysicsBody2D(body, size, position, velocity, rotation);
  return body;
}

void updatePhysicsBody2D(PhysicsBody2D *body, const float dt) {
  vec2 newPos = {0};
  vec2_add(newPos, body->position,
           (vec2){body->velocity[0] * dt, body->velocity[1] * dt});
  vec2_dup(body->position, newPos);
  vec2_dup(body->aabb.position, body->position);
}

void updateRigidBody2D(RigidBody2D *body, const float dt) {
  body->physicsBody.velocity[0] += body->acceleration[0] * dt;
  body->physicsBody.velocity[1] += body->acceleration[1] * dt;
  updatePhysicsBody2D(&body->physicsBody, dt);
}

void movePhysicsBody2D(PhysicsBody2D *body, const vec2 velocity) {
  vec2_dup(body->velocity, velocity);
}

void applyForceRigidBody2D(RigidBody2D *body, const vec2 force) {
  body->acceleration[0] += force[0] * body->invMass;
  body->acceleration[1] += force[1] * body->invMass;
}

void minMaxAABB(vec2 min, vec2 max, AABB aabb) {
  vec2_sub(min, aabb.position, aabb.halfSize);
  vec2_add(max, aabb.position, aabb.halfSize);
}

int physicsPointIntersectAABB(vec2 point, AABB aabb) {
  vec2 min, max;
  minMaxAABB(min, max, aabb);
  return point[0] >= min[0] && point[0] <= max[0] && point[1] >= min[1] &&
         point[1] <= max[1];
}

int physicsAABBIntersectAABB(AABB a, AABB b) {
  vec2 min, max;
  minMaxAABB(min, max, minkowskiDifferenceAABB(a, b));

  return (min[0] <= 0 && max[0] >= 0 && min[1] <= 0 && max[1] >= 0);
}

AABB minkowskiDifferenceAABB(AABB a, AABB b) {
  AABB result;
  vec2_sub(result.position, a.position, b.position);
  vec2_add(result.halfSize, a.halfSize, b.halfSize);
  return result;
}

void penetrationVectorAABB(vec2 r, AABB aabb) {
  vec2 min, max;
  minMaxAABB(min, max, aabb);

  float minDist = fabsf(min[0]);
  r[0] = min[0];
  r[1] = 0;

  if (fabsf(max[0]) < minDist) {
    minDist = fabsf(max[0]);
    r[0] = max[0];
  }
  if (fabsf(min[1]) < minDist) {
    minDist = fabsf(min[1]);
    r[0] = 0;
    r[1] = min[1];
  }
  if (fabsf(max[1]) < minDist) {
    r[0] = 0;
    r[1] = max[1];
  }
}

CollisionInfo rayIntersectAABB(const vec2 position, const vec2 magnitude,
                               AABB aabb) {
  CollisionInfo collision = {0};
  vec2 min, max;
  minMaxAABB(min, max, aabb);

  float lastEntry = -INFINITY;
  float firstExit = INFINITY;

  for (uint8_t i = 0; i < 2; i++) {
    if (magnitude[i] != 0) {
      float t1 = (min[i] - position[i]) / magnitude[i];
      float t2 = (max[i] - position[i]) / magnitude[i];

      lastEntry = fmaxf(lastEntry, fminf(t1, t2));
      firstExit = fminf(firstExit, fmaxf(t1, t2));
    } else if (position[i] <= min[i] || position[i] >= max[i]) {
      return collision;
    }
  }

  if (firstExit > lastEntry && firstExit > 0 && lastEntry < 1) {
    collision.position[0] = position[0] + magnitude[0] * lastEntry;
    collision.position[1] = position[1] + magnitude[1] * lastEntry;
    collision.colliding = 1;
    collision.time = lastEntry;

    float dx = collision.position[0] - aabb.position[0];
    float dy = collision.position[1] - aabb.position[1];
    float px = aabb.halfSize[0] - fabsf(dx);
    float py = aabb.halfSize[1] - fabsf(dy);

    if (px < py)
      collision.normal[0] = (dx > 0) - (dx < 0);
    else
      collision.normal[1] = (dy > 0) - (dy < 0);
  }
  return collision;
}

void resolveRigidBodyCollision(RigidBody2D *bodyA, RigidBody2D *bodyB,
                               const vec2 normal, float depth) {
  vec2 relativeVelocity;
  vec2_sub(relativeVelocity, bodyA->physicsBody.velocity,
           bodyB->physicsBody.velocity);
  float dot = dotProduct(relativeVelocity, normal, 2);

  if (dot > 0.0f) {
    return;
  }
  float e = fminf(bodyA->restitution, bodyB->restitution);
  float j = (1.0f + e) * dot;
  j /= bodyA->mass + bodyB->mass;

  vec2 impulse;
  impulse[0] = normal[0] * j;
  impulse[1] = normal[1] * j;

  bodyA->physicsBody.velocity[0] += -impulse[0] * bodyA->invMass;
  bodyA->physicsBody.velocity[1] += -impulse[1] * bodyA->invMass;

  bodyB->physicsBody.velocity[0] += impulse[0] * bodyB->invMass;
  bodyB->physicsBody.velocity[1] += impulse[1] * bodyB->invMass;
}

int isColliding(PhysicsBody2D *bodyA, PhysicsBody2D *bodyB) { return 0; }

void broadPhaseSweep(void) {}

static void collisionResponseSweep(RigidBody2D *bodyA, RigidBody2D *bodyB,
                                   const float dt) {
  return;
}

static void collisionResponseStationary(PhysicsBody2D *body,
                                        PhysicsBody2D *staticBody) {
  if (physicsAABBIntersectAABB(body->aabb, staticBody->aabb)) {
    AABB aabb = minkowskiDifferenceAABB(staticBody->aabb, body->aabb);
    vec2 penetrationVector;
    penetrationVectorAABB(penetrationVector, aabb);
    vec2_add(body->position, body->position, penetrationVector);
  }
}

void narrowPhaseSweep(void) {
  for (size_t i = 0; i < physicsState.collidingBodyCount; i++) {
    CollisionPair *pair =
        (CollisionPair *)arrayListGet(physicsState.collidingBodies, i);
    physicsAABBIntersectAABB(pair->bodyA->aabb, pair->bodyB->aabb);

    if (isColliding(pair->bodyA, pair->bodyB)) {
      collisionResponseStationary(pair->bodyA, pair->bodyB);
    }
  }
}

// int collisionCheckCCD(PhysicsBody2D* bodyA, PhysicsBody2D* bodyB)
//{
//   AABB sumAABB;
//   vec2 magnitude;
//   vec2_sub(magnitude, bodyA->position, bodyB->position);
//   CollisionInfo hit = rayIntersectAABB(bodyB->position, magnitude, sumAABB);
//   return hit.colliding;
// }

void windowCollision(PhysicsBody2D *body, const int windowWidth,
                     const int windowHeight, float dt) {
  float speed = vec2_len(body->velocity);
  if (body->position[0] - 0.5f * body->size[0] < 0 ||
      body->position[0] + 0.5f * body->size[0] > windowWidth) {
    body->velocity[0] *= -1;
    body->position[0] += body->velocity[0] * dt;
  }
  if (body->position[1] - 0.5f * body->size[1] < 0 ||
      body->position[1] + 0.5f * body->size[1] > windowHeight) {
    body->velocity[1] *= -1;
    body->position[1] += body->velocity[1] * dt;
  }
}
