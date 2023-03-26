#include "array_list.h"
#include "linmath.h"

typedef struct {
  vec2 position;
  vec2 halfSize;
} AABB;

typedef enum {
  CIRCLE = 0,
  RECTANGLE,
} CollisionShape;

typedef struct {
  CollisionShape collisionShape;
  AABB aabb;
  vec2 size;
  vec2 position;
  vec2 velocity;
  float rotation;
  int active;
} PhysicsBody2D;

typedef struct {
  PhysicsBody2D physicsBody;
  vec2 acceleration;
  float mass;
  float invMass;
  float inertia;
  float density;
  float restitution;
  float angularVelocity;
} RigidBody2D;

typedef struct {
  PhysicsBody2D *bodyA;
  PhysicsBody2D *bodyB;
} CollisionPair;

typedef struct {
  int colliding;
  float time;
  vec2 position;
  vec2 normal;
  CollisionPair pairs;
} CollisionInfo;

typedef struct {
  vec2 gravity;
  float terminalVelocity;
  float physicsFPS;
  int iterations;
  int collidingBodyCount;
  ArrayList *collidingBodies;
  ArrayList *physicsBodies;
} PhysicsState2D;

void initPhysicsState2D(const vec2 gravity, const float terminalVelocity,
                        const int FPS, const int iterations);

void initPhysicsBody2D(PhysicsBody2D *body, const vec2 size,
                       const vec2 position, const vec2 velocity,
                       const float rotation);
void initRigidBody2D(RigidBody2D *body, const vec2 size, const vec2 position,
                     const vec2 velocity, const float rotation,
                     const float mass, const float density,
                     const float restitution);
PhysicsBody2D *createPhysicsBody2D(const vec2 size, const vec2 position,
                                   const vec2 velocity, const float rotation);

void movePhysicsBody2D(PhysicsBody2D *body, vec2 const velocity);
void applyForceRigidBody2D(RigidBody2D *body, const vec2 force);

void updatePhysicsBody2D(PhysicsBody2D *body, const float dt);
void updateRigidBody2D(RigidBody2D *body, const float dt);
void physicsUpdate(const float dt);

void minMaxAABB(vec2 min, vec2 max, AABB aabb);
AABB minkowskiDifferenceAABB(AABB a, AABB b);
void penetrationVectorAABB(vec2 r, AABB aabb);
int physicsPointIntersectAABB(vec2 point, AABB aabb);
int physicsAABBIntersectAABB(AABB a, AABB b);

int isColliding(PhysicsBody2D *bodyA, PhysicsBody2D *bodyB);

void broadPhaseSweep(void);
void narrowPhaseSweep(void);

void windowCollision(PhysicsBody2D *body, const int windowWidth,
                     const int windowHeight, float dt);
