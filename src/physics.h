#include "array_list.h"
#include "linmath.h"
#include <stddef.h>
#include <stdint.h>

#define MAX_COLLISIONS 1024

typedef struct
{
    vec2 position;
    vec2 halfSize;
} AABB;

typedef enum
{
    CIRCLE = 0,
    RECTANGLE,
	POLYGON,
} CollisionShape;

typedef struct
{
	float rotation;
	vec2 position;
	vec2 size;
} Rect;

typedef struct
{
	vec2 position;
	float sin;
	float cos;
} Transform2D;

typedef struct
{
    CollisionShape collisionShape;
    int8_t kinematic;
    int8_t active;
    AABB aabb;
    vec2 size;
    vec2 position;
    vec2 velocity;
    vec2 acceleration;
    float rotation;
    float mass;
    float invMass;
    float inertia;
    float density;
    float restitution;
    float angularVelocity;
	int32_t id;
} PhysicsBody2D;

typedef struct
{
    int8_t colliding;
    float time;
    vec2 position;
    vec2 normal;
    float depth;
} CollisionInfo;

typedef struct
{
    vec2 gravity;
    float terminalVelocity;
    float physicsFPS;
    uint8_t iterations;
    uint32_t collidingBodyCount;
    int collidingBodies[MAX_COLLISIONS];
    ArrayList *physicsBodies;
} PhysicsState2D;

void initPhysicsState2D(const vec2 gravity, const float terminalVelocity, const int FPS, const uint8_t iterations);

void initPhysicsBody2D(PhysicsBody2D *body, const vec2 size, const vec2 position, const vec2 velocity,
                       const float rotation, const float mass, const float density, const float restitution,
                       const uint8_t kinematic, CollisionShape shape);

int createPhysicsBody2D(const vec2 size, const vec2 position, const vec2 velocity, const float rotation,
                        const float mass, const float density, const float restitution, const uint8_t kinematic,
                        CollisionShape shape);

Transform2D createTransform2D(const vec2 pos, const float angle);

PhysicsBody2D *getPhysicsBody2D(size_t id);
void movePhysicsBody2D(PhysicsBody2D *body, vec2 const amount);
void setVelocityPhysicsBody2D(PhysicsBody2D *body, const vec2 velocity);
void applyForcePhysicsBody2D(PhysicsBody2D *body, const vec2 force);

void updatePhysicsBody2D(PhysicsBody2D *body, const float dt);
void physicsUpdate(const float dt);

void minMaxAABB(vec2 min, vec2 max, AABB aabb);
AABB minkowskiDifferenceAABB(AABB a, AABB b);
void penetrationVectorAABB(vec2 r, AABB aabb);
int physicsPointIntersectAABB(vec2 point, AABB aabb);
int physicsAABBIntersectAABB(AABB a, AABB b);

CollisionInfo isCircleColliding(const float ra, const float rb, const vec2 posA, const vec2 posB);
CollisionInfo isColliding(PhysicsBody2D *bodyA, PhysicsBody2D *bodyB);
void windowCollision(PhysicsBody2D *body, float dt);

void broadPhaseSweep(void);
void narrowPhaseSweep(void);

