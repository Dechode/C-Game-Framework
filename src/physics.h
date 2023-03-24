#include "linmath.h"

typedef struct 
{
    vec2 position;
    vec2 halfSize;
} AABB;

typedef struct 
{
    AABB aabb;
	vec2 size;
	vec2 position;
    vec2 velocity;
} KinematicBody2D;

void initKinematicBody(KinematicBody2D* body, const vec2 size, const vec2 position);
void updateKinematicBody2D(KinematicBody2D* body, const float dt);
void moveKinematicBody2D(KinematicBody2D* body, const vec2 direction, const float speed);

bool physicsPointIntersectAABB(vec2 point, AABB aabb);
bool physicsAABBIntersectAABB(AABB a, AABB b);

AABB minkowskiDifferenceAABB(AABB a, AABB b);
void penetrationVectorAABB(vec2 r, AABB aabb);

void minMaxAABB(vec2 min, vec2 max, AABB aabb);

