#include <stdbool.h>
#include <stdint.h>
#include "physics.h"

void minMaxAABB(vec2 min, vec2 max, AABB aabb)
{
    vec2_sub(min, aabb.position, aabb.halfSize);
    vec2_add(max, aabb.position, aabb.halfSize);
}

bool physicsPointIntersectAABB(vec2 point, AABB aabb)
{
    vec2 min, max;
    minMaxAABB(min, max, aabb);
    return point[0] >= min[0] &&
           point[0] <= max[0] &&
           point[1] >= min[1] &&
           point[1] <= max[1];
}


bool physicsAABBIntersectAABB(AABB a, AABB b)
{
    vec2 min, max;
    minMaxAABB(min, max, minkowskiDifferenceAABB(a, b));
    
    return (min[0] <= 0 && max[0] >= 0 && min[1] <= 0 && max[1] >= 0);
}

AABB minkowskiDifferenceAABB(AABB a, AABB b)
{
    AABB result;
    vec2_sub(result.position, a.position, b.position);
    vec2_add(result.halfSize, a.halfSize, b.halfSize);
    return result;
}


void penetrationVectorAABB(vec2 r, AABB aabb)
{
    vec2 min, max;
    minMaxAABB(min, max, aabb);
    
    float minDist = fabsf(min[0]);
    r[0] = min[0];
    r[1] = 0;
    
    if (fabsf(max[0]) < minDist)
    {
        minDist = fabsf(max[0]);
        r[0] = max[0];
    }
    if (fabsf(min[1]) < minDist)
    {
        minDist = fabsf(min[1]);
        r[0] = 0;
        r[1] = min[1];
    }
    if (fabsf(max[1]) < minDist)
    {
        r[0] = 0;
        r[1] = max[1];
    }
}


void initKinematicBody(KinematicBody2D* body, const vec2 size, const vec2 position)
{
	body->aabb.position[0] = position[0];
	body->aabb.position[1] = position[1];
	body->aabb.halfSize[0] = size[0] * 0.5f;
	body->aabb.halfSize[1] = size[1] * 0.5f;
	body->position[0] = position[0];
	body->position[1] = position[1];
	body->size[0] = size[0];
	body->size[1] = size[1];
	body->velocity[0] = 0.0f;
	body->velocity[1] = 0.0f;
}

void updateKinematicBody2D(KinematicBody2D* body, const float dt)
{
	body->position[0] += body->velocity[0] * dt;
	body->position[1] += body->velocity[1] * dt;
	body->aabb.position[0] = body->position[0];
	body->aabb.position[1] = body->position[1];
}

void moveKinematicBody2D(KinematicBody2D* body, const vec2 direction, const float speed)
{
	body->velocity[0] = direction[0] * speed;
	body->velocity[1] = direction[1] * speed;
}


