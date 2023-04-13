#include "physics.h"

#include <float.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include "array_list.h"
#include "linmath.h"
#include "renderer.h"

static PhysicsState2D physicsState;

float dotProduct(const float *a, const float *b, int n)
{
    float result = 0;
    for (int i = 0; i < n; i++)
    {
        result += a[i] * b[i];
    }
    return result;
}

void initPhysicsState2D(const vec2 gravity, const float terminalVelocity, const int FPS, const uint8_t iterations)
{
    physicsState.physicsBodies = arrayListCreate(sizeof(PhysicsBody2D), 1);
    physicsState.collidingBodyCount = 0;
    physicsState.physicsFPS = FPS;
    physicsState.iterations = iterations;
    physicsState.terminalVelocity = terminalVelocity;
    vec2_dup(physicsState.gravity, gravity);

    for (int i = 0; i < MAX_COLLISIONS; i++)
    {
        physicsState.collidingBodies[i] = -1;
    }
}

void initPhysicsBody2D(PhysicsBody2D *body, const vec2 size, const vec2 position, const vec2 velocity,
                       const float rotation, const float mass, const float density, const float restitution,
                       const uint8_t kinematic, CollisionShape shape)
{
    vec2_dup(body->aabb.position, position);
    vec2_dup(body->position, position);
    vec2_dup(body->size, size);
    vec2_dup(body->velocity, velocity);
    body->acceleration[0] = 0.0f;
    body->acceleration[1] = 0.0f;
    body->aabb.halfSize[0] = size[0] * 0.5f;
    body->aabb.halfSize[1] = size[1] * 0.5f;
    body->rotation = rotation;
    body->active = 1;
    body->mass = mass;
    body->invMass = 1 / mass;
    body->density = density;
    body->inertia = 0.75f * size[0] * size[1] * ((size[0] * size[0]) + (size[1] * size[1])) * body->density;
    body->restitution = restitution;
    body->angularVelocity = 0.0f;
    body->kinematic = kinematic;
    body->collisionShape = shape;

    if (kinematic)
    {
        setVelocityPhysicsBody2D(body, velocity);
    }
}

int createPhysicsBody2D(const vec2 size, const vec2 position, const vec2 velocity, const float rotation,
                        const float mass, const float density, const float restitution, const uint8_t kinematic,
                        CollisionShape shape)
{
    size_t id = physicsState.physicsBodies->len;
    for (size_t i = 0; i < physicsState.physicsBodies->len; ++i)
    {
        PhysicsBody2D *body = arrayListGet(physicsState.physicsBodies, i);
        if (!body->active)
        {
            id = i;
            break;
        }
    }
    if (id == physicsState.physicsBodies->len)
    {
        if (arrayListAppend(physicsState.physicsBodies, &(PhysicsBody2D){0}) == (size_t)-1)
        {
            printf("Error! Could not append physics body to array list\n");
            return -1;
        }
    }

    PhysicsBody2D *body = (PhysicsBody2D *)arrayListGet(physicsState.physicsBodies, id);

    initPhysicsBody2D(body, size, position, velocity, rotation, mass, density, restitution, kinematic, shape);
	body->id = id;
    return id;
}

PhysicsBody2D *getPhysicsBody2D(size_t id)
{
    return (PhysicsBody2D *)arrayListGet(physicsState.physicsBodies, id);
}

void updatePhysicsBody2D(PhysicsBody2D *body, const float dt)
{
    if (!body->kinematic)
    {
        body->velocity[0] += body->acceleration[0] * dt;
        body->velocity[1] += body->acceleration[1] * dt;
    }
    vec2 newPos = {0};
    vec2_add(newPos, body->position, (vec2){body->velocity[0] * dt, body->velocity[1] * dt});
    vec2_dup(body->position, newPos);
    vec2_dup(body->aabb.position, body->position);
}

void setVelocityPhysicsBody2D(PhysicsBody2D *body, const vec2 velocity)
{
    if (!body->kinematic)
    {
        printf("Warning! Do not set rigidbody velocity, apply force instead!\n");
        return;
    }
    vec2_dup(body->velocity, velocity);
}

void movePhysicsBody2D(PhysicsBody2D *body, const vec2 amount)
{
    vec2_add(body->position, body->position, amount);
}

void applyForcePhysicsBody2D(PhysicsBody2D *body, const vec2 force)
{
    body->acceleration[0] += force[0] * body->invMass;
    body->acceleration[1] += force[1] * body->invMass;
}

void minMaxAABB(vec2 min, vec2 max, AABB aabb)
{
    vec2_sub(min, aabb.position, aabb.halfSize);
    vec2_add(max, aabb.position, aabb.halfSize);
}

int physicsPointIntersectAABB(vec2 point, AABB aabb)
{
    vec2 min, max;
    minMaxAABB(min, max, aabb);
    return point[0] >= min[0] && point[0] <= max[0] && point[1] >= min[1] && point[1] <= max[1];
}

int physicsAABBIntersectAABB(AABB a, AABB b)
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

CollisionInfo rayIntersectAABB(const vec2 position, const vec2 magnitude, AABB aabb)
{
    CollisionInfo collision = {0};
    vec2 min, max;
    minMaxAABB(min, max, aabb);

    float lastEntry = -INFINITY;
    float firstExit = INFINITY;

    for (uint8_t i = 0; i < 2; i++)
    {
        if (magnitude[i] != 0)
        {
            float t1 = (min[i] - position[i]) / magnitude[i];
            float t2 = (max[i] - position[i]) / magnitude[i];

            lastEntry = fmaxf(lastEntry, fminf(t1, t2));
            firstExit = fminf(firstExit, fmaxf(t1, t2));
        }
        else if (position[i] <= min[i] || position[i] >= max[i])
        {
            return collision;
        }
    }

    if (firstExit > lastEntry && firstExit > 0 && lastEntry < 1)
    {
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

// void resolveCollision(PhysicsBody2D *bodyA, PhysicsBody2D *bodyB, const vec2 normal, float depth)
//{
//     vec2 relativeVelocity;
//     vec2_sub(relativeVelocity, bodyA->velocity, bodyB->velocity);
//     float dot = dotProduct(relativeVelocity, normal, 2);
//
//     if (dot > 0.0f)
//     {
//         return;
//     }
//     float e = fminf(bodyA->restitution, bodyB->restitution);
//     float j = (1.0f + e) * dot;
//     j /= bodyA->mass + bodyB->mass;
//
//     vec2 impulse;
//     impulse[0] = normal[0] * j;
//     impulse[1] = normal[1] * j;
//
//     bodyA->velocity[0] += -impulse[0] * bodyA->invMass;
//     bodyA->velocity[1] += -impulse[1] * bodyA->invMass;
//
//     bodyB->velocity[0] += impulse[0] * bodyB->invMass;
//     bodyB->velocity[1] += impulse[1] * bodyB->invMass;
// }

static void projectVertices(float *min, float *max, const vec2 vertexPositions[], const uint32_t verticeCount,
                            const vec2 normal)
{
    *min = FLT_MIN;
    *max = FLT_MAX;

    for (int i = 0; i < verticeCount; i++)
    {
        vec2 v = {0};
        vec2_dup(v, vertexPositions[i]);
        float proj = dotProduct(v, normal, 2);

        if (proj < *min)
            *min = proj;
        if (proj > *max)
            *max = proj;
    }
}

CollisionInfo isPolygonColliding(vec2 vertexPositionsA[], const uint32_t vertexCountA, vec2 vertexPositionsB[],
                                 const uint32_t vertexCountB)
{
    CollisionInfo collision = {0};

    for (int i = 0; i < vertexCountA; i++)
    {
        vec2 va = {0};
        vec2 vb = {0};
        vec2_dup(va, vertexPositionsA[i]);
        vec2_dup(vb, vertexPositionsA[i] + 1 % vertexCountA);

        vec2 edge = {0};
        vec2_sub(edge, vb, va);

        vec2 normal = {0};
        normal[0] = -edge[1];
        normal[1] = edge[0];

        float minA, minB, maxA, maxB;
        projectVertices(&minA, &maxA, vertexPositionsA, vertexCountA, normal);
        projectVertices(&minB, &maxB, vertexPositionsB, vertexCountB, normal);

        if (minA >= maxB || minB >= maxA)
        {
            collision.colliding = 0;
			return collision;
        }
    }

    for (int i = 0; i < vertexCountB; i++)
    {
        vec2 va = {0};
        vec2 vb = {0};
        vec2_dup(va, vertexPositionsB[i]);
        vec2_dup(vb, vertexPositionsB[i] + 1 % vertexCountB);

        vec2 edge = {0};
        vec2_sub(edge, vb, va);

        vec2 normal = {0};
        normal[0] = -edge[1];
        normal[1] = edge[0];

        float minA, minB, maxA, maxB;
        projectVertices(&minA, &maxA, vertexPositionsA, vertexCountA, normal);
        projectVertices(&minB, &maxB, vertexPositionsB, vertexCountB, normal);

        if (minA >= maxB || minB >= maxA)
        {
            collision.colliding = 0;
			return collision;
        }
    }

	collision.colliding = 1;
    return collision;
}

CollisionInfo isRectColliding(const Rect *rectA, const Rect *rectB)
{
	CollisionInfo collision = {0};
	
	vec2 transformedVertexPositionsA[4];
	vec2 transformedVertexPositionsB[4];
	for (int i = 0; i < 4; i++)
	{
		vec2 tmp;
		// Get transformed vertices
		vec2_dup(transformedVertexPositionsA[i], tmp);
	}

	collision = isPolygonColliding(transformedVertexPositionsA, 4, transformedVertexPositionsB, 4);

	return collision;
}

CollisionInfo isCircleColliding(const float ra, const float rb, const vec2 posA, const vec2 posB)
{
    CollisionInfo collision = {0};
    vec2 tmp = {0};
    vec2_sub(tmp, posA, posB);
    //vec2_len(tmp);
    float distance = vec2_len(tmp);
    float radii = ra + rb;

    if (distance >= radii)
    {
        collision.colliding = 0;
        return collision;
    }

    vec2_sub(collision.normal, posA, posB);
    if (vec2_len(collision.normal) > 1.0f)
    {
        vec2_norm(collision.normal, collision.normal);
    }

    collision.depth = radii - distance;
    collision.colliding = 1;
    // collision.position[0] = ?;
    // collision.position[1] = ?;
    return collision;
}

CollisionInfo isColliding(PhysicsBody2D *bodyA, PhysicsBody2D *bodyB)
{
    CollisionInfo collision = {0};
    if (bodyA->collisionShape == CIRCLE && bodyB->collisionShape == CIRCLE)
    {
        collision = isCircleColliding(bodyA->size[0] * 0.5f, bodyB->size[0] * 0.5f, bodyA->position, bodyB->position);
    }
	if (bodyA->collisionShape == RECTANGLE && bodyB->collisionShape == RECTANGLE)
	{
		Rect rectA = {
			.size[0] = bodyA->size[0],
			.size[1] = bodyA->size[1],

			.position[0] = bodyA->position[0],
			.position[1] = bodyA->position[1],

			.rotation = bodyA->rotation,
		};

		Rect rectB = {
			.size[0] = bodyB->size[0],
			.size[1] = bodyB->size[1],

			.position[0] = bodyB->position[0],
			.position[1] = bodyB->position[1],

			.rotation = bodyB->rotation,
		};
		
		collision = isRectColliding(&rectA, &rectB);
	}
    return collision;
}

void broadPhaseSweep(void)
{
    // TODO
	physicsState.collidingBodyCount = 0;
	for (int i = 0; i < (int)physicsState.physicsBodies->len; i++)
	{
		PhysicsBody2D *body = getPhysicsBody2D(i);
		if (body == NULL)
		{
			printf("Could not get physics body with id %d\n", i);
			continue;
		}
		physicsState.collidingBodies[i] = body->id;
		physicsState.collidingBodyCount++;
	}
}

// TODO
void narrowPhaseSweep(void)
{
    for (size_t i = 0; i < physicsState.collidingBodyCount - 1; i++)
    {
		int aID = physicsState.collidingBodies[i];
        PhysicsBody2D *bodyA = getPhysicsBody2D(aID);
        for (size_t j = i + 1; j < physicsState.collidingBodyCount; j++)
        {
            int bID = physicsState.collidingBodies[j];
			PhysicsBody2D *bodyB = getPhysicsBody2D(bID);

            if (!bodyA || !bodyB)
			{
				printf("Physics bodies are null!\n");
				continue;
			}

            CollisionInfo hit = isColliding(bodyA, bodyB);
            if (hit.colliding)
            {
                // kinematic only?
                vec2 amount = {0};
                amount[0] = hit.normal[0] * hit.depth * 0.5f;
                amount[1] = hit.normal[1] * hit.depth * 0.5f;
                movePhysicsBody2D(bodyA, amount);
                amount[0] *= -1;
                amount[1] *= -1;
                movePhysicsBody2D(bodyB, amount);
            }
        }
    }
}

void windowCollision(PhysicsBody2D *body, float dt)
{
    vec2 windowSize = {0};
    getWindowSize(windowSize);

    if (body->position[0] - 0.5f * body->size[0] < 0 || body->position[0] + 0.5f * body->size[0] > windowSize[0])
    {
        body->velocity[0] *= -1;
        body->position[0] += body->velocity[0] * dt;
    }
    if (body->position[1] - 0.5f * body->size[1] < 0 || body->position[1] + 0.5f * body->size[1] > windowSize[1])
    {
        body->velocity[1] *= -1;
        body->position[1] += body->velocity[1] * dt;
    }
}

void physicsUpdate(const float dt)
{
    PhysicsBody2D *body;

    for (size_t bodyID = 0; bodyID < physicsState.physicsBodies->len; bodyID++)
    {
        body = (PhysicsBody2D *)arrayListGet(physicsState.physicsBodies, bodyID);
        if (!body->active)
        {
            continue;
        }

        if (!body->kinematic)
        {
            body->velocity[0] += physicsState.gravity[0];
            body->velocity[1] += physicsState.gravity[1];
            if (vec2_len(body->velocity) > physicsState.terminalVelocity)
            {
                vec2 vel;
                vec2_norm(vel, body->velocity);
                vel[0] *= physicsState.terminalVelocity;
                vel[1] *= physicsState.terminalVelocity;
                vec2_dup(body->velocity, vel);
            }
        }

        updatePhysicsBody2D(body, dt);
        windowCollision(body, dt);
    }
    // TODO
    // Broad phase sweep
    broadPhaseSweep();
    // narrow phase sweep
    narrowPhaseSweep();
}
