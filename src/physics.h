#include "linmath.h"

typedef struct aabb {
    vec2 position;
    vec2 halfSize;
} AABB;

typedef struct body {
    AABB aabb;
    vec2 velocity;
    vec2 acceleration;
} Body;

//void physicsInit(void);
//void physicsUpdate(void);

//size_t physicsBodyCreate(vec2 position, vec2 size);
//Body *physicsBodyGet(size_t index);

bool physicsPointIntersectAABB(vec2 point, AABB aabb);
bool physicsAABBIntersectAABB(AABB a, AABB b);

AABB minkowskiDifferenceAABB(AABB a, AABB b);
void penetrationVectorAABB(vec2 r, AABB aabb);

void minMaxAABB(vec2 min, vec2 max, AABB aabb);

