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


//void physicsInit(void) {
//    state.bodyList = arrayListCreate(sizeof(Body), 0);
//}

//void physicsUpdate(void){
//    Body *body;
//
//    for (uint32_t i = 0; i < state.bodyList->len; ++i) {
//        body = arrayListGet(state.bodyList, i);
//        body->velocity[0] += body->acceleration[0] * globalState.time.delta;
//        body->velocity[1] += body->acceleration[1] * globalState.time.delta;
//        body->aabb.position[0] += body->velocity[0] * globalState.time.delta;
//        body->aabb.position[1] += body->velocity[1] * globalState.time.delta;
//    }
//}

//size_t physicsBodyCreate(vec2 position, vec2 size) {
//    Body body = {
//        .aabb = {
//            .position = { position[0], position[1] },
//            .halfSize = { size[0], size[1] },
//        },
//        .velocity = { 0, 0 },
//    };
//
//    if (arrayListAppend(state.bodyList, &body) == (size_t)-1)
//        ERROR_EXIT("Could not append body to list\n");
//
//    return state.bodyList->len - 1;
//}

//Body *physicsBodyGet(size_t index) {
//    return arrayListGet(state.bodyList, index);
//}

