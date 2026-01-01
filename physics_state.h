//
// Created by oguzh on 20.12.2025.
//

#ifndef PHYSICS_STATE_H
#define PHYSICS_STATE_H
#include "vec2.h"

struct PhysicsState {
    Vec2 acceleration2d {9.7 , 9.7};
    Vec2 velocity2d {0.0, 0.0};
    Vec2 position2d {0.0, 0.0};
};

#endif //PHYSICS_STATE_H
