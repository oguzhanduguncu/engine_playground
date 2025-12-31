//
// Created by oguzh on 31.12.2025.
//

#ifndef BODY_H
#define BODY_H
#include "vec2.h"
#include <cstdint>

using BodyID = uint32_t;

struct Body {
    BodyID id;
    Vec2 position;
    Vec2 velocity;

//    Shape shape;
};

#endif //BODY_H
