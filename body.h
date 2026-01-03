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
    Vec2 position {0.0 ,0.0};
    Vec2 velocity {0.0 ,0.0};
    Vec2 acceleration {0.0 ,0.0};
    Vec2 pseudoVelocity = {0.0, 0.0};

    double invMass {0.0};  // 1/mass, for static bodies invMass = 0 e.g:wall

//    Shape shape;
};

#endif //BODY_H
