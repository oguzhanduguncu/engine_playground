//
// Created by oguzh on 31.12.2025.
//

#ifndef BODY_H
#define BODY_H
#include "glm/vec2.hpp"
#include <cstdint>

enum class BodyType {
    Dynamic,
    Static,
    Kinematic
};

using BodyID = uint32_t;

struct Body {
    BodyID id;
    BodyType type;
    glm::vec2 position{0.0, 0.0};
    glm::vec2 velocity {0.0 ,0.0};
    glm::vec2 acceleration {0.0 ,0.0};
    glm::vec2 pseudoVelocity = {0.0, 0.0};

    float invMass {0.0};  // 1/mass, for static bodies invMass = 0 e.g:wall

//    Shape shape;
};

#endif //BODY_H
