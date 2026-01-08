//
// Created by oguzh on 31.12.2025.
//

#ifndef CONTACT_H
#define CONTACT_H
#include "glm/vec2.hpp"

struct ContactPoint {
    glm::vec2 position;     // world space
    glm::vec2 normal;       // normalized
    float penetration; // <= 0 for touching
    float Pn = 0.0f;    // accumulated normal impulse
    float Pt = 0.0f;    // accumulated tangent impulse
};
#endif //CONTACT_H
