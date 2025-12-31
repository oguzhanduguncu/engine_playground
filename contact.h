//
// Created by oguzh on 31.12.2025.
//

#ifndef CONTACT_H
#define CONTACT_H
#include "vec2.h"

struct ContactPoint {
    Vec2 position;     // world space
    Vec2 normal;       // normalized
    float penetration; // <= 0 for touching
};
#endif //CONTACT_H
