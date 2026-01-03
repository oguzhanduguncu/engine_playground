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
    double Pn = 0.0;    // accumulated normal impulse
    double Pt = 0.0;    // accumulated tangent impulse
};
#endif //CONTACT_H
