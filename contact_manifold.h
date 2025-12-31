//
// Created by oguzh on 31.12.2025.
//

#ifndef CONTACT_MANIFOLD_H
#define CONTACT_MANIFOLD_H
#include "body.h"
#include "contact.h"

struct ContactManifold {
    static constexpr size_t MAX_POINTS = 2;
    BodyID a;
    BodyID b;

    ContactPoint points[MAX_POINTS];
    int pointCount;
};
#endif //CONTACT_MANIFOLD_H
