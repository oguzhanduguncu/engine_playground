//
// Created by oguzh on 31.12.2025.
//

#ifndef WORLD_H
#define WORLD_H
#include "physics_world.h"
#include <vector>
#include "body.h"
#include "contact_manifold.h"

struct World {
    PhysicsWorld physics;
    std::vector<Body> bodies;
    std::vector<ContactManifold> manifolds;

    void step(double dt);
};




#endif //WORLD_H
