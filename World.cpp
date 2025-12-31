//
// Created by oguzh on 31.12.2025.
//

#include "World.h"


void World::step(double dt) {
    manifolds.clear();

    physics.step(dt);

    if (physics.getHit().hit()) {
        const HitInfo& h = physics.getHit();

        ContactManifold m;
        m.a = 0;
        m.b = 1;

        m.pointCount = 1;
        m.points[0].position = h.contactPoint();
        m.points[0].normal   = h.contactNormal();
        m.points[0].penetration = 0.0;

        manifolds.push_back(m);
    }
}
