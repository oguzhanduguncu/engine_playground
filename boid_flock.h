//
// Created by oguzh on 4.03.2026.
//

#ifndef ENGINELOOP_BOID_FLOCK_H
#define ENGINELOOP_BOID_FLOCK_H
#include <vector>
#include "boid.h"
#include <glm/vec2.hpp>

class Flock
{
public:
    static constexpr float WORLD_HALF_W = 10.0f;
    static constexpr float WORLD_HALF_H = 7.5f;

    void add_boid(Boid boid);

    // Called by PhysicsWorld::fixed_step — computes steering then integrates
    // via Integrator::semi_implicit_euler, then clamps speed and wraps.
    void step(float dt);

    const std::vector<Boid>& getBoids() const { return boids; }

private:
    glm::vec2 separation(const Boid& boid) const;
    glm::vec2 alignment(const Boid& boid) const;
    glm::vec2 cohesion(const Boid& boid) const;
    void wrap(Boid& boid);

    std::vector<Boid> boids;
};
#endif //ENGINELOOP_BOID_FLOCK_H
