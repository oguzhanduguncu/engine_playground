//
// Created by oguzh on 4.03.2026.
//
#include "boid_flock.h"
#include "Integrator.h"
#include <glm/glm.hpp>

void Flock::add_boid(Boid boid)
{
    boids.push_back(std::move(boid));
}

glm::vec2 Flock::separation(const Boid& boid) const
{
    glm::vec2 steer{0.0f, 0.0f};
    int count = 0;
    for (const auto& other : boids) {
        if (&other == &boid) continue;
        glm::vec2 diff = boid.body.position - other.body.position;
        float d = glm::length(diff);
        if (d > 0.0f && d < boid.perception) {
            steer += diff / d;
            ++count;
        }
    }
    if (count > 0) steer /= static_cast<float>(count);
    return steer;
}

glm::vec2 Flock::alignment(const Boid& boid) const
{
    glm::vec2 avg{0.0f, 0.0f};
    int count = 0;
    for (const auto& other : boids) {
        if (&other == &boid) continue;
        float d = glm::length(other.body.position - boid.body.position);
        if (d < boid.perception) {
            avg += other.body.velocity;
            ++count;
        }
    }
    if (count == 0) return glm::vec2{0.0f, 0.0f};
    avg /= static_cast<float>(count);
    if (glm::length(avg) > boid.max_speed)
        avg = glm::normalize(avg) * boid.max_speed;
    return avg - boid.body.velocity;
}

glm::vec2 Flock::cohesion(const Boid& boid) const
{
    glm::vec2 center{0.0f, 0.0f};
    int count = 0;
    for (const auto& other : boids) {
        if (&other == &boid) continue;
        float d = glm::length(other.body.position - boid.body.position);
        if (d < boid.perception) {
            center += other.body.position;
            ++count;
        }
    }
    if (count == 0) return glm::vec2{0.0f, 0.0f};
    center /= static_cast<float>(count);
    glm::vec2 desired = center - boid.body.position;
    if (glm::length(desired) > 0.0f)
        desired = glm::normalize(desired) * boid.max_speed;
    return desired - boid.body.velocity;
}

void Flock::wrap(Boid& boid)
{
    auto& p = boid.body.position;
    if (p.x >  WORLD_HALF_W) p.x = -WORLD_HALF_W;
    if (p.x < -WORLD_HALF_W) p.x =  WORLD_HALF_W;
    if (p.y >  WORLD_HALF_H) p.y = -WORLD_HALF_H;
    if (p.y < -WORLD_HALF_H) p.y =  WORLD_HALF_H;
}

void Flock::step(float dt)
{
    // 1. Compute and accumulate steering forces into acceleration
    for (auto& boid : boids) {
        glm::vec2 sep = separation(boid) * boid.w_separation;
        glm::vec2 ali = alignment(boid)  * boid.w_alignment;
        glm::vec2 coh = cohesion(boid)   * boid.w_cohesion;

        glm::vec2 steering = sep + ali + coh;
        if (glm::length(steering) > boid.max_force)
            steering = glm::normalize(steering) * boid.max_force;

        boid.body.acceleration += steering * boid.body.invMass;
    }

    // 2. Integrate via the engine's semi-implicit Euler, clamp speed, wrap, reset
    for (auto& boid : boids) {
        Integrator::semi_implicit_euler(boid.body, dt);

        if (glm::length(boid.body.velocity) > boid.max_speed)
            boid.body.velocity = glm::normalize(boid.body.velocity) * boid.max_speed;

        wrap(boid);
        boid.body.acceleration = {0.0f, 0.0f};
    }
}
