//
// Created by oguzh on 20.12.2025.
//
// Simulation for fixed timestep
#include "physics_world.h"

#include <algorithm>

#include <cmath>
#include <complex>
#include <iostream>
#include <limits>
#include <vector>
#include <ranges>

#include "glm/glm.hpp"

#include "Integrator.h"
#include "contact_manifold.h"

PhysicsWorld::PhysicsWorld(const float fixed_dt_seconds)
    : m_fixed_dt(fixed_dt_seconds)
{
}

void PhysicsWorld::update_kinematics(const float dt)
{
    for (Body &b: bodies) {
        if (b.type == BodyType::Kinematic) {
            b.position += b.velocity * dt;
        }
    }
}

inline float bottom(const Body& b) {
    return b.position.y - b.halfHeight;
}

bool PhysicsWorld::collidesWithGround(const Body& b) {
    return b.position.y <= GROUND_Y;
}

void PhysicsWorld::resolveGroundPenetration(Body& b) {
    float penetration = GROUND_Y - bottom(b);
    if (penetration > 0.0f) {
        b.position.y += penetration;
    }
}

void resolveGroundVelocity(Body& b) {
    if (b.velocity.y < 0.0f) {
        b.velocity.y = 0.0f;
    }
}

void PhysicsWorld::solveY(Body& b, float dt) {
    if (collidesWithGround(b)) {
        resolveGroundPenetration(b);
        resolveGroundVelocity(b);
        b.onGround = true;
    } else {
        b.onGround = false;
    }
    Integrator::semi_implicit_euler(b,dt);
}


void PhysicsWorld::update(const float frame_dt_seconds)
{
    update_kinematics(frame_dt_seconds);
    m_accumulator += frame_dt_seconds;

    while (m_accumulator >= m_fixed_dt) {
        fixed_step(m_fixed_dt);
        m_accumulator -= m_fixed_dt;
        ++m_steps;
    }
}

void PhysicsWorld::fixed_step(float dt)
{
    step_bodies_with_ccd(dt, manifolds);
    solve_contacts(dt, 0.0);
    solve_split_impulse(dt);
    integrate_pseudo(dt);
}

struct TOIResult
{
    bool hit = false;
    float t = 0.0;
};

static TOIResult compute_toi_1d(const float x0, const float v0, const float a,
                                const float dt) {

    if (std::abs(v0) < PhysicsWorld::eps && std::abs(x0) < PhysicsWorld::slop) {
        return { false, 0.0}; // no hit, resting contact
    }

    // a == 0 case
    if (std::abs(a) < PhysicsWorld::eps) {
        if (std::abs(v0) < PhysicsWorld::eps)
            return { false, 0.0 };   // prevents 0/0

        float t = -x0 / v0;
        if (t >= 0.0 && t <= dt)
            return { true, t };

        return { false, 0.0 };
    }

    TOIResult r;

    if (a == 0.0) {
        if (v0 < 0.0)
            return r;
        if (const float t = -x0 / v0; t >= 0.0 && t <= dt) {
            r.hit = true;
            r.t = t;
        }
        return r;
    }

    const float A = 0.5f * a;
    const float B = v0;
    const float C = x0;

    float disc = B*B - 4*A*C;
    if (disc < 0.0 || !std::isfinite(disc))
        return {false, 0.0};

    float s = std::sqrt(disc);
    float t1 = (-B + s) / (2 * A);
    float t2 = (-B - s) / (2 * A);

    if (!std::isfinite(t1) || !std::isfinite(t2))
        return {false, 0.0};

    float t_hit = std::numeric_limits<float>::infinity();
    if (t1 > 0.0 && t1 <= dt) t_hit = t1;
    if (t2 > 0.0 && t2 <= dt) t_hit = std::min(t_hit, t2);

    if (std::isfinite(t_hit)) {
        r.hit = true;
        r.t = t_hit;
    }
    return r;
}

void PhysicsWorld::step_bodies_with_ccd(
    const float dt, std::vector<ContactManifold> &contact_manifolds)
{
    contact_manifolds.clear();

    auto boxBodies = bodies | std::views::filter(
    [](const Body& b) {
            return b.shape.type == Type::box;
    });

    auto planeBodies = bodies | std::views::filter(
    [](const Body& b) {
        return b.shape.type == Type::plane;
    });

    auto staticBodies = bodies | std::views::filter(
    [](const Body& b) {
        return b.type == BodyType::Static;
    });

    auto kinematicBodies = bodies | std::views::filter(
    [](const Body& b) {
        return b.type == BodyType::Kinematic;
    });

    Body wall;

    for (Body& kinematics : kinematicBodies)
    {
        wall = kinematics;
    }

 //   Body &wall = bodies[1]; // Kinematic or Static

    for (Body &b: bodies) {
        if (b.type != BodyType::Dynamic)
            continue;

        std::cout << "step=" << m_steps << " xA=" << b.position.x
                << " xB=" << wall.position.x << " yA=" << b.position.y
                << " yB=" << wall.position.y << " vA=" << b.velocity.x
                << " vB=" << wall.velocity.x << "\n";

        // --- RELATIVE MOTION ---
        const float x0 = b.position.x - wall.position.x;
        const float v0 = b.velocity.x - wall.velocity.x;
        const float a = b.acceleration.x;

        if (!std::isfinite(x0) || !std::isfinite(v0)) {
            std::cout << "INVALID RELATIVE STATE\n";
            continue;
        }

        auto toi = compute_toi_1d(x0, v0, a, dt);

        if (toi.hit) {
            const float t = toi.t;

            // integrate dynamic until TOI
            b.position.x += b.velocity.x * t + 0.5 * a * t * t;
            b.velocity.x += a * t;

            b.velocity.y += b.acceleration.y * t;
            b.position.y += b.velocity.y * t;

            // --- CCD contact manifold ---
            ContactManifold m;
            m.bodyA = b.id;
            m.bodyB = wall.id;
            m.pointCount = 1;
            m.points[0].normal = {-1.0, 0.0};
            m.points[0].position = {wall.position.x, b.position.y};
            m.points[0].penetration = 0.0;

            merge_manifold(contact_manifolds, m);

            // remaining time
            float remaining = dt - t;
            Integrator::semi_implicit_euler(b, remaining);
        } else {
            Integrator::semi_implicit_euler(b, dt);
        }

        solveY(b,dt);
        // --- DISCRETE CONTACT (STAYING CONTACT) ---
        ContactManifold m;
        if (discrete_wall_contact(b, wall, m)) {
            merge_manifold(contact_manifolds, m);
        }
    }
}

glm::vec2 PhysicsWorld::position() const
{
    return bodies.empty() ? glm::vec2{} : bodies[0].position;
}

glm::vec2 PhysicsWorld::velocity() const
{
    return bodies.empty() ? glm::vec2{} : bodies[0].velocity;
}

std::uint64_t PhysicsWorld::step_count() const noexcept { return m_steps; }

float PhysicsWorld::accumulator() const { return m_accumulator; }

const std::vector<ContactManifold> &PhysicsWorld::getManifolds() const
{
    return manifolds;
}

std::vector<Body> &PhysicsWorld::getBodies() { return bodies; }

/* This engine is not an event-driven system, it is a fixed timestep simulation.
 * Discrete wall contact is the function that checks the positions to decide
 * whether contact still exists or not. This solved the secondly opened issue
 * of engine playground repo.
 * https://github.com/oguzhanduguncu/engine_playground/issues/2
 */
bool PhysicsWorld::discrete_wall_contact(
    const Body& b,
    const Body& wall,
    ContactManifold& out
) {

    float distance = wall.position.x - b.position.x;

    if (distance > slop)
        return false;

    out.bodyA = b.id;
    out.bodyB = wall.id;
    out.pointCount = 1;

    // normal: against the body
    out.points[0].normal = {-1.0, 0.0};

    // contact point: on the wall
    out.points[0].position = {wall.position.x, b.position.y};

    // overshoot, violation of constraints
    const float penetration = b.position.x - wall.position.x;
    out.points[0].penetration = (penetration > 0.0f) ? penetration : 0.0f;

    return true;
}

void PhysicsWorld::merge_manifold(std::vector<ContactManifold> &dst,
                                  const ContactManifold &m)
{
    for (auto &e: dst) {
        if (e.bodyA == m.bodyA && e.bodyB == m.bodyB) {
            ContactManifold merged = m;

            // --- carry accumulated impulses ---
            if (e.pointCount > 0 && merged.pointCount > 0) {
                merged.points[0].Pn = e.points[0].Pn;
                merged.points[0].Pt = e.points[0].Pt;
            }

            e = merged;
            return;
        }
    }
    dst.push_back(m);
}

void PhysicsWorld::solve_contacts(float dt, float restitution)
{
    for (ContactManifold &m: manifolds) {
        if (m.pointCount == 0)
            continue;

        Body *A = nullptr;
        Body *B = nullptr;

        for (auto &b: bodies) {
            if (b.id == m.bodyA)
                A = &b;
            if (b.id == m.bodyB)
                B = &b;
        }
        if (!A || !B)
            continue;
        if (A->type != BodyType::Dynamic)
            continue;

        ContactPoint &cp = m.points[0];
        const glm::vec2 n = cp.normal;
        glm::vec2 t = {-n.y, n.x};

        // --- RELATIVE VELOCITY ---
        glm::vec2 vrel = A->velocity;
        if (B->type == BodyType::Kinematic)
            vrel -= B->velocity;

        float vn = glm::dot(vrel, n);
        if (vn >= 0.0)
            continue;

        float dPn = -vn / A->invMass;
        float Pn0 = cp.Pn;
        cp.Pn = std::max(Pn0 + dPn, 0.0f);
        dPn = cp.Pn - Pn0;

        A->velocity += n * static_cast<float>(dPn * A->invMass);

        float vt = glm::dot(vrel, t);
        float dPt = -vt / A->invMass;
        float Pt0 = cp.Pt;

        float mu = 0.5;
        float maxPt = mu * cp.Pn;
        cp.Pt = std::clamp(Pt0 + dPt, -maxPt, maxPt);
        dPt = cp.Pt - Pt0;

        A->velocity += t * static_cast<float>(dPt * A->invMass);
    }
}

void PhysicsWorld::solve_split_impulse(const float dt)
{
    for (const ContactManifold &m: manifolds) {
        Body *A = nullptr;
        Body *B = nullptr;

        for (auto &b: bodies) {
            if (b.id == m.bodyA)
                A = &b;
            if (b.id == m.bodyB)
                B = &b;
        }
        if (!A || !B)
            continue;
        if (A->invMass == 0.0)
            continue;

        const ContactPoint &cp = m.points[0];
        const glm::vec2 &n = cp.normal;

        const float p = cp.penetration;
        if (p <= 0.0)
            continue;

        const float lambda = p / (dt * A->invMass);

        A->pseudoVelocity.x += (lambda * A->invMass) * n.x;
        A->pseudoVelocity.y += (lambda * A->invMass) * n.y;
    }
}

// pseudo/split impulse for position correction
void PhysicsWorld::integrate_pseudo(float dt)
{
    for (Body &b: bodies) {
        if (b.invMass == 0.0)
            continue;

        b.position.x += b.pseudoVelocity.x * dt;
        b.position.y += b.pseudoVelocity.y * dt;

        b.pseudoVelocity = {0.0, 0.0};
    }
}
