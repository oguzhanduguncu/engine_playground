//
// Created by oguzh on 20.12.2025.
//
// Simulation for fixed timestep
#include "physics_world.h"
#include "render_console.h"

#include <complex>
#include <iostream>

#include "Integrator.h"

PhysicsWorld::PhysicsWorld(double fixed_dt_seconds)
    : m_fixed_dt(fixed_dt_seconds) {}


void PhysicsWorld::update(const double frame_dt_seconds) {
    m_accumulator += frame_dt_seconds;

    while (m_accumulator >= m_fixed_dt) {
        step(m_fixed_dt);
        m_accumulator -= m_fixed_dt;
    }
}

double PhysicsWorld::position() const {
    return m_state.position;
}

double PhysicsWorld::velocity() const {
    return m_state.velocity;
}

void PhysicsWorld::step() {
    Integrator::semi_implicit_euler(m_state, m_fixed_dt);
    ++m_steps;
}

void PhysicsWorld::step(double dt) {

    if ((m_curr.acceleration == 0.0) && (m_curr.velocity == 0.0)) {
        // sleeping body -> no-op
        return;
    }
    m_prev = m_curr;
    if ((wall_x - m_curr.position) < m_curr.velocity*dt ) {
        step_with_ccd(dt);
    } else {
        hit.hit(false);
        Integrator::semi_implicit_euler(m_curr,dt);
    }
    ++m_steps;
}

const PhysicsState& PhysicsWorld::current() const { return m_curr; }
const PhysicsState& PhysicsWorld::previous() const { return m_prev; }

void PhysicsWorld::step_with_ccd(double dt) {
    // Snapshot for render + replay

    // Calculate the TOI
    hit = compute_toi(m_curr, dt);

    if (!hit.hit()) {
        Integrator::semi_implicit_euler(m_curr, dt);
        return;
    }

    // Iterate until the collision
    Integrator::semi_implicit_euler(m_curr, hit.time());

    // Collision response
    resolve_collision(m_curr, hit);

    //Iterate the remaining
    double remaining = dt - hit.time();
    Integrator::semi_implicit_euler(m_curr, remaining);

}


std::uint64_t PhysicsWorld::step_count() const noexcept {
    return m_steps;
}


void PhysicsWorld::resolve_collision(PhysicsState &m_curr, HitInfo hit ) {
    m_curr.velocity = -m_curr.velocity;
}

HitInfo PhysicsWorld::compute_toi(PhysicsState& m_curr, double dt) {
    const double x0 = m_curr.position;
    const double v0 = m_curr.velocity;
    const double a = m_curr.acceleration;
    double r1 = 0.0;
    double r2 = 0.0;
    double discriminant = v0*v0 - 4 * (x0-wall_x) * a/2;

    if (a == 0.0 && v0 > 0.0 && x0 < wall_x) {
        const double t = (wall_x - x0) / v0;
        if (t >= 0.0 && t <= dt) {
            hit.time(t);
            hit.hit(true);
        }
    } else if (a> 0.0 && v0 > 0.0 && x0 < wall_x) {
        double t_hit = std::numeric_limits<double>::max();
        r1 = (-v0 + sqrt(discriminant))/(2*a);
        r2 = (-v0 - sqrt(discriminant))/(2*a);
        if (r1 > 0.0 && r1 <= dt) t_hit = r1;
        if (r2 > 0.0 && r2 <= dt) t_hit = std::min(t_hit, r2);
        hit.time(t_hit);
        hit.hit(true);
    }
    return hit;
}

bool PhysicsWorld::check_collision() const {
    return hit.hit();
}

double PhysicsWorld::accumulator() const {
    return m_accumulator;
}

double PhysicsWorld::collision_time() const {
    return hit.time();
}

const HitInfo& PhysicsWorld::getHit() const {
    return hit;
}