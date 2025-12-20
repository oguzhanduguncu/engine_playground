//
// Created by oguzh on 20.12.2025.
//

#include "physics_world.h"

#include "Integrator.h"

PhysicsWorld::PhysicsWorld(double fixed_dt_seconds)
    : m_fixed_dt(fixed_dt_seconds) {}

void PhysicsWorld::update(const double frame_dt_seconds) {
    m_accumulator += frame_dt_seconds;

    while (m_accumulator >= m_fixed_dt) {
        step();
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

std::uint64_t PhysicsWorld::step_count() const noexcept {
    return m_steps;
}

