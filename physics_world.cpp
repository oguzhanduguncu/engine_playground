//
// Created by oguzh on 20.12.2025.
//

#include "physics_world.h"

PhysicsWorld::PhysicsWorld(double fixed_dt_seconds)
    : m_fixed_dt(fixed_dt_seconds) {}

void PhysicsWorld::update(double frame_dt_seconds) {
    m_accumulator += frame_dt_seconds;

    while (m_accumulator >= m_fixed_dt) {
        step();
        m_accumulator -= m_fixed_dt;
    }
}

void PhysicsWorld::step() {
    ++m_steps;
}

std::uint64_t PhysicsWorld::step_count() const noexcept {
    return m_steps;
}

