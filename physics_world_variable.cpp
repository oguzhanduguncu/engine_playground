//
// Created by oguzh on 20.12.2025.
//
//Simulation for Varying timestep
#include "PhysicsWorldVariable.h"


physics_world_variable::physics_world_variable(double fixed_dt)
    : m_fixed_dt(fixed_dt){}


void physics_world_variable::update(double frame_dt) {
    Integrator::semi_implicit_euler(m_state, frame_dt);
    ++m_steps;
}

double physics_world_variable::velocity() const {
    return m_state.velocity2d.x;
}

double physics_world_variable::position() const {
    return m_state.position2d.x;
}


const PhysicsState &physics_world_variable::state() const{ return m_state; }

std::uint64_t physics_world_variable::step_count() const noexcept {
    return m_steps;
}