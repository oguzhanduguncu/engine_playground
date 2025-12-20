//
// Created by oguzh on 20.12.2025.
//

#include "Integrator.h"

void Integrator::semi_implicit_euler(
    PhysicsState& state,
    const double dt
) {
    state.velocity += state.acceleration * dt;
    state.position += state.velocity * dt;
}
