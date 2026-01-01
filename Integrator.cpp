//
// Created by oguzh on 20.12.2025.
//

#include "Integrator.h"

#include <iostream>

void Integrator::semi_implicit_euler(
    PhysicsState& state,
    const double dt
) {
    state.velocity2d.x += state.acceleration2d.x * dt;
    state.velocity2d.y += state.acceleration2d.y * dt;
    state.position2d.x += state.velocity2d.x * dt;
    state.position2d.y += state.velocity2d.y * dt;
}
