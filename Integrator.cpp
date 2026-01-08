//
// Created by oguzh on 20.12.2025.
//

#include "Integrator.h"

void Integrator::semi_implicit_euler(
    Body& b,
    const float dt
) {
    b.velocity += b.acceleration * dt;
    b.position += b.velocity * dt;
}
