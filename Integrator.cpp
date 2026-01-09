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

void Integrator::integrateY(Body& b, float dt) {
    if (b.onGround)
    {
        b.position.y = 0.0;
    }
    else
    {
        b.velocity.y += b.acceleration.y * dt;
        b.position.y  += b.velocity.y * dt;
    }
}
