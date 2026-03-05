//
// Created by oguzh on 4.03.2026.
//

#ifndef ENGINELOOP_BOID_H
#define ENGINELOOP_BOID_H
#include "body.h"

struct Boid {
    Body body;
    float perception;
    float max_speed;
    float max_force;
    float w_separation;
    float w_alignment;
    float w_cohesion;
};
#endif //ENGINELOOP_BOID_H
