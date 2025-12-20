//
// Created by oguzh on 20.12.2025.
//

#ifndef INTEGRATOR_H
#define INTEGRATOR_H
#include "physics_state.h"


struct Integrator {
    static void semi_implicit_euler(
        PhysicsState& state,
        double dt
    );
};




#endif //INTEGRATOR_H
