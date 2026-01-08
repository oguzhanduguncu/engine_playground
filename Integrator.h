//
// Created by oguzh on 20.12.2025.
//

#ifndef INTEGRATOR_H
#define INTEGRATOR_H
#include "body.h"


struct Integrator {
    static void semi_implicit_euler(
        Body& b,
        float dt
    );
};




#endif //INTEGRATOR_H
