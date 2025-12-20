//
// Created by oguzh on 20.12.2025.
//

#ifndef PHYSICSWORLDVARIABLE_H
#define PHYSICSWORLDVARIABLE_H
#include "Integrator.h"
#include <cstdint>


class physics_world_variable {
public:
    explicit physics_world_variable(double fixed_dt);
    void update(double frame_dt);
    double velocity() const;
    double position() const;
    const PhysicsState& state() const;
    std::uint64_t step_count() const noexcept;

private:
    const double m_fixed_dt;
    PhysicsState m_state;
    std::uint64_t m_steps = 0;
};




#endif //PHYSICSWORLDVARIABLE_H
