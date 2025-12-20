//
// Created by oguzh on 20.12.2025.
//

#ifndef PHYSICS_WORLD_H
#define PHYSICS_WORLD_H
#include <cstdint>
#include "physics_state.h"

class PhysicsWorld {
public:
    explicit PhysicsWorld(double fixed_dt_seconds);

    void update(double frame_dt_seconds);

    std::uint64_t step_count() const noexcept;

    double position() const;

    double velocity() const;

private:
    void step(); // one fixed physics step

private:
    PhysicsState m_state;
    const double m_fixed_dt;
    double m_accumulator = 0.0;
    std::uint64_t m_steps = 0;
};



#endif //PHYSICS_WORLD_H
