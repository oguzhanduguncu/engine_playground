//
// Created by oguzh on 20.12.2025.
//

#ifndef PHYSICS_WORLD_H
#define PHYSICS_WORLD_H
#include <cstdint>

#include "HitInfo.h"
#include "physics_state.h"

extern double wall_x;

class PhysicsWorld {
public:
    explicit PhysicsWorld(double fixed_dt_seconds);

    void update(double frame_dt_seconds);

    std::uint64_t step_count() const noexcept;

    HitInfo compute_toi_quad(PhysicsState&, double);

    double position() const;

    double velocity() const;

    const PhysicsState& current() const;

    const PhysicsState& previous() const;

    HitInfo compute_toi(PhysicsState&, double);

private:
    void step(); // one fixed physics step

    void step(double dt);

    void step_with_ccd(double dt);

    void resolve_collision(PhysicsState&, HitInfo);

    PhysicsState m_prev;
    PhysicsState m_curr;
    PhysicsState m_state;
    const double m_fixed_dt;
    double m_accumulator = 0.0;
    std::uint64_t m_steps = 0;
};



#endif //PHYSICS_WORLD_H
