//
// Created by oguzh on 20.12.2025.
//

#ifndef PHYSICS_WORLD_H
#define PHYSICS_WORLD_H
#include <cstdint>
#include <vector>

#include "body.h"
#include "contact_manifold.h"

class PhysicsWorld {
public:
    static constexpr float slop = 0.001f;

    static constexpr float eps = 1e-6f;

    explicit PhysicsWorld(float fixed_dt_seconds);

    void update_kinematics(float dt);

    void update(float frame_dt_seconds);

    void fixed_step(float dt);

    [[nodiscard]] std::uint64_t step_count() const noexcept;

    void step_bodies_with_ccd(float dt, std::vector<ContactManifold> &contact_manifolds);

    [[nodiscard]] glm::vec2 position() const;

    [[nodiscard]] glm::vec2 velocity() const;

    [[nodiscard]] float accumulator() const;

    void solve_contacts(float dt, float restitution);

    void solve_split_impulse(float dt);

    void integrate_pseudo(float dt);

    [[nodiscard]] const std::vector<ContactManifold>& getManifolds() const;

    std::vector<Body>& getBodies();

    bool discrete_wall_contact(
    const Body& b,
    const Body& wall,
    ContactManifold& out
);
    void merge_manifold(
    std::vector<ContactManifold>& dst,
    const ContactManifold& m
);

private:
    std::vector<ContactManifold> manifolds;
    std::vector<Body> bodies;
    const float m_fixed_dt;
    float m_accumulator = 0.0;
    std::uint64_t m_steps = 0;
};



#endif //PHYSICS_WORLD_H
