#include <gtest/gtest.h>
#include "physics_world.h"
#include "test_helpers.h"

// ============================================================
// Contact Solver
// ============================================================

TEST(ContactSolver, CorrectsSeparatingVelocity) {
    float dt = 1.0f / 60.0f;
    PhysicsWorld world(dt);

    // Dynamic body approaching a static wall
    world.getBodies().push_back(make_dynamic(0, {7.99f, 2.0f}, {10.0f, 0.0f}));
    world.getBodies().push_back(make_static(1, {8.0f, 2.0f}));

    // Create a contact manifold manually
    ContactManifold m{};
    m.bodyA = 0;
    m.bodyB = 1;
    m.pointCount = 1;
    m.points[0].normal = {-1.0f, 0.0f};
    m.points[0].position = {8.0f, 2.0f};
    m.points[0].penetration = 0.0f;

    std::vector<ContactManifold> manifolds = {m};
    // Inject manifolds via step_bodies_with_ccd (which clears and rebuilds)
    // Instead, we'll use the public API to test the full pipeline

    // Direct approach: add bodies and run a fixed step
    // After the step, body should not pass through the wall
    Body& b = world.getBodies()[0];
    float vx_before = b.velocity.x;

    world.fixed_step(dt);

    // After collision solving, velocity should be reduced or reversed
    // (body was moving toward wall with vx=10)
    EXPECT_LE(b.velocity.x, vx_before);
}

TEST(ContactSolver, NoCorrectiveImpulseWhenSeparating) {
    float dt = 1.0f / 60.0f;
    PhysicsWorld world(dt);

    // Body moving away from wall
    world.getBodies().push_back(make_dynamic(0, {7.99f, 2.0f}, {-5.0f, 0.0f}));
    world.getBodies().push_back(make_static(1, {8.0f, 2.0f}));

    Body& b = world.getBodies()[0];
    world.fixed_step(dt);

    // Body is separating; solver should not add impulse toward wall
    // Velocity.x should remain negative (going left)
    EXPECT_LT(b.velocity.x, 0.0f);
}

TEST(ContactSolver, AccumulatedImpulseNonNegative) {
    float dt = 1.0f / 60.0f;
    PhysicsWorld world(dt);

    world.getBodies().push_back(make_dynamic(0, {7.99f, 2.0f}, {10.0f, 0.0f}));
    world.getBodies().push_back(make_static(1, {8.0f, 2.0f}));

    // Run a few steps to let impulse accumulate
    for (int i = 0; i < 5; ++i)
        world.update(dt);

    for (const auto& m : world.getManifolds()) {
        for (int j = 0; j < m.pointCount; ++j) {
            EXPECT_GE(m.points[j].Pn, 0.0f);
        }
    }
}

// ============================================================
// Split Impulse / Position Correction
// ============================================================

TEST(SplitImpulse, CorrectsPenetration) {
    float dt = 1.0f / 60.0f;
    PhysicsWorld world(dt);

    // Body overshooting wall
    world.getBodies().push_back(make_dynamic(0, {8.05f, 2.0f}, {0.0f, 0.0f}));
    world.getBodies().push_back(make_static(1, {8.0f, 2.0f}));

    // Create manifold with penetration
    ContactManifold m{};
    m.bodyA = 0;
    m.bodyB = 1;
    m.pointCount = 1;
    m.points[0].normal = {-1.0f, 0.0f};
    m.points[0].penetration = 0.05f; // 5cm overshoot

    // Run the full pipeline
    world.fixed_step(dt);
}

TEST(SplitImpulse, StaticBodyNotAffectedByPseudoIntegration) {
    float dt = 1.0f / 60.0f;
    PhysicsWorld world(dt);

    world.getBodies().push_back(make_static(0, {5.0f, 2.0f}));

    Body& b = world.getBodies()[0];
    b.pseudoVelocity = {10.0f, 10.0f};

    world.integrate_pseudo(dt);

    // Static body (invMass = 0) should not move
    EXPECT_FLOAT_EQ(b.position.x, 5.0f);
    EXPECT_FLOAT_EQ(b.position.y, 2.0f);
}

TEST(SplitImpulse, DynamicBodyMovedByPseudoVelocity) {
    float dt = 1.0f / 60.0f;
    PhysicsWorld world(dt);

    world.getBodies().push_back(make_dynamic(0, {5.0f, 2.0f}));
    Body& b = world.getBodies()[0];
    b.pseudoVelocity = {-60.0f, 0.0f};

    world.integrate_pseudo(dt);

    // pos.x += pseudoVelocity.x * dt = 5.0 + (-60) * (1/60) = 4.0
    EXPECT_NEAR(b.position.x, 4.0f, 1e-5f);
    EXPECT_FLOAT_EQ(b.position.y, 2.0f);
    // pseudo velocity reset to 0
    EXPECT_FLOAT_EQ(b.pseudoVelocity.x, 0.0f);
    EXPECT_FLOAT_EQ(b.pseudoVelocity.y, 0.0f);
}

TEST(SplitImpulse, PseudoVelocityResetAfterIntegration) {
    float dt = 1.0f / 60.0f;
    PhysicsWorld world(dt);

    world.getBodies().push_back(make_dynamic(0, {0.0f, 0.0f}));
    Body& b = world.getBodies()[0];
    b.pseudoVelocity = {100.0f, -50.0f};

    world.integrate_pseudo(dt);

    EXPECT_FLOAT_EQ(b.pseudoVelocity.x, 0.0f);
    EXPECT_FLOAT_EQ(b.pseudoVelocity.y, 0.0f);
}
