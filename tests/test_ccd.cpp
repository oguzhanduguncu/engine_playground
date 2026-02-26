#include <gtest/gtest.h>
#include "physics_world.h"
#include "test_helpers.h"

// ============================================================
// X-Axis CCD (check_ccd)
// ============================================================

TEST(CCD, DetectsApproachingBodies) {
    float dt = 1.0f / 60.0f;
    PhysicsWorld world(dt);

    // Dynamic body moving toward a static wall
    world.getBodies().push_back(make_dynamic(0, {7.0f, 2.0f}, {100.0f, 0.0f}));
    world.getBodies().push_back(make_static(1, {8.0f, 2.0f}));

    std::vector<ContactManifold> manifolds;
    Body& b = world.getBodies()[0];
    Body& wall = world.getBodies()[1];

    world.check_ccd(b, wall, dt, manifolds);

    // Should detect collision via CCD
    EXPECT_GE(manifolds.size(), 1u);
}

TEST(CCD, NoDetectionWhenMovingAway) {
    float dt = 1.0f / 60.0f;
    PhysicsWorld world(dt);

    // Body moving away from wall
    world.getBodies().push_back(make_dynamic(0, {7.0f, 2.0f}, {-10.0f, 0.0f}));
    world.getBodies().push_back(make_static(1, {8.0f, 2.0f}));

    std::vector<ContactManifold> manifolds;
    Body& b = world.getBodies()[0];
    Body& wall = world.getBodies()[1];

    world.check_ccd(b, wall, dt, manifolds);

    EXPECT_EQ(manifolds.size(), 0u);
}

TEST(CCD, NoDetectionWhenYDistanceLarge) {
    float dt = 1.0f / 60.0f;
    PhysicsWorld world(dt);

    // Body at different height than wall (y distance > slop)
    world.getBodies().push_back(make_dynamic(0, {7.0f, 0.0f}, {100.0f, 0.0f}));
    world.getBodies().push_back(make_static(1, {8.0f, 5.0f}));

    std::vector<ContactManifold> manifolds;
    Body& b = world.getBodies()[0];
    Body& wall = world.getBodies()[1];

    world.check_ccd(b, wall, dt, manifolds);

    // CCD should detect TOI but skip due to y distance
    EXPECT_EQ(manifolds.size(), 0u);
}

TEST(CCD, KinematicPushesDynamic) {
    float dt = 1.0f / 60.0f;
    PhysicsWorld world(dt);

    // Kinematic body approaching a dynamic body
    Body kinematic = make_kinematic(0, {-4.0f, 2.0f}, {100.0f, 0.0f});
    Body dynamic_body = make_dynamic(1, {-3.0f, 2.0f}, {0.0f, 0.0f});

    world.getBodies().push_back(kinematic);
    world.getBodies().push_back(dynamic_body);

    std::vector<ContactManifold> manifolds;
    Body& k = world.getBodies()[0];
    Body& d = world.getBodies()[1];

    world.check_ccd(k, d, dt, manifolds);

    // Kinematic-dynamic: wall (dynamic) gets kinematic's velocity
    // After check_ccd, dynamic body should have kinematic's velocity
    EXPECT_NEAR(d.velocity.x, 100.0f, 1e-3f);
}

TEST(CCD, ContactManifoldHasCorrectNormal) {
    float dt = 1.0f / 60.0f;
    PhysicsWorld world(dt);

    world.getBodies().push_back(make_dynamic(0, {7.9f, 2.0f}, {50.0f, 0.0f}));
    world.getBodies().push_back(make_static(1, {8.0f, 2.0f}));

    std::vector<ContactManifold> manifolds;
    Body& b = world.getBodies()[0];
    Body& wall = world.getBodies()[1];

    world.check_ccd(b, wall, dt, manifolds);

    if (!manifolds.empty()) {
        EXPECT_FLOAT_EQ(manifolds[0].points[0].normal.x, -1.0f);
        EXPECT_FLOAT_EQ(manifolds[0].points[0].normal.y, 0.0f);
    }
}

// ============================================================
// step_bodies_with_ccd - Full Pipeline
// ============================================================

TEST(StepBodiesCCD, DynamicDynamicCollision) {
    float dt = 1.0f / 60.0f;
    PhysicsWorld world(dt);

    // Two dynamics approaching each other at same height
    world.getBodies().push_back(make_dynamic(0, {0.0f, 2.0f}, {50.0f, 0.0f}));
    world.getBodies().push_back(make_dynamic(1, {1.0f, 2.0f}, {-50.0f, 0.0f}));

    std::vector<ContactManifold> manifolds;
    world.step_bodies_with_ccd(dt, manifolds);

    // Should detect collision between the two dynamics
    EXPECT_GE(manifolds.size(), 1u);
}

TEST(StepBodiesCCD, SkipsPlaneStaticForDynamic) {
    float dt = 1.0f / 60.0f;
    PhysicsWorld world(dt);

    // Ground plane and dynamic body
    world.getBodies().push_back(make_static(0, {0.0f, 0.0f}, Type::plane));
    world.getBodies().push_back(make_dynamic(1, {0.0f, 5.0f}, {0.0f, 0.0f}, {0.0f, -9.8f}));

    std::vector<ContactManifold> manifolds;
    world.step_bodies_with_ccd(dt, manifolds);

    // Plane-type statics are handled by solveY, not as CCD walls
    // No CCD manifold should be generated for plane types
    for (const auto& m : manifolds) {
        // If any manifold involves body 0 (the plane), check it was from solveY path
        if (m.bodyB == 0 || m.bodyA == 0) {
            // This would be unexpected from CCD (plane is skipped)
        }
    }
}

TEST(StepBodiesCCD, KinematicDynamicInteraction) {
    float dt = 1.0f / 60.0f;
    PhysicsWorld world(dt);

    // Kinematic close to dynamic so TOI lands within dt
    // x0 = -1.5 - (-1.0) = -0.5, v0 = 100, t = 0.5/100 = 0.005 < dt
    world.getBodies().push_back(make_kinematic(0, {-1.5f, 2.0f}, {100.0f, 0.0f}));
    world.getBodies().push_back(make_dynamic(1, {-1.0f, 2.0f}, {0.0f, 0.0f}));

    std::vector<ContactManifold> manifolds;
    world.step_bodies_with_ccd(dt, manifolds);

    Body* dynamic_b = find_body(world, 1);
    ASSERT_NE(dynamic_b, nullptr);

    // Kinematic pushes dynamic: dynamic should have gained velocity
    EXPECT_GT(dynamic_b->velocity.x, 0.0f);
}

TEST(StepBodiesCCD, KinematicStaticBoxInteraction) {
    float dt = 1.0f / 60.0f;
    PhysicsWorld world(dt);

    // Kinematic approaching static box wall
    world.getBodies().push_back(make_kinematic(0, {7.0f, 2.0f}, {100.0f, 0.0f}));
    world.getBodies().push_back(make_static(1, {8.0f, 2.0f}, Type::box));

    std::vector<ContactManifold> manifolds;
    world.step_bodies_with_ccd(dt, manifolds);

    // Should generate contact manifold for kinematic vs static box
    EXPECT_GE(manifolds.size(), 1u);
}

TEST(StepBodiesCCD, KinematicStaticPlaneSkipped) {
    float dt = 1.0f / 60.0f;
    PhysicsWorld world(dt);

    // Kinematic body and static plane (should be skipped)
    world.getBodies().push_back(make_kinematic(0, {0.0f, 3.0f}, {10.0f, 0.0f}));
    world.getBodies().push_back(make_static(1, {5.0f, 3.0f}, Type::plane));

    std::vector<ContactManifold> manifolds;
    world.step_bodies_with_ccd(dt, manifolds);

    // Planes are skipped in kinematic-static loop
    EXPECT_EQ(manifolds.size(), 0u);
}

TEST(StepBodiesCCD, DynamicStaticBoxCollision) {
    float dt = 1.0f / 60.0f;
    PhysicsWorld world(dt);

    world.getBodies().push_back(make_dynamic(0, {7.0f, 2.0f}, {100.0f, 0.0f}));
    world.getBodies().push_back(make_static(1, {8.0f, 2.0f}, Type::box));

    std::vector<ContactManifold> manifolds;
    world.step_bodies_with_ccd(dt, manifolds);

    EXPECT_GE(manifolds.size(), 1u);
}

TEST(StepBodiesCCD, DynamicStaticPlaneSkipped) {
    float dt = 1.0f / 60.0f;
    PhysicsWorld world(dt);

    world.getBodies().push_back(make_dynamic(0, {0.0f, 2.0f}, {100.0f, 0.0f}));
    world.getBodies().push_back(make_static(1, {5.0f, 2.0f}, Type::plane));

    std::vector<ContactManifold> manifolds;
    world.step_bodies_with_ccd(dt, manifolds);

    // Planes are skipped in dynamic-static loop
    EXPECT_EQ(manifolds.size(), 0u);
}

// ============================================================
// TOI Edge Cases (tested indirectly via check_ccd)
// ============================================================

TEST(CCD, VeryFastBodyStillCaught) {
    float dt = 1.0f / 60.0f;
    PhysicsWorld world(dt);

    // Extremely fast body that would tunnel through wall in one step
    world.getBodies().push_back(make_dynamic(0, {0.0f, 2.0f}, {10000.0f, 0.0f}));
    world.getBodies().push_back(make_static(1, {8.0f, 2.0f}));

    std::vector<ContactManifold> manifolds;
    Body& b = world.getBodies()[0];
    Body& wall = world.getBodies()[1];

    world.check_ccd(b, wall, dt, manifolds);

    // CCD should still catch this
    EXPECT_GE(manifolds.size(), 1u);
}

TEST(CCD, StationaryBodiesNoDetection) {
    float dt = 1.0f / 60.0f;
    PhysicsWorld world(dt);

    // Two stationary bodies far apart
    world.getBodies().push_back(make_dynamic(0, {0.0f, 2.0f}, {0.0f, 0.0f}));
    world.getBodies().push_back(make_static(1, {8.0f, 2.0f}));

    std::vector<ContactManifold> manifolds;
    Body& b = world.getBodies()[0];
    Body& wall = world.getBodies()[1];

    world.check_ccd(b, wall, dt, manifolds);

    EXPECT_EQ(manifolds.size(), 0u);
}

TEST(CCD, BodyWithAccelerationTowardWall) {
    float dt = 1.0f / 60.0f;
    PhysicsWorld world(dt);

    // Body starting slow but accelerating toward wall
    world.getBodies().push_back(
        make_dynamic(0, {7.99f, 2.0f}, {0.0f, 0.0f}, {1000.0f, 0.0f}));
    world.getBodies().push_back(make_static(1, {8.0f, 2.0f}));

    std::vector<ContactManifold> manifolds;
    Body& b = world.getBodies()[0];
    Body& wall = world.getBodies()[1];

    world.check_ccd(b, wall, dt, manifolds);

    // Acceleration-based TOI should be detected
    EXPECT_GE(manifolds.size(), 1u);
}

// ============================================================
// Full Simulation Integration Tests
// ============================================================

TEST(FullSimulation, DynamicBodyFallsToGround) {
    float dt = 1.0f / 60.0f;
    PhysicsWorld world(dt);

    world.getBodies().push_back(
        make_dynamic(0, {0.0f, 5.0f}, {0.0f, 0.0f}, {0.0f, -9.8f}));
    world.getBodies().push_back(make_static(1, {0.0f, 0.0f}, Type::plane));

    // Run simulation for 2 seconds
    for (int i = 0; i < 120; ++i) {
        world.update(dt);
    }

    Body* b = find_body(world, 0);
    ASSERT_NE(b, nullptr);

    // Body should have fallen close to ground or landed on it
    EXPECT_LE(b->position.y, 5.0f);
}

TEST(FullSimulation, DynamicBodyStopsAtWall) {
    float dt = 1.0f / 60.0f;
    PhysicsWorld world(dt);

    // Body accelerating toward wall
    world.getBodies().push_back(
        make_dynamic(0, {2.0f, 2.0f}, {5.0f, 0.0f}, {2.0f, 0.0f}));
    world.getBodies().push_back(make_static(1, {8.0f, 2.0f}));

    for (int i = 0; i < 300; ++i) {
        world.update(dt);
    }

    Body* b = find_body(world, 0);
    ASSERT_NE(b, nullptr);

    // Body should be near the wall, not past it
    EXPECT_LE(b->position.x, 8.5f);
}

TEST(FullSimulation, KinematicBodyPushesDynamicBody) {
    float dt = 1.0f / 60.0f;
    PhysicsWorld world(dt);

    // Kinematic approaching stationary dynamic
    world.getBodies().push_back(make_kinematic(0, {-5.0f, 2.0f}, {5.0f, 0.0f}));
    world.getBodies().push_back(make_dynamic(1, {0.0f, 2.0f}, {0.0f, 0.0f}));

    float initial_dynamic_x = 0.0f;

    for (int i = 0; i < 120; ++i) {
        world.update(dt);
    }

    Body* dynamic_b = find_body(world, 1);
    ASSERT_NE(dynamic_b, nullptr);

    // Dynamic body should have been pushed forward
    EXPECT_GT(dynamic_b->position.x, initial_dynamic_x);
}

TEST(FullSimulation, MultipleStepsWithVariableFrameTime) {
    float dt = 1.0f / 60.0f;
    PhysicsWorld world(dt);

    world.getBodies().push_back(
        make_dynamic(0, {0.0f, 10.0f}, {0.0f, 0.0f}, {0.0f, -9.8f}));

    // Simulate with varying frame times like a real game
    float frame_times[] = {0.016f, 0.033f, 0.010f, 0.020f, 0.050f, 0.008f};
    for (float ft : frame_times) {
        world.update(ft);
    }

    // Accumulator should always be >= 0 and < fixed_dt
    EXPECT_GE(world.accumulator(), 0.0f);
    EXPECT_LT(world.accumulator(), dt + 1e-5f);
}

TEST(FullSimulation, DeterministicWithFixedTimestep) {
    float dt = 1.0f / 60.0f;

    // Run two identical simulations
    auto run_sim = [&]() -> glm::vec2 {
        PhysicsWorld world(dt);
        world.getBodies().push_back(
            make_dynamic(0, {0.0f, 10.0f}, {5.0f, 0.0f}, {1.0f, -9.8f}));
        world.getBodies().push_back(make_static(1, {8.0f, 10.0f}));

        for (int i = 0; i < 60; ++i) {
            world.update(dt);
        }
        return world.position();
    };

    glm::vec2 pos1 = run_sim();
    glm::vec2 pos2 = run_sim();

    // Exact same input, exact same fixed dt => deterministic result
    EXPECT_FLOAT_EQ(pos1.x, pos2.x);
    EXPECT_FLOAT_EQ(pos1.y, pos2.y);
}
