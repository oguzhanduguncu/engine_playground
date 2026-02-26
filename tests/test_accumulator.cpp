#include <gtest/gtest.h>
#include "physics_world.h"
#include "test_helpers.h"

// --- Accumulator / Time Management ---

TEST(Accumulator, InitialStateIsZero) {
    PhysicsWorld world(1.0f / 60.0f);
    EXPECT_FLOAT_EQ(world.accumulator(), 0.0f);
    EXPECT_EQ(world.step_count(), 0u);
}

TEST(Accumulator, SmallFrameDtNoStep) {
    // If frame_dt < fixed_dt, no physics step should occur
    PhysicsWorld world(1.0f / 60.0f);
    world.update(0.005f); // 5ms < 16.67ms

    EXPECT_EQ(world.step_count(), 0u);
    EXPECT_NEAR(world.accumulator(), 0.005f, 1e-6f);
}

TEST(Accumulator, ExactlyOneStep) {
    float fixed_dt = 1.0f / 60.0f;
    PhysicsWorld world(fixed_dt);
    world.update(fixed_dt);

    EXPECT_EQ(world.step_count(), 1u);
    EXPECT_NEAR(world.accumulator(), 0.0f, 1e-5f);
}

TEST(Accumulator, MultipleStepsPerFrame) {
    float fixed_dt = 1.0f / 60.0f;
    PhysicsWorld world(fixed_dt);

    // 3x fixed_dt should trigger 3 steps
    world.update(fixed_dt * 3.0f);

    EXPECT_EQ(world.step_count(), 3u);
    EXPECT_NEAR(world.accumulator(), 0.0f, 1e-4f);
}

TEST(Accumulator, AccumulatesRemainder) {
    float fixed_dt = 1.0f / 60.0f;
    PhysicsWorld world(fixed_dt);

    // 1.5x fixed_dt: 1 step, remainder = 0.5 * fixed_dt
    world.update(fixed_dt * 1.5f);

    EXPECT_EQ(world.step_count(), 1u);
    EXPECT_NEAR(world.accumulator(), fixed_dt * 0.5f, 1e-5f);
}

TEST(Accumulator, RemainderCarriesOver) {
    float fixed_dt = 1.0f / 60.0f;
    PhysicsWorld world(fixed_dt);

    // Two frames each 0.75 * fixed_dt: first has 0, second accumulates to 1.5x
    world.update(fixed_dt * 0.75f);
    EXPECT_EQ(world.step_count(), 0u);

    world.update(fixed_dt * 0.75f);
    EXPECT_EQ(world.step_count(), 1u);
    EXPECT_NEAR(world.accumulator(), fixed_dt * 0.5f, 1e-5f);
}

TEST(Accumulator, StepCountMonotonicallyIncreases) {
    float fixed_dt = 1.0f / 60.0f;
    PhysicsWorld world(fixed_dt);

    uint64_t prev = 0;
    for (int i = 0; i < 10; ++i) {
        world.update(fixed_dt);
        EXPECT_GE(world.step_count(), prev);
        prev = world.step_count();
    }
}

TEST(Accumulator, ZeroFrameDtNoStep) {
    PhysicsWorld world(1.0f / 60.0f);
    world.update(0.0f);

    EXPECT_EQ(world.step_count(), 0u);
    EXPECT_FLOAT_EQ(world.accumulator(), 0.0f);
}

TEST(Accumulator, LargeFrameDtMultipleSteps) {
    float fixed_dt = 1.0f / 60.0f;
    PhysicsWorld world(fixed_dt);

    // Simulate 1 second in a single frame (spike)
    world.update(1.0f);

    EXPECT_EQ(world.step_count(), 60u);
    EXPECT_GE(world.accumulator(), 0.0f);
    EXPECT_LT(world.accumulator(), fixed_dt);
}
