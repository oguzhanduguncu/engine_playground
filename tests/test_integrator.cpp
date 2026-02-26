#include <gtest/gtest.h>
#include "Integrator.h"
#include "body.h"
#include "test_helpers.h"

// --- Semi-Implicit Euler ---

TEST(Integrator, ZeroAccelerationZeroVelocity) {
    Body b = make_dynamic(0, {5.0f, 3.0f});
    Integrator::semi_implicit_euler(b, 1.0f / 60.0f);

    EXPECT_FLOAT_EQ(b.position.x, 5.0f);
    EXPECT_FLOAT_EQ(b.position.y, 3.0f);
    EXPECT_FLOAT_EQ(b.velocity.x, 0.0f);
    EXPECT_FLOAT_EQ(b.velocity.y, 0.0f);
}

TEST(Integrator, ConstantVelocityNoAcceleration) {
    Body b = make_dynamic(0, {0.0f, 0.0f}, {10.0f, -5.0f});
    float dt = 1.0f / 60.0f;
    Integrator::semi_implicit_euler(b, dt);

    EXPECT_FLOAT_EQ(b.velocity.x, 10.0f);
    EXPECT_FLOAT_EQ(b.velocity.y, -5.0f);
    EXPECT_NEAR(b.position.x, 10.0f * dt, 1e-5f);
    EXPECT_NEAR(b.position.y, -5.0f * dt, 1e-5f);
}

TEST(Integrator, AccelerationUpdatesVelocityFirst) {
    // Semi-implicit Euler: v += a*dt, then p += v*dt
    // So position uses the NEW velocity
    Body b = make_dynamic(0, {0.0f, 0.0f}, {0.0f, 0.0f}, {10.0f, 0.0f});
    float dt = 1.0f;
    Integrator::semi_implicit_euler(b, dt);

    EXPECT_FLOAT_EQ(b.velocity.x, 10.0f);
    // position = 0 + 10*1 = 10 (uses updated velocity)
    EXPECT_FLOAT_EQ(b.position.x, 10.0f);
}

TEST(Integrator, GravityFreefall) {
    Body b = make_dynamic(0, {0.0f, 100.0f}, {0.0f, 0.0f}, {0.0f, -9.8f});
    float dt = 1.0f / 60.0f;

    Integrator::semi_implicit_euler(b, dt);

    float expected_vy = -9.8f * dt;
    float expected_y = 100.0f + expected_vy * dt;

    EXPECT_NEAR(b.velocity.y, expected_vy, 1e-5f);
    EXPECT_NEAR(b.position.y, expected_y, 1e-5f);
}

TEST(Integrator, ZeroDtNoChange) {
    Body b = make_dynamic(0, {1.0f, 2.0f}, {3.0f, 4.0f}, {5.0f, 6.0f});
    Integrator::semi_implicit_euler(b, 0.0f);

    EXPECT_FLOAT_EQ(b.position.x, 1.0f);
    EXPECT_FLOAT_EQ(b.position.y, 2.0f);
    EXPECT_FLOAT_EQ(b.velocity.x, 3.0f);
    EXPECT_FLOAT_EQ(b.velocity.y, 4.0f);
}

TEST(Integrator, MultipleStepsAccumulate) {
    Body b = make_dynamic(0, {0.0f, 0.0f}, {1.0f, 0.0f});
    float dt = 0.1f;

    for (int i = 0; i < 10; ++i) {
        Integrator::semi_implicit_euler(b, dt);
    }

    // After 10 steps of 0.1s with v=1.0: position should be ~1.0
    EXPECT_NEAR(b.position.x, 1.0f, 1e-5f);
}

// --- IntegrateY ---

TEST(IntegrateY, OnGroundPinsToZero) {
    Body b = make_dynamic(0, {5.0f, 0.5f}, {0.0f, -10.0f}, {0.0f, -9.8f});
    b.onGround = true;

    Integrator::integrateY(b, 1.0f / 60.0f);

    EXPECT_FLOAT_EQ(b.position.y, 0.0f);
    // Velocity is not modified by integrateY when onGround
}

TEST(IntegrateY, NotOnGroundIntegratesNormally) {
    Body b = make_dynamic(0, {0.0f, 10.0f}, {0.0f, 0.0f}, {0.0f, -9.8f});
    b.onGround = false;
    float dt = 1.0f / 60.0f;

    Integrator::integrateY(b, dt);

    float expected_vy = -9.8f * dt;
    float expected_y = 10.0f + expected_vy * dt;

    EXPECT_NEAR(b.velocity.y, expected_vy, 1e-5f);
    EXPECT_NEAR(b.position.y, expected_y, 1e-5f);
}
