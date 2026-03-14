#include <gtest/gtest.h>
#include "rvo_solver.h"
#include <glm/geometric.hpp>
#include <cmath>

static constexpr float DT = 1.0f / 60.0f;

static float speed(const RVOAgent& a) { return glm::length(a.body.velocity); }

// ── Tests ─────────────────────────────────────────────────────────────────────

// Single agent with no neighbours: preferred velocity is returned unchanged.
TEST(RVOSolver, SingleAgent_NoConstraints) {
    RVOSolver solver(DT);
    uint32_t id = solver.addAgent({0.0f, 0.0f}, {1.0f, 0.5f}, 0.3f, 5.0f);
    solver.setPreferredVelocity(id, {1.0f, 0.5f});

    solver.step();

    const RVOAgent& a = solver.getAgents()[id];
    EXPECT_FLOAT_EQ(a.body.velocity.x, 1.0f);
    EXPECT_FLOAT_EQ(a.body.velocity.y, 0.5f);
}

// Single agent: preferred velocity exceeding maxSpeed is clamped.
TEST(RVOSolver, SingleAgent_SpeedClamped) {
    RVOSolver solver(DT);
    uint32_t id = solver.addAgent({0.0f, 0.0f}, {0.0f, 0.0f}, 0.3f, 1.0f);
    solver.setPreferredVelocity(id, {5.0f, 0.0f});

    solver.step();

    const RVOAgent& a = solver.getAgents()[id];
    EXPECT_NEAR(speed(a), 1.0f, 1e-5f);
    EXPECT_GT(a.body.velocity.x, 0.0f);
    EXPECT_NEAR(a.body.velocity.y, 0.0f, 1e-5f);
}

// Single agent: position is integrated via semi_implicit_euler.
// With a=0 (no neighbours), it reduces to p += v*dt.
TEST(RVOSolver, SingleAgent_PositionIntegration) {
    RVOSolver solver(DT);
    uint32_t id = solver.addAgent({0.0f, 0.0f}, {2.0f, 1.0f}, 0.3f, 5.0f);
    solver.setPreferredVelocity(id, {2.0f, 1.0f});

    solver.step();

    const RVOAgent& a = solver.getAgents()[id];
    EXPECT_NEAR(a.body.position.x, 2.0f * DT, 1e-5f);
    EXPECT_NEAR(a.body.position.y, 1.0f * DT, 1e-5f);
}

// Two agents far apart (beyond neighbourDist): no ORCA lines built,
// preferred velocities returned unchanged.
TEST(RVOSolver, TwoAgents_FarApart_NoAvoidance) {
    RVOSolver solver(DT);
    uint32_t a0 = solver.addAgent({-10.0f, 0.0f}, {1.0f, 0.0f}, 0.5f, 3.0f, 5.0f);
    uint32_t a1 = solver.addAgent({ 10.0f, 0.0f}, {-1.0f, 0.0f}, 0.5f, 3.0f, 5.0f);
    solver.setPreferredVelocity(a0, {1.0f, 0.0f});
    solver.setPreferredVelocity(a1, {-1.0f, 0.0f});

    solver.step();

    const auto& agents = solver.getAgents();
    EXPECT_FLOAT_EQ(agents[a0].body.velocity.x,  1.0f);
    EXPECT_FLOAT_EQ(agents[a0].body.velocity.y,  0.0f);
    EXPECT_FLOAT_EQ(agents[a1].body.velocity.x, -1.0f);
    EXPECT_FLOAT_EQ(agents[a1].body.velocity.y,  0.0f);
}

// Two agents moving in the same direction: relative velocity is zero,
// no ORCA correction needed.
TEST(RVOSolver, TwoAgents_DivergingPaths_NoChange) {
    RVOSolver solver(DT);
    uint32_t a0 = solver.addAgent({-1.0f, 0.0f}, {1.0f, 0.0f}, 0.3f, 3.0f, 10.0f, 2.0f);
    uint32_t a1 = solver.addAgent({ 1.0f, 0.0f}, {1.0f, 0.0f}, 0.3f, 3.0f, 10.0f, 2.0f);
    solver.setPreferredVelocity(a0, {1.0f, 0.0f});
    solver.setPreferredVelocity(a1, {1.0f, 0.0f});

    solver.step();

    const auto& agents = solver.getAgents();
    EXPECT_NEAR(agents[a0].body.velocity.x, 1.0f, 1e-4f);
    EXPECT_NEAR(agents[a0].body.velocity.y, 0.0f, 1e-4f);
    EXPECT_NEAR(agents[a1].body.velocity.x, 1.0f, 1e-4f);
    EXPECT_NEAR(agents[a1].body.velocity.y, 0.0f, 1e-4f);
}

// Two agents on a direct head-on collision course: ORCA steers them to
// opposite sides of the collision axis.
TEST(RVOSolver, TwoAgents_HeadOnAvoidance) {
    RVOSolver solver(DT);
    uint32_t a0 = solver.addAgent({-2.0f, 0.0f}, {2.0f, 0.0f}, 0.5f, 4.0f, 10.0f, 2.0f);
    uint32_t a1 = solver.addAgent({ 2.0f, 0.0f}, {-2.0f, 0.0f}, 0.5f, 4.0f, 10.0f, 2.0f);
    solver.setPreferredVelocity(a0, {2.0f, 0.0f});
    solver.setPreferredVelocity(a1, {-2.0f, 0.0f});

    solver.step();

    const auto& agents = solver.getAgents();

    // Both agents still make forward progress.
    EXPECT_GT(agents[a0].body.velocity.x, 0.0f);
    EXPECT_LT(agents[a1].body.velocity.x, 0.0f);

    // Agents deviate in opposite y-directions.
    EXPECT_NE(agents[a0].body.velocity.y, 0.0f);
    EXPECT_NE(agents[a1].body.velocity.y, 0.0f);
    EXPECT_LT(agents[a0].body.velocity.y * agents[a1].body.velocity.y, 0.0f);

    // Neither agent exceeds its max speed.
    EXPECT_LE(speed(agents[a0]), agents[a0].maxSpeed + 1e-4f);
    EXPECT_LE(speed(agents[a1]), agents[a1].maxSpeed + 1e-4f);
}

// After avoidance, agents must not overlap over many steps.
TEST(RVOSolver, TwoAgents_HeadOn_NoCollision_ManySteps) {
    RVOSolver solver(DT);
    uint32_t a0 = solver.addAgent({-3.0f, 0.0f}, {1.5f, 0.0f}, 0.4f, 2.0f, 10.0f, 2.0f);
    uint32_t a1 = solver.addAgent({ 3.0f, 0.0f}, {-1.5f, 0.0f}, 0.4f, 2.0f, 10.0f, 2.0f);
    solver.setPreferredVelocity(a0, {1.5f, 0.0f});
    solver.setPreferredVelocity(a1, {-1.5f, 0.0f});

    float minDist = std::numeric_limits<float>::max();
    for (int s = 0; s < 200; ++s) {
        solver.step();
        const auto& ag   = solver.getAgents();
        glm::vec2   d    = ag[a1].body.position - ag[a0].body.position;
        float       dist = glm::length(d);
        if (dist < minDist) minDist = dist;
    }

    // Combined radius = 0.8; significant overlap must not occur.
    EXPECT_GT(minDist, 0.6f);
}

// Two agents on crossing paths must not collide.
TEST(RVOSolver, TwoAgents_CrossingPaths_Avoidance) {
    RVOSolver solver(DT);
    uint32_t a0 = solver.addAgent({0.0f, -2.0f}, {0.0f, 2.0f}, 0.4f, 3.0f, 10.0f, 2.0f);
    uint32_t a1 = solver.addAgent({-2.0f, 0.0f}, {2.0f, 0.0f}, 0.4f, 3.0f, 10.0f, 2.0f);
    solver.setPreferredVelocity(a0, {0.0f, 2.0f});
    solver.setPreferredVelocity(a1, {2.0f, 0.0f});

    float minDist = std::numeric_limits<float>::max();
    for (int s = 0; s < 200; ++s) {
        solver.step();
        const auto& ag   = solver.getAgents();
        glm::vec2   d    = ag[a1].body.position - ag[a0].body.position;
        float       dist = glm::length(d);
        if (dist < minDist) minDist = dist;
    }

    // Combined radius = 0.8; significant overlap must not occur.
    EXPECT_GT(minDist, 0.6f);
}
