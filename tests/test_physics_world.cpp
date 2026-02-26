#include <gtest/gtest.h>
#include "physics_world.h"
#include "test_helpers.h"

// ============================================================
// Ground Collision
// ============================================================

TEST(GroundCollision, BodyAtGroundCollides) {
    PhysicsWorld world(1.0f / 60.0f);
    Body b = make_dynamic(0, {0.0f, 0.0f});
    EXPECT_TRUE(world.collidesWithGround(b));
}

TEST(GroundCollision, BodyBelowGroundCollides) {
    PhysicsWorld world(1.0f / 60.0f);
    Body b = make_dynamic(0, {0.0f, -1.0f});
    EXPECT_TRUE(world.collidesWithGround(b));
}

TEST(GroundCollision, BodyAboveGroundDoesNotCollide) {
    PhysicsWorld world(1.0f / 60.0f);
    Body b = make_dynamic(0, {0.0f, 5.0f});
    EXPECT_FALSE(world.collidesWithGround(b));
}

TEST(GroundCollision, BodyJustAboveGroundDoesNotCollide) {
    PhysicsWorld world(1.0f / 60.0f);
    Body b = make_dynamic(0, {0.0f, 0.001f});
    EXPECT_FALSE(world.collidesWithGround(b));
}

TEST(GroundCollision, ResolveGroundPenetrationPushesUp) {
    PhysicsWorld world(1.0f / 60.0f);
    Body b = make_dynamic(0, {0.0f, -0.5f});
    b.halfHeight = 1.0f;

    world.resolveGroundPenetration(b);

    // bottom(b) = pos.y - halfHeight = -0.5 - 1.0 = -1.5
    // penetration = GROUND_Y - bottom = 0 - (-1.5) = 1.5
    // new pos.y = -0.5 + 1.5 = 1.0
    EXPECT_NEAR(b.position.y, 1.0f, 1e-5f);
}

TEST(GroundCollision, ResolveGroundPenetrationNoOpWhenAbove) {
    PhysicsWorld world(1.0f / 60.0f);
    Body b = make_dynamic(0, {0.0f, 5.0f});
    b.halfHeight = 1.0f;

    world.resolveGroundPenetration(b);

    // bottom = 5.0 - 1.0 = 4.0, penetration = 0 - 4 = -4 (not > 0)
    EXPECT_FLOAT_EQ(b.position.y, 5.0f);
}

// ============================================================
// SolveY - Ground + Platform Integration
// ============================================================

TEST(SolveY, SetsOnGroundWhenAtGround) {
    float dt = 1.0f / 60.0f;
    PhysicsWorld world(dt);
    world.getBodies().push_back(make_dynamic(0, {0.0f, 0.0f}, {0.0f, -5.0f}));

    Body& b = world.getBodies()[0];
    world.solveY(b, dt);

    EXPECT_TRUE(b.onGround);
    EXPECT_GE(b.velocity.y, 0.0f);
}

TEST(SolveY, ClearsOnGroundWhenAbove) {
    float dt = 1.0f / 60.0f;
    PhysicsWorld world(dt);
    world.getBodies().push_back(make_dynamic(0, {0.0f, 10.0f}, {0.0f, 5.0f}));

    Body& b = world.getBodies()[0];
    world.solveY(b, dt);

    EXPECT_FALSE(b.onGround);
}

TEST(SolveY, IntegratesWhenAboveGround) {
    float dt = 1.0f / 60.0f;
    PhysicsWorld world(dt);
    world.getBodies().push_back(make_dynamic(0, {0.0f, 10.0f}, {0.0f, 0.0f}, {0.0f, -9.8f}));

    Body& b = world.getBodies()[0];
    float y_before = b.position.y;
    world.solveY(b, dt);

    EXPECT_LT(b.position.y, y_before);
}

// ============================================================
// SolveY - Platform CCD
// ============================================================

TEST(SolveY, BodyFallingOntoPlatformIsCaught) {
    float dt = 1.0f / 60.0f;
    PhysicsWorld world(dt);

    // Platform above ground at y=5
    world.getBodies().push_back(make_static(1, {0.0f, 5.0f}, Type::plane));

    // Dynamic body just above platform, falling fast
    // TOI must land within dt: y0=0.1, vy=-20 => t≈0.005 < 1/60
    world.getBodies().push_back(make_dynamic(0, {0.0f, 5.1f}, {0.0f, -20.0f}, {0.0f, -9.8f}));

    Body& b = world.getBodies()[1];
    world.solveY(b, dt);

    EXPECT_TRUE(b.onGround);
    EXPECT_NEAR(b.position.y, 5.0f, PhysicsWorld::slop + 1e-5f);
    EXPECT_FLOAT_EQ(b.velocity.y, 0.0f);
}

TEST(SolveY, BodyRestingOnPlatformStays) {
    float dt = 1.0f / 60.0f;
    PhysicsWorld world(dt);

    world.getBodies().push_back(make_static(1, {0.0f, 5.0f}, Type::plane));
    world.getBodies().push_back(make_dynamic(0, {0.0f, 5.0f}, {0.0f, -0.001f}, {0.0f, -9.8f}));

    Body& b = world.getBodies()[1];
    world.solveY(b, dt);

    EXPECT_TRUE(b.onGround);
    EXPECT_FLOAT_EQ(b.position.y, 5.0f);
    EXPECT_FLOAT_EQ(b.velocity.y, 0.0f);
}

TEST(SolveY, BodyBelowPlatformIsIgnored) {
    float dt = 1.0f / 60.0f;
    PhysicsWorld world(dt);

    world.getBodies().push_back(make_static(1, {0.0f, 5.0f}, Type::plane));
    world.getBodies().push_back(make_dynamic(0, {0.0f, 3.0f}, {0.0f, 0.0f}, {0.0f, -9.8f}));

    Body& b = world.getBodies()[1];
    world.solveY(b, dt);

    // Body below platform, should not land on it
    EXPECT_FALSE(b.onGround);
}

TEST(SolveY, GroundLevelPlaneIsSkipped) {
    float dt = 1.0f / 60.0f;
    PhysicsWorld world(dt);

    // Plane at ground level (y=0) should be skipped for platform CCD
    world.getBodies().push_back(make_static(1, {0.0f, 0.0f}, Type::plane));
    world.getBodies().push_back(make_dynamic(0, {0.0f, 1.0f}, {0.0f, 0.0f}, {0.0f, -9.8f}));

    Body& b = world.getBodies()[1];
    world.solveY(b, dt);

    // Should not be caught by the ground-level plane as a "platform"
    // Body is still above ground, so solveY integrates normally
    EXPECT_FALSE(b.onGround);
}

TEST(SolveY, NonPlaneStaticBodiesAreSkipped) {
    float dt = 1.0f / 60.0f;
    PhysicsWorld world(dt);

    // Static box at y=5 (not a plane, should not act as platform)
    world.getBodies().push_back(make_static(1, {0.0f, 5.0f}, Type::box));
    world.getBodies().push_back(make_dynamic(0, {0.0f, 5.5f}, {0.0f, -20.0f}, {0.0f, -9.8f}));

    Body& b = world.getBodies()[1];
    world.solveY(b, dt);

    // Static box should not trigger platform CCD
    EXPECT_FALSE(b.onGround);
}

// ============================================================
// Discrete Wall Contact
// ============================================================

TEST(DiscreteContact, DetectsOverlappingBodies) {
    PhysicsWorld world(1.0f / 60.0f);
    // Body must be within slop (0.005) of wall for contact detection
    Body b = make_dynamic(0, {7.998f, 2.0f});
    Body wall = make_static(1, {8.0f, 2.0f});

    ContactManifold m{};
    bool contact = world.discrete_wall_contact(b, wall, m);

    EXPECT_TRUE(contact);
    EXPECT_EQ(m.bodyA, 0u);
    EXPECT_EQ(m.bodyB, 1u);
    EXPECT_EQ(m.pointCount, 1);
}

TEST(DiscreteContact, NoContactWhenFarApart) {
    PhysicsWorld world(1.0f / 60.0f);
    Body b = make_dynamic(0, {0.0f, 2.0f});
    Body wall = make_static(1, {8.0f, 2.0f});

    ContactManifold m{};
    bool contact = world.discrete_wall_contact(b, wall, m);

    EXPECT_FALSE(contact);
}

TEST(DiscreteContact, NoContactWhenYDistanceLarge) {
    PhysicsWorld world(1.0f / 60.0f);
    Body b = make_dynamic(0, {8.0f, 0.0f});
    Body wall = make_static(1, {8.0f, 5.0f});

    ContactManifold m{};
    bool contact = world.discrete_wall_contact(b, wall, m);

    EXPECT_FALSE(contact);
}

TEST(DiscreteContact, PenetrationCalculation) {
    PhysicsWorld world(1.0f / 60.0f);
    // Body is past the wall (overshoot)
    Body b = make_dynamic(0, {8.1f, 2.0f});
    Body wall = make_static(1, {8.0f, 2.0f});

    ContactManifold m{};
    bool contact = world.discrete_wall_contact(b, wall, m);

    EXPECT_TRUE(contact);
    // penetration = b.pos.x - wall.pos.x = 0.1
    EXPECT_NEAR(m.points[0].penetration, 0.1f, 1e-5f);
}

TEST(DiscreteContact, NoPenetrationWhenNotOvershot) {
    PhysicsWorld world(1.0f / 60.0f);
    Body b = make_dynamic(0, {7.99f, 2.0f});
    Body wall = make_static(1, {8.0f, 2.0f});

    ContactManifold m{};
    world.discrete_wall_contact(b, wall, m);

    // b.pos.x < wall.pos.x, penetration should be 0
    EXPECT_FLOAT_EQ(m.points[0].penetration, 0.0f);
}

TEST(DiscreteContact, NormalPointsAgainstBody) {
    PhysicsWorld world(1.0f / 60.0f);
    Body b = make_dynamic(0, {8.0f, 2.0f});
    Body wall = make_static(1, {8.0f, 2.0f});

    ContactManifold m{};
    world.discrete_wall_contact(b, wall, m);

    EXPECT_FLOAT_EQ(m.points[0].normal.x, -1.0f);
    EXPECT_FLOAT_EQ(m.points[0].normal.y, 0.0f);
}

// ============================================================
// Manifold Merging
// ============================================================

TEST(ManifoldMerge, AppendsNewManifold) {
    PhysicsWorld world(1.0f / 60.0f);
    std::vector<ContactManifold> dst;

    ContactManifold m{};
    m.bodyA = 0;
    m.bodyB = 1;
    m.pointCount = 1;
    m.points[0].Pn = 5.0f;

    world.merge_manifold(dst, m);

    EXPECT_EQ(dst.size(), 1u);
    EXPECT_EQ(dst[0].bodyA, 0u);
    EXPECT_EQ(dst[0].bodyB, 1u);
    EXPECT_FLOAT_EQ(dst[0].points[0].Pn, 5.0f);
}

TEST(ManifoldMerge, UpdatesExistingManifold) {
    PhysicsWorld world(1.0f / 60.0f);
    std::vector<ContactManifold> dst;

    ContactManifold m1{};
    m1.bodyA = 0;
    m1.bodyB = 1;
    m1.pointCount = 1;
    m1.points[0].Pn = 5.0f;
    m1.points[0].penetration = 0.1f;
    dst.push_back(m1);

    ContactManifold m2{};
    m2.bodyA = 0;
    m2.bodyB = 1;
    m2.pointCount = 1;
    m2.points[0].Pn = 0.0f; // new frame, reset
    m2.points[0].penetration = 0.05f;

    world.merge_manifold(dst, m2);

    EXPECT_EQ(dst.size(), 1u); // Still 1 entry
    // Accumulated impulse should be preserved from old manifold
    EXPECT_FLOAT_EQ(dst[0].points[0].Pn, 5.0f);
    // But penetration is from new manifold
    EXPECT_NEAR(dst[0].points[0].penetration, 0.05f, 1e-5f);
}

TEST(ManifoldMerge, DifferentPairCreatesNewEntry) {
    PhysicsWorld world(1.0f / 60.0f);
    std::vector<ContactManifold> dst;

    ContactManifold m1{};
    m1.bodyA = 0;
    m1.bodyB = 1;
    m1.pointCount = 1;
    dst.push_back(m1);

    ContactManifold m2{};
    m2.bodyA = 2;
    m2.bodyB = 3;
    m2.pointCount = 1;

    world.merge_manifold(dst, m2);

    EXPECT_EQ(dst.size(), 2u);
}

TEST(ManifoldMerge, PreservesTangentImpulse) {
    PhysicsWorld world(1.0f / 60.0f);
    std::vector<ContactManifold> dst;

    ContactManifold m1{};
    m1.bodyA = 0;
    m1.bodyB = 1;
    m1.pointCount = 1;
    m1.points[0].Pn = 3.0f;
    m1.points[0].Pt = 1.5f;
    dst.push_back(m1);

    ContactManifold m2{};
    m2.bodyA = 0;
    m2.bodyB = 1;
    m2.pointCount = 1;

    world.merge_manifold(dst, m2);

    EXPECT_FLOAT_EQ(dst[0].points[0].Pn, 3.0f);
    EXPECT_FLOAT_EQ(dst[0].points[0].Pt, 1.5f);
}

// ============================================================
// Position / Velocity Accessors
// ============================================================

TEST(Accessors, EmptyWorldReturnsZeroVec) {
    PhysicsWorld world(1.0f / 60.0f);
    auto pos = world.position();
    auto vel = world.velocity();

    EXPECT_FLOAT_EQ(pos.x, 0.0f);
    EXPECT_FLOAT_EQ(pos.y, 0.0f);
    EXPECT_FLOAT_EQ(vel.x, 0.0f);
    EXPECT_FLOAT_EQ(vel.y, 0.0f);
}

TEST(Accessors, ReturnsFirstBodyState) {
    PhysicsWorld world(1.0f / 60.0f);
    world.getBodies().push_back(make_dynamic(0, {3.0f, 4.0f}, {1.0f, 2.0f}));

    EXPECT_FLOAT_EQ(world.position().x, 3.0f);
    EXPECT_FLOAT_EQ(world.position().y, 4.0f);
    EXPECT_FLOAT_EQ(world.velocity().x, 1.0f);
    EXPECT_FLOAT_EQ(world.velocity().y, 2.0f);
}
