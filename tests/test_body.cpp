#include <gtest/gtest.h>
#include "body.h"
#include "contact.h"
#include "contact_manifold.h"

// --- Body Default Initialization ---

TEST(Body, DefaultValues) {
    Body b{};
    EXPECT_FLOAT_EQ(b.position.x, 0.0f);
    EXPECT_FLOAT_EQ(b.position.y, 0.0f);
    EXPECT_FLOAT_EQ(b.velocity.x, 0.0f);
    EXPECT_FLOAT_EQ(b.velocity.y, 0.0f);
    EXPECT_FLOAT_EQ(b.acceleration.x, 0.0f);
    EXPECT_FLOAT_EQ(b.acceleration.y, 0.0f);
    EXPECT_FLOAT_EQ(b.pseudoVelocity.x, 0.0f);
    EXPECT_FLOAT_EQ(b.pseudoVelocity.y, 0.0f);
    EXPECT_FLOAT_EQ(b.invMass, 0.0f);
    EXPECT_FLOAT_EQ(b.halfWidth, 0.0f);
    EXPECT_FLOAT_EQ(b.halfHeight, 0.0f);
    EXPECT_FALSE(b.onGround);
}

TEST(Body, StaticBodyHasZeroInvMass) {
    Body b{};
    b.type = BodyType::Static;
    b.invMass = 0.0f;
    EXPECT_FLOAT_EQ(b.invMass, 0.0f);
}

TEST(Body, BodyTypes) {
    Body dynamic{};
    dynamic.type = BodyType::Dynamic;
    EXPECT_EQ(dynamic.type, BodyType::Dynamic);

    Body statik{};
    statik.type = BodyType::Static;
    EXPECT_EQ(statik.type, BodyType::Static);

    Body kinematic{};
    kinematic.type = BodyType::Kinematic;
    EXPECT_EQ(kinematic.type, BodyType::Kinematic);
}

// --- ContactPoint ---

TEST(ContactPoint, DefaultAccumulatedImpulses) {
    ContactPoint cp{};
    EXPECT_FLOAT_EQ(cp.Pn, 0.0f);
    EXPECT_FLOAT_EQ(cp.Pt, 0.0f);
}

// --- ContactManifold ---

TEST(ContactManifold, MaxPointsIsTwo) {
    EXPECT_EQ(ContactManifold::MAX_POINTS, 2u);
}

TEST(ContactManifold, DefaultValues) {
    ContactManifold m{};
    EXPECT_EQ(m.bodyA, UINT32_MAX);
    EXPECT_EQ(m.bodyB, UINT32_MAX);
    EXPECT_EQ(m.pointCount, 0);
}

// --- Shape ---

TEST(Shape, BoxType) {
    Shape s{Type::box};
    EXPECT_EQ(s.type, Type::box);
}

TEST(Shape, PlaneType) {
    Shape s{Type::plane};
    EXPECT_EQ(s.type, Type::plane);
}
