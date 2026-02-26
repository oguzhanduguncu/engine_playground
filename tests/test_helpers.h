#ifndef TEST_HELPERS_H
#define TEST_HELPERS_H

#include "body.h"
#include "physics_world.h"

inline Body make_dynamic(BodyID id, glm::vec2 pos, glm::vec2 vel = {0, 0},
                         glm::vec2 acc = {0, 0}, float invMass = 1.0f) {
    Body b{};
    b.id = id;
    b.type = BodyType::Dynamic;
    b.position = pos;
    b.velocity = vel;
    b.acceleration = acc;
    b.invMass = invMass;
    b.shape.type = Type::box;
    return b;
}

inline Body make_static(BodyID id, glm::vec2 pos, Type shapeType = Type::box) {
    Body b{};
    b.id = id;
    b.type = BodyType::Static;
    b.position = pos;
    b.velocity = {0, 0};
    b.acceleration = {0, 0};
    b.invMass = 0.0f;
    b.shape.type = shapeType;
    return b;
}

inline Body make_kinematic(BodyID id, glm::vec2 pos, glm::vec2 vel = {0, 0}) {
    Body b{};
    b.id = id;
    b.type = BodyType::Kinematic;
    b.position = pos;
    b.velocity = vel;
    b.acceleration = {0, 0};
    b.invMass = 1.0f;
    b.shape.type = Type::box;
    return b;
}

inline Body* find_body(PhysicsWorld& world, BodyID id) {
    for (auto& b : world.getBodies()) {
        if (b.id == id) return &b;
    }
    return nullptr;
}

#endif // TEST_HELPERS_H
