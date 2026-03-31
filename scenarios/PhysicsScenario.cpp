#include "PhysicsScenario.h"
#include "../render_2d.h"
#include "../body.h"
#include "../shape.h"

PhysicsScenario::PhysicsScenario() : world_(1.0f / 60.0f) {}

void PhysicsScenario::init()
{
    world_.getBodies().clear();

    Body b;
    b.id = 0; b.type = BodyType::Dynamic;
    b.position = {2.0f, 2.0f}; b.velocity = {0.0f, 0.0f};
    b.acceleration = {2.3f, -9.8f}; b.invMass = 1.0f;
    world_.getBodies().push_back(b);

    Body wall;
    wall.id = 1; wall.type = BodyType::Static;
    wall.position = {8.0f, 2.0f}; wall.velocity = {0.0f, 0.0f};
    wall.acceleration = {0.0f, 0.0f}; wall.invMass = 0.0f;
    world_.getBodies().push_back(wall);

    Body b2;
    b2.id = 2; b2.type = BodyType::Dynamic;
    b2.position = {-3.0f, 1.4f}; b2.velocity = {0.5f, 0.0f};
    b2.acceleration = {0.8f, -9.8f}; b2.invMass = 1.0f;
    world_.getBodies().push_back(b2);

    Body ground;
    ground.id = 3; ground.type = BodyType::Static;
    ground.position = {0.0f, 0.0f}; ground.velocity = {0.0f, 0.0f};
    ground.acceleration = {0.0f, 0.0f}; ground.invMass = 0.0f;
    ground.shape.type = Type::plane;
    world_.getBodies().push_back(ground);

    Body platform;
    platform.id = 4; platform.type = BodyType::Static;
    platform.position = {0.0f, 2.0f}; platform.velocity = {0.0f, 0.0f};
    platform.acceleration = {0.0f, 0.0f}; platform.invMass = 0.0f;
    platform.shape.type = Type::plane;
    world_.getBodies().push_back(platform);
}

void PhysicsScenario::update(float dt)
{
    world_.update(dt);
}

void PhysicsScenario::render(render_2d& renderer)
{
    renderer.render(world_);
}
