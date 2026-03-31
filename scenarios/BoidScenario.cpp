#include "BoidScenario.h"
#include "../render_2d.h"
#include "../boid.h"
#include <random>

BoidScenario::BoidScenario() : world_(1.0f / 60.0f) {}

void BoidScenario::init()
{
    flock_ = Flock{};
    world_.getBodies().clear();

    std::mt19937 rng{42};
    std::uniform_real_distribution<float> rx(-Flock::WORLD_HALF_W, Flock::WORLD_HALF_W);
    std::uniform_real_distribution<float> ry(-Flock::WORLD_HALF_H, Flock::WORLD_HALF_H);
    std::uniform_real_distribution<float> rv(-2.0f, 2.0f);

    for (int i = 0; i < 200; ++i) {
        Boid b;
        b.body.id           = static_cast<uint32_t>(i);
        b.body.type         = BodyType::Dynamic;
        b.body.position     = {rx(rng), ry(rng)};
        b.body.velocity     = {rv(rng), rv(rng)};
        b.body.acceleration = {0.0f, 0.0f};
        b.body.invMass      = 1.0f;
        b.perception        = 2.5f;
        b.max_speed         = 9.0f;
        b.max_force         = 2.5f;
        b.w_separation      = 10.5f;
        b.w_alignment       = 10.0f;
        b.w_cohesion        = 1.0f;
        flock_.add_boid(b);
    }

    world_.attach_flock(&flock_);
}

void BoidScenario::update(float dt)
{
    world_.update(dt);
}

void BoidScenario::render(render_2d& renderer)
{
    renderer.render(world_, flock_);
}
