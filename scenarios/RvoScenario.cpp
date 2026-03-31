#include "RvoScenario.h"
#include "../render_2d.h"
#include "../body.h"

RvoScenario::RvoScenario()
    : world_(1.0f / 60.0f)
    , solver_(std::make_unique<RVOSolver>(1.0f / 60.0f))
{}

void RvoScenario::init()
{
    solver_ = std::make_unique<RVOSolver>(1.0f / 60.0f);
    world_.getBodies().clear();

    for (int i = 0; i < 10; ++i) {
        glm::vec2 pos = {-200.0f, -90.0f + i * 20.0f};
        uint32_t id = solver_->addAgent(pos, {0, 0}, 8.0f, 100.0f, 150.0f, 2.0f);
        solver_->setPreferredVelocity(id, {100.0f, 0.0f});
    }
    for (int i = 0; i < 10; ++i) {
        glm::vec2 pos = {200.0f, -90.0f + i * 20.0f};
        uint32_t id = solver_->addAgent(pos, {0, 0}, 8.0f, 100.0f, 150.0f, 2.0f);
        solver_->setPreferredVelocity(id, {-100.0f, 0.0f});
    }

    sync_to_world();
}

void RvoScenario::update(float /*dt*/)
{
    solver_->step();
    sync_to_world();
}

void RvoScenario::sync_to_world()
{
    auto& bodies        = world_.getBodies();
    const auto& agents  = solver_->getAgents();
    bodies.resize(agents.size());
    for (size_t i = 0; i < agents.size(); ++i)
        bodies[i] = agents[i].body;
}

void RvoScenario::render(render_2d& renderer)
{
    renderer.render(world_);
}
