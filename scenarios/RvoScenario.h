#pragma once
#include "scenario.h"
#include "../rvo_solver.h"
#include "../physics_world.h"
#include <memory>

class RvoScenario : public Scenario {
public:
    RvoScenario();
    void init()   override;
    void update(float dt) override;
    void render(render_2d& renderer) override;
    const char* name() const override { return "RVO"; }

private:
    PhysicsWorld               world_;
    std::unique_ptr<RVOSolver> solver_;

    void sync_to_world();
};
