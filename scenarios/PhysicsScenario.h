#pragma once
#include "scenario.h"
#include "../physics_world.h"

class PhysicsScenario : public Scenario {
public:
    PhysicsScenario();
    void init()   override;
    void update(float dt) override;
    void render(render_2d& renderer) override;
    const char* name() const override { return "Physics"; }

private:
    PhysicsWorld world_;
};
