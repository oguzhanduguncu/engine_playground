#pragma once
#include "scenario.h"
#include "../boid_flock.h"
#include "../physics_world.h"

class BoidScenario : public Scenario {
public:
    BoidScenario();
    void init()   override;
    void update(float dt) override;
    void render(render_2d& renderer) override;
    const char* name() const override { return "Boid Flock"; }

private:
    PhysicsWorld world_;
    Flock        flock_;
};
