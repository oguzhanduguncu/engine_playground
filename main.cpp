#include "engine_time.h"
#include "render_2d.h"
#include "scenario_manager.h"
#include "scenarios/PhysicsScenario.h"
#include "scenarios/BoidScenario.h"
#include "scenarios/RvoScenario.h"

int main()
{
    render_2d renderer{800, 600};

    ScenarioManager manager;
    manager.add(std::make_unique<PhysicsScenario>());
    manager.add(std::make_unique<BoidScenario>());
    manager.add(std::make_unique<RvoScenario>());
    manager.select(0);  // init first scenario

    auto last = engine::now();

    while (renderer.isRunning()) {
        auto now = engine::now();
        std::chrono::duration<float> dt = now - last;
        last = now;

        // Keys captured during the previous frame's render()
        if (renderer.wasKeyPressed(SDLK_TAB))  manager.next();
        if (renderer.wasKeyPressed(SDLK_1))    manager.select(0);
        if (renderer.wasKeyPressed(SDLK_2))    manager.select(1);
        if (renderer.wasKeyPressed(SDLK_3))    manager.select(2);

        manager.update(dt.count());
        manager.render(renderer);
    }
}
