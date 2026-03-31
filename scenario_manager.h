#pragma once
#include "scenarios/scenario.h"
#include <vector>
#include <memory>

class ScenarioManager {
public:
    void add(std::unique_ptr<Scenario> s);
    void select(int index);
    void next();
    void update(float dt);
    void render(render_2d& renderer);
    int  activeIndex() const { return active_; }
    const std::vector<std::unique_ptr<Scenario>>& list() const;

private:
    std::vector<std::unique_ptr<Scenario>> scenarios_;
    int active_ = 0;
};
