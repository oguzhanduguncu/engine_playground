#include "scenario_manager.h"

void ScenarioManager::add(std::unique_ptr<Scenario> s)
{
    scenarios_.push_back(std::move(s));
}

void ScenarioManager::select(int index)
{
    if (index >= 0 && index < (int)scenarios_.size()) {
        active_ = index;
        scenarios_[active_]->init();
    }
}

void ScenarioManager::next()
{
    if (scenarios_.empty()) return;
    active_ = (active_ + 1) % (int)scenarios_.size();
    scenarios_[active_]->init();
}

void ScenarioManager::update(float dt)
{
    if (!scenarios_.empty())
        scenarios_[active_]->update(dt);
}

void ScenarioManager::render(render_2d& renderer)
{
    if (!scenarios_.empty())
        scenarios_[active_]->render(renderer);
}

const std::vector<std::unique_ptr<Scenario>>& ScenarioManager::list() const
{
    return scenarios_;
}
