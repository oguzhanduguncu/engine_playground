#pragma once
#include "../physics_world.h"

class render_2d;  // full definition only needed in .cpp files

class Scenario {
public:
    virtual ~Scenario() = default;
    virtual void init()                         = 0;
    virtual void update(float dt)               = 0;
    virtual void render(render_2d& renderer)    = 0;
    virtual const char* name() const            = 0;
};
