#pragma once
#include <cstdint>
#include <vector>
#include "body.h"

// An ORCA half-plane constraint.
// Feasible region: dot(v - point, left_perp(direction)) >= 0
// where left_perp(d) = (-d.y, d.x)
struct OrcaLine {
    glm::vec2 point;      // a point on the constraint boundary
    glm::vec2 direction;  // unit tangent of the boundary line
};

// An RVO agent wraps a Body (position/velocity/acceleration) the same way
// Boid does, adding the ORCA-specific fields.
struct RVOAgent {
    Body      body;
    float     radius;        // circular collision radius for ORCA
    float     maxSpeed;
    float     neighborDist;  // how far to look for neighbours
    float     timeHorizon;   // look-ahead seconds for avoidance
    glm::vec2 prefVelocity;  // desired velocity set each frame by the caller
};

// ORCA-based reciprocal velocity obstacle solver.
// Mirrors PhysicsWorld: constructor takes the fixed timestep;
// call step() once per fixed tick.
class RVOSolver {
public:
    explicit RVOSolver(float fixed_dt);

    // Add an agent; returns its id (== index into getAgents()).
    uint32_t addAgent(glm::vec2 pos, glm::vec2 vel, float radius,
                      float maxSpeed,
                      float neighborDist = 5.0f,
                      float timeHorizon  = 2.0f);

    void setPreferredVelocity(uint32_t agentId, glm::vec2 prefVel);

    // Compute avoidance velocities and integrate all agents using
    // Integrator::semi_implicit_euler with the stored fixed timestep.
    void step();

    const std::vector<RVOAgent>& getAgents() const { return agents; }

private:
    std::vector<RVOAgent> agents;
    const float           m_dt;

    glm::vec2 computeNewVelocity(size_t idx) const;

    static size_t linearProgram2(const std::vector<OrcaLine>& lines,
                                 float maxSpeed, glm::vec2 optVel,
                                 bool  dirOpt,   glm::vec2& result);

    static void linearProgram3(const std::vector<OrcaLine>& lines,
                               size_t beginLine, float maxSpeed,
                               glm::vec2& result);
};
