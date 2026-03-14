#include "rvo_solver.h"
#include "Integrator.h"

#include <glm/geometric.hpp>
#include <algorithm>
#include <cmath>

static float det2(glm::vec2 a, glm::vec2 b) {
    return a.x * b.y - a.y * b.x;
}

static bool linearProgram1(const std::vector<OrcaLine>& lines, size_t lineNo,
                            float maxSpeed, glm::vec2 optVel, bool dirOpt,
                            glm::vec2& result)
{
    const glm::vec2& pt  = lines[lineNo].point;
    const glm::vec2& dir = lines[lineNo].direction;

    // Circle constraint: |pt + t*dir|^2 <= maxSpeed^2
    float b    = glm::dot(pt, dir);
    float c    = glm::dot(pt, pt) - maxSpeed * maxSpeed;
    float disc = b * b - c;
    if (disc < 0.0f) return false;
    float sqrtDisc = std::sqrt(disc);
    float tLeft  = -b - sqrtDisc;
    float tRight = -b + sqrtDisc;

    for (size_t j = 0; j < lineNo; ++j) {
        float D = det2(lines[j].direction, dir);
        float A = det2(lines[j].direction, pt - lines[j].point);
        if (std::abs(D) < 1e-6f) {
            if (A < 0.0f) return false;
            continue;
        }
        float t = -A / D;
        if (D > 0.0f) tLeft  = std::max(tLeft,  t);
        else          tRight = std::min(tRight, t);
        if (tLeft > tRight + 1e-6f) return false;
    }

    float tOpt = dirOpt ? tRight : glm::dot(optVel - pt, dir);
    tOpt   = std::clamp(tOpt, tLeft, tRight);
    result = pt + tOpt * dir;
    return true;
}

size_t RVOSolver::linearProgram2(const std::vector<OrcaLine>& lines,
                                 float maxSpeed, glm::vec2 optVel,
                                 bool  dirOpt,   glm::vec2& result)
{
    if (dirOpt) {
        float optLen = glm::length(optVel);
        result = (optLen > 1e-6f) ? (optVel / optLen) * maxSpeed
                                  : glm::vec2(maxSpeed, 0.0f);
    } else {
        float len = glm::length(optVel);
        result    = (len > maxSpeed) ? (optVel / len) * maxSpeed : optVel;
    }

    for (size_t i = 0; i < lines.size(); ++i) {
        if (det2(lines[i].direction, result - lines[i].point) < 0.0f) {
            if (!linearProgram1(lines, i, maxSpeed, optVel, dirOpt, result))
                return i;
        }
    }
    return lines.size();
}

void RVOSolver::linearProgram3(const std::vector<OrcaLine>& lines,
                               size_t beginLine, float maxSpeed,
                               glm::vec2& result)
{
    float distance = 0.0f;

    for (size_t i = beginLine; i < lines.size(); ++i) {
        float violation = det2(lines[i].direction, lines[i].point - result);
        if (violation <= distance) continue;

        std::vector<OrcaLine> projLines;
        projLines.reserve(i);

        for (size_t j = 0; j < i; ++j) {
            OrcaLine pl;
            float d = det2(lines[i].direction, lines[j].direction);
            if (std::abs(d) < 1e-6f) {
                if (glm::dot(lines[i].direction, lines[j].direction) > 0.0f)
                    continue;
                pl.point = 0.5f * (lines[i].point + lines[j].point);
            } else {
                float t = det2(lines[j].direction,
                               lines[i].point - lines[j].point) / d;
                pl.point = lines[i].point + t * lines[i].direction;
            }
            glm::vec2 projDir = lines[j].direction - lines[i].direction;
            float len = glm::length(projDir);
            if (len < 1e-6f) continue;
            pl.direction = projDir / len;
            projLines.push_back(pl);
        }

        glm::vec2 optDir = {-lines[i].direction.y, lines[i].direction.x};
        glm::vec2 prev   = result;
        if (linearProgram2(projLines, maxSpeed, optDir, true, result)
                < projLines.size())
            result = prev;

        distance = det2(lines[i].direction, lines[i].point - result);
    }
}

RVOSolver::RVOSolver(float fixed_dt) : m_dt(fixed_dt) {}

uint32_t RVOSolver::addAgent(glm::vec2 pos, glm::vec2 vel, float radius,
                             float maxSpeed, float neighborDist,
                             float timeHorizon)
{
    RVOAgent a{};
    // Initialise the embedded Body the same way the engine does for dynamics.
    a.body.id           = static_cast<BodyID>(agents.size());
    a.body.type         = BodyType::Dynamic;
    a.body.position     = pos;
    a.body.velocity     = vel;
    a.body.acceleration = {0.0f, 0.0f};
    a.body.invMass      = 1.0f;
    a.body.halfWidth    = radius;
    a.body.halfHeight   = radius;

    a.radius       = radius;
    a.maxSpeed     = maxSpeed;
    a.neighborDist = neighborDist;
    a.timeHorizon  = timeHorizon;
    a.prefVelocity = vel;

    agents.push_back(a);
    return a.body.id;
}

void RVOSolver::setPreferredVelocity(uint32_t agentId, glm::vec2 prefVel) {
    agents[agentId].prefVelocity = prefVel;
}

glm::vec2 RVOSolver::computeNewVelocity(size_t idx) const {
    const RVOAgent& A = agents[idx];
    std::vector<OrcaLine> lines;
    lines.reserve(agents.size() - 1);

    for (size_t j = 0; j < agents.size(); ++j) {
        if (j == idx) continue;
        const RVOAgent& B = agents[j];

        glm::vec2 relPos = B.body.position - A.body.position;
        float     distSq = glm::dot(relPos, relPos);

        if (distSq > A.neighborDist * A.neighborDist) continue;

        glm::vec2 relVel      = A.body.velocity - B.body.velocity;
        float     combinedR   = A.radius + B.radius;
        float     combinedRSq = combinedR * combinedR;

        OrcaLine  line;
        glm::vec2 u;

        if (distSq > combinedRSq) {
            // ── Non-collision: use time horizon τ ──────────────────────────
            float     tau   = A.timeHorizon;
            glm::vec2 w     = relVel - relPos / tau;
            float     wLSq  = glm::dot(w, w);
            float     dotWP = glm::dot(w, relPos);

            if (dotWP < 0.0f && dotWP * dotWP > combinedRSq * wLSq) {
                // Cut-off circle.
                float     wLen = std::sqrt(wLSq);
                glm::vec2 wHat = w / wLen;
                line.direction = { wHat.y, -wHat.x };
                u = (combinedR / tau - wLen) * wHat;
            } else {
                // Cone legs.
                float leg = std::sqrt(std::max(0.0f, distSq - combinedRSq));
                if (det2(relPos, w) > 0.0f) {
                    line.direction = glm::vec2(
                         relPos.x * leg - relPos.y * combinedR,
                         relPos.x * combinedR + relPos.y * leg) / distSq;
                } else {
                    line.direction = -glm::vec2(
                         relPos.x * leg + relPos.y * combinedR,
                        -relPos.x * combinedR + relPos.y * leg) / distSq;
                }
                u = glm::dot(relVel, line.direction) * line.direction - relVel;
            }
        } else {
            // ── Collision: use m_dt as emergency time horizon ──────────────
            glm::vec2 w    = relVel - relPos / m_dt;
            float     wLen = glm::length(w);
            if (wLen < 1e-6f) {
                glm::vec2 sep  = (glm::dot(relPos, relPos) > 1e-12f)
                                     ? glm::normalize(-relPos)
                                     : glm::vec2(1.0f, 0.0f);
                line.direction = { -sep.y, sep.x };
                u = glm::vec2{0.0f, 0.0f};
            } else {
                glm::vec2 wHat = w / wLen;
                line.direction = { wHat.y, -wHat.x };
                u = (combinedR / m_dt - wLen) * wHat;
            }
        }

        line.point = A.body.velocity + 0.5f * u;
        lines.push_back(line);
    }

    glm::vec2 newVel;
    size_t fail = linearProgram2(lines, A.maxSpeed, A.prefVelocity, false, newVel);
    if (fail < lines.size())
        linearProgram3(lines, fail, A.maxSpeed, newVel);
    return newVel;
}

// ---------------------------------------------------------------------------
// step: compute new velocities (read-only pass), then integrate each Body
// using the engine's semi-implicit Euler.
//
// The ORCA output is mapped to an acceleration that drives velocity to newVel
// in exactly one timestep:
//   a = (newVel - v) / m_dt
// Integrator then does:
//   v += a * m_dt  →  v = newVel
//   p += v * m_dt
// ---------------------------------------------------------------------------
void RVOSolver::step() {
    std::vector<glm::vec2> newVelocities(agents.size());
    for (size_t i = 0; i < agents.size(); ++i)
        newVelocities[i] = computeNewVelocity(i);

    for (size_t i = 0; i < agents.size(); ++i) {
        Body& b = agents[i].body;
        b.acceleration = (newVelocities[i] - b.velocity) / m_dt;
        Integrator::semi_implicit_euler(b, m_dt);
    }
}
