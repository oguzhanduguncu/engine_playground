#include "bench.h"
#include "boid_flock.h"
#include "boid.h"
#include "physics_world.h"
#include "body.h"
#include <random>
#include <vector>

// ── helpers ──────────────────────────────────────────────────────────────────

static std::mt19937 rng{42};

static Flock make_flock(int n)
{
    std::uniform_real_distribution<float> rx(-Flock::WORLD_HALF_W, Flock::WORLD_HALF_W);
    std::uniform_real_distribution<float> ry(-Flock::WORLD_HALF_H, Flock::WORLD_HALF_H);
    std::uniform_real_distribution<float> rv(-2.0f, 2.0f);

    Flock flock;
    for (int i = 0; i < n; ++i) {
        Boid b;
        b.body.id           = static_cast<uint32_t>(i);
        b.body.type         = BodyType::Dynamic;
        b.body.position     = {rx(rng), ry(rng)};
        b.body.velocity     = {rv(rng), rv(rng)};
        b.body.acceleration = {0.0f, 0.0f};
        b.body.invMass      = 1.0f;
        b.perception        = 2.5f;
        b.max_speed         = 9.0f;
        b.max_force         = 2.5f;
        b.w_separation      = 10.5f;
        b.w_alignment       = 10.0f;
        b.w_cohesion        = 1.0f;
        flock.add_boid(b);
    }
    return flock;
}

// Bodies spread across a wide grid so broadphase returns few/no pairs.
// This isolates integration + broadphase build cost from collision solving.
static PhysicsWorld make_physics_world(int n, float spread = 200.0f)
{
    PhysicsWorld world(1.0f / 60.0f);
    std::uniform_real_distribution<float> rx(-spread, spread);
    std::uniform_real_distribution<float> ry(-spread, spread);
    std::uniform_real_distribution<float> rv(-2.0f, 2.0f);

    for (int i = 0; i < n; ++i) {
        Body b;
        b.id           = static_cast<uint32_t>(i);
        b.type         = BodyType::Dynamic;
        b.position     = {rx(rng), ry(rng)};
        b.velocity     = {rv(rng), rv(rng)};
        b.acceleration = {0.0f, -9.8f};
        b.invMass      = 1.0f;
        world.getBodies().push_back(b);
    }
    return world;
}

// ── main ─────────────────────────────────────────────────────────────────────

int main()
{
    constexpr float dt = 1.0f / 60.0f;

    // Boid scenarios at different N — O(N²) brute-force neighbour search
    auto flock_500  = make_flock(500);
    auto flock_1000 = make_flock(1000);
    auto flock_2000 = make_flock(2000);
    auto flock_5000 = make_flock(5000);

    // Physics-world scenarios (integration + broadphase, sparse so no CCD log spam)
    auto world_100  = make_physics_world(100);
    auto world_500  = make_physics_world(500);
    auto world_1000 = make_physics_world(1000);

    bench_run({
        // ── boids ──────────────────────────────────────────────────────────
        { "boids/brute_force  N=500",  [&]{ flock_500 .step(dt); }, 5, 200 },
        { "boids/brute_force  N=1000", [&]{ flock_1000.step(dt); }, 5, 100 },
        { "boids/brute_force  N=2000", [&]{ flock_2000.step(dt); }, 5,  50 },
        { "boids/brute_force  N=5000", [&]{ flock_5000.step(dt); }, 5,  20 },

        // ── physics world (sparse — no CCD collisions) ─────────────────────
        { "physics/sparse  N=100",  [&]{ world_100 .fixed_step(dt); }, 5, 200 },
        { "physics/sparse  N=500",  [&]{ world_500 .fixed_step(dt); }, 5, 100 },
        { "physics/sparse  N=1000", [&]{ world_1000.fixed_step(dt); }, 5,  50 },
    });
}
