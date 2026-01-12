#include <iostream>
#include <thread>
#include "engine_time.h"
#include "physics_world.h"
#include "render_console.h"
#include <random>

#include "body.h"
#include "render_2d.h"

std::mt19937 rng{std::random_device{}()};
std::uniform_int_distribution jitter_ms(-5, 5);
float simulation_time;
float collision_global_time;
float collision_position;
bool prev_hit;

int main()
{
    constexpr float physics_dt = 1.0f / 60.0f; // 60 Hz physics
    PhysicsWorld world(physics_dt);

    auto last = engine::now();

    Body b;
    b.id = 0;
    b.type = BodyType::Dynamic;
    b.position = {-0.0, 8.0};
    b.velocity = {0.0, 0.0};
    b.acceleration = {2.8,-9.8};
    b.invMass = 1.0;

    Body wall;
    wall.id = 1;
    wall.type = BodyType::Kinematic;
    wall.position = { 8.0, 0.0 };
    wall.velocity = { 0.0, 0.0 };
    wall.acceleration = { 0.0, 0.0 };
    wall.invMass = 0.0; // static

    world.getBodies().push_back(b);
    world.getBodies().push_back(wall);

    render_2d renderer{800,600};

    for (int frame = 0; frame < 150; ++frame) {
        auto now = engine::now();
        std::chrono::duration<float> frame_dt = now - last;
        last = now;
        int base_ms = 16; // ~60 FPS
        int jitter = jitter_ms(rng); // simulation of unsteady frame pacing

        std::vector<ContactManifold> manifolds;
        world.update(frame_dt.count());
        std::cout << "RENDER manifolds.size = " << world.getManifolds().size() << "\n";

        renderer.render(world);

        render_console_2d(
            world.getBodies()[0].position,
            wall.position.x,10,10,
            world.getManifolds()
        );

        std::this_thread::sleep_for(std::chrono::milliseconds(base_ms + jitter));
    }
}
