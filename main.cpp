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
    b.acceleration = {0.3,-9.8};
    b.invMass = 1.0;

    Body wall;
    wall.id = 1;
    wall.type = BodyType::Kinematic;
    wall.position = { 8.0, 2.0 };
    wall.velocity = { 0.0, 0.0 };
    wall.acceleration = { 0.0, 0.0 };
    wall.invMass = 0.0; // static

    Body b2;
    b2.id = 2;
    b2.type = BodyType::Dynamic;
    b2.position = {-3.0, 8.0};
    b2.velocity = {0.5, 0.0};
    b2.acceleration = {0.8,-9.8};
    b2.invMass = 1.0;

    Body ground;
    ground.id = 3;
    ground.type = BodyType::Static;
    ground.position = {0.0, 0.0};
    ground.velocity = {0.0, 0.0};
    ground.acceleration = {0.0,0.0};
    ground.invMass = 0.0;
    ground.shape.type = Type::plane;

    Body platform;
    platform.id = 4;
    platform.type = BodyType::Static;
    platform.position = {0.0, 2.0};
    platform.velocity = {0.0, 0.0};
    platform.acceleration = {0.0,0.0};
    platform.invMass = 0.0;
    platform.shape.type = Type::plane;

    world.getBodies().push_back(b);
    world.getBodies().push_back(wall);
    world.getBodies().push_back(ground);
    world.getBodies().push_back(platform);
    world.getBodies().push_back(b2);

    render_2d renderer{800,600};

    for (int frame = 0; frame < 300; ++frame) {
        auto now = engine::now();
        std::chrono::duration<float> frame_dt = now - last;
        last = now;
        int base_ms = 16; // ~60 FPS
        int jitter = jitter_ms(rng); // simulation of unsteady frame pacing

        std::vector<ContactManifold> manifolds;
        world.update(frame_dt.count());
        std::cout << "RENDER manifolds.size = " << world.getManifolds().size() << "\n";

        renderer.render(world);


        std::this_thread::sleep_for(std::chrono::milliseconds(base_ms + jitter));
    }
}
