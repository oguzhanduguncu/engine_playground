#include <iostream>
#include <thread>
#include <random>
#include "engine_time.h"
#include "physics_world.h"
#include "render_console.h"
#include "body.h"
#include "render_2d.h"
#include "boid_flock.h"
#include "boid.h"

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
    b.position = {2.0, 2.0};
    b.velocity = {0.0, 0.0};
    b.acceleration = {2.3,-9.8};
    b.invMass = 1.0;

    Body wall;
    wall.id = 1;
    wall.type = BodyType::Static;
    wall.position = { 8.0, 2.0 };
    wall.velocity = { 0.0, 0.0 };
    wall.acceleration = { 0.0, 0.0 };
    wall.invMass = 0.0; // static

    Body b2;
    b2.id = 2;
    b2.type = BodyType::Dynamic;
    b2.position = {-3.0, 1.4};
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

    Body player;
    player.id = 5;
    player.type = BodyType::Kinematic;
    player.position = {-6.0, 2.0};
    player.velocity = {3.0, 0.0};
    player.acceleration = {0.0,0.0};
    player.invMass = 1.0;

    Body ball;
    ball.id = 6;
    ball.type = BodyType::Dynamic;
    ball.position = {-4.0, 2.0};
    ball.velocity = {0.0, 0.0};
    ball.acceleration = {0.0,0.0};
    ball.invMass = 1.0;

 //   world.getBodies().push_back(b);
 //   world.getBodies().push_back(wall);
 //   world.getBodies().push_back(ground);
 //   world.getBodies().push_back(platform);
 //   world.getBodies().push_back(b2);
 //   world.getBodies().push_back(player);
 //   world.getBodies().push_back(ball);

    std::uniform_real_distribution<float> rx(-Flock::WORLD_HALF_W, Flock::WORLD_HALF_W);
    std::uniform_real_distribution<float> ry(-Flock::WORLD_HALF_H, Flock::WORLD_HALF_H);
    std::uniform_real_distribution<float> rv(-2.0f, 2.0f);

    Flock flock;
    for (int i = 0; i < 200; ++i) {
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
        flock.add_boid(std::move(b));
    }

    for (int i = 0; i < 200; ++i) {
        Body b;
        b.id           = static_cast<uint32_t>(i);
        b.type         = BodyType::Dynamic;
        b.position     = {rx(rng), ry(rng)};
        b.velocity     = {rv(rng), rv(rng)};
        b.acceleration = {0.0f, 0.0f};
        b.invMass      = 1.0f;
//        world.getBodies().push_back(std::move(b));
    }

    world.attach_flock(&flock);

    render_2d renderer{800, 600};
//    renderer.loadTexture(ASSET_DIR "cat.png");

//    for (int frame = 0; frame < 300; ++frame) {
//        auto now = engine::now();
//        std::chrono::duration<float> frame_dt = now - last;
//        last = now;
//        int base_ms = 16;
//        int jitter = jitter_ms(rng);
//        std::vector<ContactManifold> manifolds;
//        world.update(frame_dt.count());
//        std::cout << "RENDER manifolds.size = " << world.getManifolds().size() << "\n";
//        renderer.render(world);
//        std::this_thread::sleep_for(std::chrono::milliseconds(base_ms + jitter));
//    }

    while (renderer.isRunning()) {
        renderer.handleEvents();

        auto now = engine::now();
        std::chrono::duration<float> dt = now - last;
        last = now;

        world.update(dt.count());
        renderer.render(world,flock);
    }
}
