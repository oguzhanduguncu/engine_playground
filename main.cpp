#include <iostream>
#include <thread>
#include "engine_time.h"
#include "physics_world.h"
#include "render_console.h"
#include <random>

#include "physics_world_variable.h"

std::mt19937 rng{std::random_device{}()};
std::uniform_int_distribution<int> jitter_ms(-5, 5);
double simulation_time;
double collision_global_time;
double collision_position;
bool prev_hit;
double wall_x = 10;

int main() {
    constexpr double physics_dt = 1.0 / 60.0; // 60 Hz physics
    PhysicsWorld world(physics_dt);
    physics_world_variable world_variable(physics_dt);

    auto last = engine::now();

    for (int frame = 0; frame < 150; ++frame) {
        auto now = engine::now();
        std::chrono::duration<double> frame_dt = now - last;
        last = now;
        int base_ms = 16; // ~60 FPS
        int jitter = jitter_ms(rng); // simulation of unsteady frame pacing

        world.update(frame_dt.count());
        world_variable.update(frame_dt.count());
        simulation_time += frame_dt.count();

        const double alpha = world.accumulator()/frame_dt.count();
        const double render_x =
            interpolate(world.previous().position,
                        world.current().position,
                        alpha);

        render_console(render_x, wall_x);

        // Debug
        collision_global_time = simulation_time - frame_dt.count() + world.collision_time();
        collision_position = world.current().acceleration * collision_global_time * collision_global_time / 2;
        //Edge triggered collision check
        if (world.check_collision() && !prev_hit) {
            std::cout << "\n[CCD] collision at x=" << collision_position
              << " t=" << collision_global_time << "\n";
        }
        prev_hit = world.check_collision();

        std::this_thread::sleep_for(std::chrono::milliseconds(base_ms + jitter));
    }
}
