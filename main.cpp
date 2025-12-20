#include <iostream>
#include <thread>
#include "engine_time.h"
#include "physics_world.h"
#include <random>

#include "physics_world_variable.h"

std::mt19937 rng{std::random_device{}()};
std::uniform_int_distribution<int> jitter_ms(-5, 5);

int main() {
    constexpr double physics_dt = 1.0 / 60.0; // 60 Hz physics
    PhysicsWorld world(physics_dt);
    physics_world_variable world_variable(physics_dt);

    auto last = engine::now();

    for (int frame = 0; frame < 60; ++frame) {
        auto now = engine::now();
        std::chrono::duration<double> frame_dt = now - last;
        last = now;
        int base_ms = 16; // ~60 FPS
        int jitter = jitter_ms(rng); // simulation of unsteady frame pacing

        world.update(frame_dt.count());
        world_variable.update(frame_dt.count());
        std::cout << "--------Fixed timestep--------" << '\n';
        std::cout << "Frame: " << frame
                  << " | Physics steps: "
                  << world.step_count() << '\n';

        std::cout << frame << ", " << world.velocity() << ", " << world.position() << "\n";

        std::cout << "--------Varying Timestep--------" << '\n';
        std::cout << "Frame: " << frame
                  << " | Physics steps: "
                  << world_variable.step_count() << '\n';
        std::cout << frame << ", " << world_variable.velocity() << ", " << world_variable.position() << "\n";


        std::this_thread::sleep_for(std::chrono::milliseconds(base_ms + jitter));
    }
}
