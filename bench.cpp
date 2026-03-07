#include "bench.h"
#include <chrono>
#include <format>
#include <iostream>

void bench_run(std::initializer_list<Scenario> scenarios)
{
    std::cout << std::format("{:<36}  {:>8}  {:>12}  {:>10}\n",
                             "scenario", "iters", "total_ms", "us/step");
    std::cout << std::format("{:<36}  {:>8}  {:>12}  {:>10}\n",
                             "------------------------------------",
                             "--------", "------------", "----------");

    for (const auto& s : scenarios) {
        for (int i = 0; i < s.warmup; ++i)
            s.step();

        auto t0 = std::chrono::high_resolution_clock::now();
        for (int i = 0; i < s.iters; ++i)
            s.step();
        auto t1 = std::chrono::high_resolution_clock::now();

        double total_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
        double us_step  = total_ms * 1000.0 / s.iters;

        std::cout << std::format("{:<36}  {:>8}  {:>12.3f}  {:>10.2f}\n",
                                 s.name, s.iters, total_ms, us_step);
    }
}
