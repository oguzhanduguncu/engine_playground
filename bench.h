#pragma once
#include <functional>
#include <string_view>

// A single benchmark scenario.
// - setup  : called once before warmup (optional, can be nullptr)
// - step   : the hot loop body, called warmup+iters times
// - warmup : steps to run before measurement starts
// - iters  : measured steps
struct Scenario {
    std::string_view      name;
    std::function<void()> step;
    int                   warmup = 5;
    int                   iters  = 200;
};

// Run all scenarios and print a summary table to stdout.
// Use `perf stat ./bench_sim` or `perf record ./bench_sim` for hardware counters.
void bench_run(std::initializer_list<Scenario> scenarios);
