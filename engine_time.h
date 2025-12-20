//
// Created by oguzh on 20.12.2025.
//

#ifndef ENGINE_TIME_H
#define ENGINE_TIME_H
#include <chrono>

namespace engine {

    using clock = std::chrono::steady_clock;
    using time_point = clock::time_point;
    using duration = clock::duration;

    inline time_point now() noexcept {
        return clock::now();
    }

}
#endif //ENGINE_TIME_H
