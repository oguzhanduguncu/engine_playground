cmake_minimum_required(VERSION 3.16)

project(engineloop LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()

add_executable(engineloop
        physics_world.cpp
        Integrator.cpp
        physics_world_variable.cpp
        HitInfo.cpp
        render_console.cpp
        World.cpp
        main.cpp
)

target_compile_options(engineloop PRIVATE
        -Wall
        -Wextra
        -Wpedantic
)
if(UNIX)
    target_link_libraries(engineloop PRIVATE m)
endif()
