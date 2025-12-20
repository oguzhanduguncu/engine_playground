cmake_minimum_required(VERSION 3.31)
project(engineloop)

set(CMAKE_CXX_STANDARD 20)

add_executable(engineloop physics_world.cpp Integrator.cpp physics_world_variable.cpp main.cpp)
