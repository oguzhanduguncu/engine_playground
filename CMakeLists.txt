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
        render_console.cpp
        main.cpp
)

target_include_directories(engineloop PRIVATE external/glm)

if(UNIX)
    target_compile_options(engineloop PRIVATE
            -Wall
            -Wextra
            -Wpedantic
    )

    target_link_libraries(engineloop PRIVATE m)
elseif (MSVC)
    target_compile_options(engineloop PRIVATE
            /W4
            /permissive-
    )
endif()
