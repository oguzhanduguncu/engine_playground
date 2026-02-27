cmake_minimum_required(VERSION 3.16)

project(engineloop LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()

find_package(SDL2 REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(SDL2_IMAGE REQUIRED SDL2_image)

add_executable(engineloop
        render_2d.cpp
        debug_draw.cpp
        physics_world.cpp
        Integrator.cpp
        render_console.cpp
        main.cpp
)

target_link_libraries(engineloop
        PRIVATE
        SDL2::SDL2
        ${SDL2_IMAGE_LIBRARIES}
)

target_include_directories(engineloop PRIVATE external/glm ${SDL2_IMAGE_INCLUDE_DIRS})
target_compile_definitions(engineloop PRIVATE ASSET_DIR="${CMAKE_SOURCE_DIR}/assets/")

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

# --- Testing ---
include(FetchContent)
FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG        v1.14.0
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

enable_testing()

add_executable(engine_tests
    tests/test_integrator.cpp
    tests/test_body.cpp
    tests/test_physics_world.cpp
    tests/test_contact_solver.cpp
    tests/test_ccd.cpp
    tests/test_accumulator.cpp
    physics_world.cpp
    Integrator.cpp
)

target_include_directories(engine_tests PRIVATE ${CMAKE_SOURCE_DIR} external/glm)
target_link_libraries(engine_tests PRIVATE GTest::gtest_main)

if(UNIX)
    target_compile_options(engine_tests PRIVATE -Wall -Wextra -Wpedantic)
    target_link_libraries(engine_tests PRIVATE m)
endif()

include(GoogleTest)
gtest_discover_tests(engine_tests)
