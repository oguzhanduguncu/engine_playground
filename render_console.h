//
// Created by oguzh on 29.12.2025.
//

#ifndef RENDER_CONSOLE_H
#define RENDER_CONSOLE_H
#include <vector>
#include <string>

#include "contact_manifold.h"
#include "glm/vec2.hpp"

struct ScreenPoint {
    int x;
    int y;
};

float interpolate(float prev, float curr, float alpha);
void render_console(float x, float wall_x);
glm::vec2 interpolate(const glm::vec2& prev,
                 const glm::vec2& curr,
                 float alpha);
void render_console_2d(const glm::vec2& pos,
                       float wall_x,
                       int width,
                       int height,
                       const std::vector<ContactManifold>& manifolds);
inline ScreenPoint world_to_screen(
    const glm::vec2& p,
    float wall_x,
    int width,
    int height
);

void draw_contact_point(
    std::vector<std::string>& grid,
    const glm::vec2& p,
    float wall_x
);

void draw_contact_normal(
    std::vector<std::string>& grid,
    const glm::vec2& p,
    const glm::vec2& n,
    float wall_x
);

void render_smoke_test();

#endif //RENDER_CONSOLE_H
