//
// Created by oguzh on 29.12.2025.
//
#include <iostream>
#include <algorithm>
#include "render_console.h"
#include <thread>
#include <chrono>
#include <cmath>

#include "contact.h"

float interpolate(float prev, float curr, float alpha) {
    return prev + (curr - prev) * alpha;
}

glm::vec2 interpolate(const glm::vec2& prev,
                 const glm::vec2& curr,
                 float alpha)
{
    return glm::vec2{
        interpolate(prev.x, curr.x, alpha),
        interpolate(prev.y, curr.y, alpha)
    };
}


void render_console(float x, float wall_x) {
    constexpr int width = 60;
    int pos = static_cast<int>((x / wall_x) * width);
    pos = std::clamp(pos, 0, width - 1);

    std::string line(width, '-');
    line[pos] = 'o';
    line[width - 1] = '|';
    std::cout << "\33[2K\r"; // satırı temizle + başa dön
    std::cout << line << std::flush;
}

void render_console_2d(
    const glm::vec2& pos,
    float wall_x,
    int width,
    int height,
    const std::vector<ContactManifold>& manifolds
) {
    if (width <= 0 || height <= 0)
        return;

    if (!std::isfinite(pos.x) || !std::isfinite(pos.y))
        return;

    // workaround for render
    if (std::abs(wall_x) < 1e-6)
        wall_x = 1.0;

    if (!std::isfinite(wall_x) || std::abs(wall_x) < 1e-6)
        return;

    float world_y_max = wall_x * static_cast<float>(height) / width;
    if (!std::isfinite(world_y_max) || std::abs(world_y_max) < 1e-6)
        return;

    float nx = pos.x / wall_x;
    float ny = pos.y / world_y_max;

    if (!std::isfinite(nx) || !std::isfinite(ny))
        return;

    nx = std::clamp(nx, 0.0f, 1.0f);
    ny = std::clamp(ny, 0.0f, 1.0f);

    int sx = static_cast<int>(nx * (width  - 1));
    int sy = static_cast<int>(ny * (height - 1));

    sx = std::clamp(sx, 0, width  - 1);
    sy = std::clamp(sy, 0, height - 1);

    std::vector<std::string> grid(height, std::string(width, ' '));

    for (int r = 0; r < height; ++r)
        grid[r][width - 1] = '|';

    int draw_y = height - 1 - sy;
    if (draw_y >= 0 && draw_y < height)
        grid[draw_y][sx] = 'o';

    for (const auto& m : manifolds) {
        if (m.pointCount <= 0)
            continue;

        for (int i = 0; i < m.pointCount; ++i) {
            const ContactPoint& cp = m.points[i];

            if (!std::isfinite(cp.position.x) ||
                !std::isfinite(cp.position.y))
                continue;

            if (!std::isfinite(cp.normal.x) ||
                !std::isfinite(cp.normal.y))
                continue;

            draw_contact_point(grid, cp.position, wall_x);
            draw_contact_normal(grid, cp.position, cp.normal, wall_x);
        }
    }
    std::cout << "\33[2J\33[H";
    for (const auto& row : grid)
        std::cout << row << '\n';

    std::cout << std::flush;
}


inline ScreenPoint world_to_screen(
    const glm::vec2& p,
    float wall_x,
    int width,
    int height
) {
    float world_y_max = wall_x * static_cast<float>(height) / width;

    float nx = p.x / wall_x;
    float ny = p.y / world_y_max;

    nx = std::clamp(nx, 0.0f, 1.0f);
    ny = std::clamp(ny, 0.0f, 1.0f);

    int sx = static_cast<int>(nx * static_cast<float>(width  - 1) + 0.5f);
    int sy = static_cast<int>(ny * static_cast<float>(height - 1) + 0.5f);

    return {
        sx,
        height - 1 - sy
    };
}

void draw_contact_point(
    std::vector<std::string>& grid,
    const glm::vec2& p,
    float wall_x
) {
    const size_t height = grid.size();
    const size_t width  = grid[0].size();

    ScreenPoint sp = world_to_screen(p, wall_x, static_cast<int>(width), static_cast<int>(height));
    grid[sp.y][sp.x] = 'x';
}

void draw_contact_normal(
    std::vector<std::string>& grid,
    const glm::vec2& p,
    const glm::vec2& n,
    float wall_x
) {
    int height = grid.size();
    int width  = grid[0].size();

    ScreenPoint base = world_to_screen(p, wall_x, width, height);

    constexpr int len = 3;

    for (int i = 1; i <= len; ++i) {
        glm::vec2 q = {
            p.x + n.x * 0.1 * i,
            p.y + n.y * 0.1 * i
        };

        ScreenPoint sp = world_to_screen(q, wall_x, width, height);

        if (sp.x >= 0 && sp.x < width &&
            sp.y >= 0 && sp.y < height)
        {
            grid[sp.y][sp.x] = (n.x < 0 ? '<' :
                                n.x > 0 ? '>' :
                                n.y > 0 ? '^' : 'v');
        }
    }
}


void render_smoke_test()
{
    std::cout << "\33[2J\33[H";
    std::cout << "SMOKE_TEST_RENDER\n";
    std::cout << "X\n";
    std::cout << std::flush;
}

