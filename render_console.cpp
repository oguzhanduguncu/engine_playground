//
// Created by oguzh on 29.12.2025.
//
#include <iostream>
#include <algorithm>
#include "render_console.h"
#include <thread>
#include <chrono>
#include <cmath>


double interpolate(double prev, double curr, double alpha) {
    return prev + (curr - prev) * alpha;
}

Vec2 interpolate(const Vec2& prev,
                 const Vec2& curr,
                 double alpha)
{
    return Vec2{
        interpolate(prev.x, curr.x, alpha),
        interpolate(prev.y, curr.y, alpha)
    };
}


void render_console(double x, double wall_x) {
    constexpr int width = 60;
    int pos = static_cast<int>((x / wall_x) * width);
    pos = std::clamp(pos, 0, width - 1);

    std::string line(width, '-');
    line[pos] = 'o';
    line[width - 1] = '|';
    std::cout << "\33[2K\r"; // satırı temizle + başa dön
    std::cout << line << std::flush;
}

void render_console_2d(const Vec2& pos,
                       double wall_x,
                       int width = 60,
                       int height = 20)
{
    double world_y_max = wall_x * static_cast<double>(height) / width;

    double nx = pos.x / wall_x;
    double ny = pos.y / world_y_max;

    nx = std::clamp(nx, 0.0, 1.0);
    ny = std::clamp(ny, 0.0, 1.0);

    int sx = static_cast<int>(nx * (width  - 1) + 0.5);
    int sy = static_cast<int>(ny * (height - 1) + 0.5);

    std::vector<std::string> grid(height, std::string(width, ' '));

    for (int r = 0; r < height; ++r)
        grid[r][width - 1] = '|';

    int draw_y = height - 1 - sy;
    grid[draw_y][sx] = 'o';

    std::cout << "\33[2J\33[H";
    for (const auto& row : grid)
        std::cout << row << '\n';

    std::cout << std::flush;
}
