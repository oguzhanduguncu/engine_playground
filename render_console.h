//
// Created by oguzh on 29.12.2025.
//

#ifndef RENDER_CONSOLE_H
#define RENDER_CONSOLE_H
#include "vec2.h"


double interpolate(double prev, double curr, double alpha);
void render_console(double x, double wall_x);
Vec2 interpolate(const Vec2& prev,
                 const Vec2& curr,
                 double alpha);
void render_console_2d(const Vec2& pos,
                       double wall_x,
                       int width,
                       int height);

#endif //RENDER_CONSOLE_H
