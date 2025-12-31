//
// Created by oguzh on 31.12.2025.
//

#ifndef VEC2_H
#define VEC2_H

struct Vec2 {
    double x;
    double y;

    Vec2() = default;
    Vec2(double x_, double y_) : x(x_), y(y_) {}

    Vec2 operator+(const Vec2& rhs) const { return {x + rhs.x, y + rhs.y}; }
    Vec2 operator-(const Vec2& rhs) const { return {x - rhs.x, y - rhs.y}; }
    Vec2 operator*(double s) const { return {x * s, y * s}; }

    double dot(const Vec2& rhs) const { return x * rhs.x + y * rhs.y; }
    double lengthSq() const { return x * x + y * y; }
};
#endif //VEC2_H
