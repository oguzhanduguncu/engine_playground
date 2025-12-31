//
// Created by oguzh on 28.12.2025.
//

#ifndef HIT_INFO_H
#define HIT_INFO_H
#include "vec2.h"


class HitInfo {
    bool state = false;
    double t = 0.0;

    Vec2 point;
    Vec2 normal;

public:
    HitInfo() = default;

    bool hit() const;
    void hit(bool s);

    double time() const;
    void time(double t_hit);

    const Vec2& contactPoint() const;
    const Vec2& contactNormal() const;

    void setContactPoint(const Vec2& p);
    void setContactNormal(const Vec2& n);
};



#endif //HIT_INFO_H
