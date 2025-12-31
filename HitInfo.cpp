//
// Created by oguzh on 28.12.2025.
//

#include "HitInfo.h"


bool HitInfo::hit() const {
    return state;
}

void HitInfo::hit(bool s) {
     state = s;
}

double HitInfo::time() const {
    return  t;
}

void HitInfo::time(double t_hit) {
    t = t_hit;
}

const Vec2& HitInfo::contactPoint() const { return point; }
const Vec2& HitInfo::contactNormal() const { return normal; }

void HitInfo::setContactPoint(const Vec2& p) { point = p; }
void HitInfo::setContactNormal(const Vec2& n) { normal = n; }