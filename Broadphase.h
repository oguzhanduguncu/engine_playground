//
// Created by oguzh on 2.03.2026.
//

#ifndef ENGINELOOP_BROADPHASE_H
#define ENGINELOOP_BROADPHASE_H
#include <unordered_map>
#include <vector>

#include "body.h"


class Broadphase
{
public:

    void build(const std::vector<Body>& bodies);

    std::vector<std::pair<int,int>> computePairs();

private:

    struct Cell {
        int x;
        int y;

        bool operator==(const Cell& other) const {
            return x == other.x && y == other.y;
        }
    };
    // Spatial hashing with two great prime numbers
    struct CellHash {
        size_t operator()(const Cell& c) const noexcept {
            constexpr auto prime1 = 73856093;
            constexpr auto prime2 = 19349663;
            return std::hash<int>()(c.x*prime1 ^ c.y*prime2);
        }
    };

    float cellSize = Body::ALLOWED_BODY_SIZE;

    std::unordered_map<Cell,std::vector<int>,CellHash> grid{};
};


#endif //ENGINELOOP_BROADPHASE_H