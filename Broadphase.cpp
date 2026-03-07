//
// Created by oguzh on 2.03.2026.
//

#include "Broadphase.h"

#include <cmath>
#include <iostream>

void Broadphase::build(const std::vector<Body>& bodies)
{
    grid.clear();
    grid.reserve(bodies.size());

    for (size_t i=0;i<bodies.size();i++)
    {
        const Body& b = bodies[i];

        const int cx = std::floor(b.position.x / cellSize);
        const int cy = std::floor(b.position.y / cellSize);

        Cell c{cx,cy};

        grid[c].push_back(i);
//        std::cout << "cell bucket: " << grid.bucket(c) << " of the body ID: " << b.id << std::endl;
    }
}

std::vector<std::pair<int, int>> Broadphase::computePairs()
{
    std::vector<std::pair<int,int>> pairs;

    for (const auto& [cell, indices] : grid) {
        // Check this cell and all 8 neighbors
        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                Cell neighbor{cell.x + dx, cell.y + dy};
                auto it = grid.find(neighbor);
                if (it == grid.end())
                    continue;

                const auto& neighborIndices = it->second;

                if (dx == 0 && dy == 0) {
                    // Same cell: pair each body with every other
                    for (size_t a = 0; a < indices.size(); ++a)
                        for (size_t b = a + 1; b < indices.size(); ++b)
                            pairs.emplace_back(indices[a], indices[b]);
                } else {
                    // Neighbor cell: only emit pair when our index < theirs
                    // to avoid duplicates (each neighbor pair is visited twice)
                    for (int i : indices)
                        for (int j : neighborIndices)
                            if (i < j)
                                pairs.emplace_back(i, j);
                }
            }
        }
    }

    return pairs;
}
