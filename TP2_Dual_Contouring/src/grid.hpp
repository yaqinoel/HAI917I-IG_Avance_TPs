#pragma once

#include <vector>

#include <glm/vec3.hpp>

struct Grid {
    glm::ivec3 cellResolution;  // nx, ny, nz
    glm::vec3 cellSize;         // sizeX, sizeY, sizeZ
    glm::vec3 origin;           // bbox.min

    std::vector<float> sdfValues;   // size = (nx+1) * (ny+1) * (nz+1)
    std::vector<int> cellVertexIds;   // size = nx * ny * nz

    Grid(const glm::ivec3& resolution, const glm::vec3& gridOrigin, const glm::vec3& gridCellSize)
    : cellResolution(resolution), cellSize(gridCellSize), origin(gridOrigin) {}

    std::size_t gridVertexIndex(int i, int j, int k) {
        const int nx = cellResolution.x + 1;
        const int ny = cellResolution.y + 1;

        return i + nx * (j + ny * k);
    }

    std::size_t cellVertexIndex(int i, int j, int k) {
        return i + cellResolution.x * (j + cellResolution.y * k);
    }

    glm::vec3 gridVertexPosition(int i, int j, int k) {
        return origin + cellSize * (glm::vec3(i, j, k));
    }

};