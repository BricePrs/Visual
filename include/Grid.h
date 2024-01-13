//
// Created by brice on 11/25/23.
//

#ifndef VISUAL_GRID_H
#define VISUAL_GRID_H

#include <vector>
#include <cstdint>
#include <iostream>

struct GridSize {
    const uint32_t sizeX;
    const uint32_t sizeY;

    const uint32_t linearSize;

    GridSize(uint32_t sizeX, uint32_t sizeY) : sizeX(sizeX), sizeY(sizeY), linearSize(sizeX * sizeY) {}
};

class Grid {
public:

    explicit Grid(GridSize resolution);

    void RandomFill();
    char* GetData();

    void Serialize(const std::string& filePath);
    static Grid Deserialize(const std::string& filePath, GridSize size);

    friend std::ostream& operator<<(std::ostream& os, const Grid& grid);

private:

    explicit Grid(std::vector<char> data, GridSize size);

    std::vector<char> data;
    GridSize size;
};



#endif //VISUAL_GRID_H
