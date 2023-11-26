//
// Created by brice on 11/25/23.
//

#include "Grid.h"
#include <random>
#include <fstream>

Grid::Grid(GridSize size) : size(size) {
    data.reserve(size.linearSize);
    RandomFill();
}

void Grid::RandomFill() {
    for (uint32_t i = 0; i < size.linearSize; ++i) {
        data.push_back(static_cast<char>(rand()));
    }
}

std::ostream& operator<<(std::ostream& os, const Grid &grid) {
    for (uint32_t i = 0; i < grid.size.sizeY; ++i) {
        for (uint32_t j = 0; j < grid.size.sizeX; ++j) {
            os << grid.data[i*grid.size.sizeY+j] << " ";
        }
        os << std::endl;
    }
    return os;
}

void Grid::Serialize(const std::string &filePath) {
    std::ofstream file(filePath, std::ios_base::binary | std::ios_base::trunc | std::ios_base::out);
    if (file) {
        for (uint32_t i = 0; i < size.linearSize; ++i) {
            file << data[i] << " ";
        }
    } else {
        throw std::runtime_error("Could not open file : " + filePath);
    }
}

Grid Grid::Deserialize(const std::string &filePath, GridSize size) {
    std::ifstream file;
    std::vector<char> data;
    data.reserve(size.linearSize);
    file.open(filePath, std::ios_base::binary | std::ios_base::in);
    if (file) {
        char buf;
        while (!file.eof()) {
            file >> buf;
            data.push_back(buf);
        }
        return Grid(data, size);
    }
    throw std::runtime_error("Could not open file : " + filePath);
}

Grid::Grid(std::vector<char> data, GridSize size) : data(data), size(size) {
    if (data.size() != size.linearSize) {
        throw std::runtime_error("Error input data does not match grid size");
    }
}

char *Grid::GetData() {
    return data.data();
}
