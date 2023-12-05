//
// Created by brice on 12/1/23.
//

#ifndef VISUAL_CONSTRAINTSET_H
#define VISUAL_CONSTRAINTSET_H

#include <vector>
#include <cstdint>
#include "glm/glm.hpp"


class ConstraintSet {
public:

    ConstraintSet(const std::vector<uint32_t>& vertices) : mVertices(vertices) {}

    void ApplyConstraint(std::vector<glm::vec3>& f);

private:

    std::vector<uint32_t> mVertices;

};


#endif //VISUAL_CONSTRAINTSET_H
