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

    void ApplyConstraint(std::vector<glm::vec3> &p, std::vector<glm::vec3> &v, std::vector<glm::vec3> &f, float t);

    std::vector<uint32_t> GetConstraints() { return mVertices; }


    float freq = 1.; // TODO move

private:

    std::vector<uint32_t> mVertices;

};


#endif //VISUAL_CONSTRAINTSET_H
