//
// Created by brice on 12/1/23.
//

#include "ConstraintSet.h"

void ConstraintSet::ApplyConstraint(std::vector<glm::vec3> &f) {
    for (auto index: mVertices) {
        f[index] = glm::vec3(0.);
    }
}
