//
// Created by brice on 12/1/23.
//

#include "ConstraintSet.h"
#include <chrono>


void ConstraintSet::ApplyConstraint(std::vector<glm::vec3> &p, std::vector<glm::vec3> &v, std::vector<glm::vec3> &f, float t) {
    for (auto index: mVertices) {
        //f[index] = glm::vec3(0.);
        v[index] = glm::vec3(glm::sin(t*freq)*1.f*freq, 0., 0.);
        f[index] = glm::vec3(0.f);
    }
}
