//
// Created by brice on 12/1/23.
//

#include "ConstraintSet.h"
#include <chrono>


void ConstraintSet::ApplyConstraint(std::vector<glm::vec3> &f, std::vector<glm::vec3> &v, std::vector<glm::vec3> &p, float t) {
    mConstraintCallback(mVerticesLUT, mVertices, mVerticesStartingPos, t, f, v, p, *mAdditionalInfos);
}


void SBRotationCallback(
        std::unordered_map<uint32_t, uint32_t> verticesLUT,
        const std::vector<uint32_t>& verticesIndex,
        const std::vector<glm::vec3>& startingPos,
        float t,
        std::vector<glm::vec3> &f,
        std::vector<glm::vec3> &v,
        std::vector<glm::vec3> &p,
        const SBFaceCenterInfo &additionalInfo
) {

    auto center = glm::vec3(0.);
    for (auto pt: additionalInfo.centerIndices) {
        center += startingPos[verticesLUT[pt]];
    }
    center /= additionalInfo.centerIndices.size();

    glm::vec3 axis(0.0f, 0.0f, 1.0f); // Rotate around the Z-axis
    float angleInDegrees = additionalInfo.angularVelocity * t;
    glm::mat3 rotationMatrix = glm::rotate(glm::mat4(1.0f), glm::radians(angleInDegrees), axis);

    for (int i = 0; i < verticesIndex.size(); ++i) {
        auto prevPos = p[verticesIndex[i]];
        auto diff = startingPos[i]-center;
        auto rot_diff = rotationMatrix * diff;

        //p[verticesIndex[i]] = startingPos[i] + (glm::vec3(0., 0., additionalInfo.center.z-25.f))*.01f*t;
        //v[verticesIndex[i]] = glm::vec3(glm::sin(t*additionalInfo.angularVelocity)*0.2f, 0.f, 0.f); // TODO : not 0
        v[verticesIndex[i]] = glm::vec3(0.f); // TODO : not 0
        f[verticesIndex[i]] = glm::vec3(0.f);
    }
}
