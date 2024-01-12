//
// Created by brice on 12/1/23.
//

#ifndef VISUAL_CONSTRAINTSET_H
#define VISUAL_CONSTRAINTSET_H

#include <vector>
#include <cstdint>
#include <functional>
#include "glm/glm.hpp"
#include "glm/gtc/matrix_transform.hpp"

struct SBFaceCenterInfo {
    std::vector<uint32_t> centerIndices;
    float angularVelocity;

        SBFaceCenterInfo(std::vector<uint32_t> centerIndices, float angularVelocity) : centerIndices(centerIndices), angularVelocity(angularVelocity) {}

};

void SBRotationCallback(
        std::unordered_map<uint32_t, uint32_t> verticesLUT,
        const std::vector<uint32_t>& verticesIndex,
        const std::vector<glm::vec3>& startingPos,
        float t,
        std::vector<glm::vec3> &f,
        std::vector<glm::vec3> &v,
        std::vector<glm::vec3> &p,
        const SBFaceCenterInfo &additionalInfo
        );

class ConstraintSet {
public:

    using Callback = std::function<void(
            std::unordered_map<uint32_t, uint32_t> verticesLUT,
            const std::vector<uint32_t>& verticesIndex,
            const std::vector<glm::vec3>& startingPos,
            float t,
            std::vector<glm::vec3> &f,
            std::vector<glm::vec3> &v,
            std::vector<glm::vec3> &p,
            const SBFaceCenterInfo &additionalInfo
        )>;

    ConstraintSet(const std::vector<uint32_t>& vertices, Callback &callback, SBFaceCenterInfo *info)
        : mVertices(vertices), mConstraintCallback(callback), mAdditionalInfos(info)
    {}

    void ApplyConstraint(std::vector<glm::vec3> &f, std::vector<glm::vec3> &v, std::vector<glm::vec3> &p, float t);
    void SetStartingPositions(std::vector<glm::vec3> &p) {
        mVerticesStartingPos.reserve(mVertices.size());
        for (auto index: mVertices) {
            mVerticesLUT.insert({mVerticesStartingPos.size(), index}); // TODO: could be passed instead of recomputed ?
            mVerticesStartingPos.push_back(p[index]);
        }
    }

    std::vector<uint32_t> GetConstraints() { return mVertices; }

    float freq = 1.; // TODO move

private:

    std::vector<uint32_t>   mVertices;
    std::vector<glm::vec3>  mVerticesStartingPos;
    Callback                mConstraintCallback;
    SBFaceCenterInfo       *mAdditionalInfos;

    std::unordered_map<uint32_t, uint32_t> mVerticesLUT;

};


#endif //VISUAL_CONSTRAINTSET_H
