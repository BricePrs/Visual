//
// Created by brice on 12/18/23.
//

#ifndef VISUAL_OCTREE_H
#define VISUAL_OCTREE_H

#include <optional>
#include <memory>
#include "glm/vec3.hpp"
#include "Drawable.h"
#include "Mesh.h"

class Octree : public Drawable {
public:

    Octree(int maxDepth, glm::vec3 minBound, glm::vec3 maxBound, bool drawable,
           const std::optional<std::vector<std::vector<bool>>> &pruningTable,
           glm::ivec3 location = glm::ivec3(0), uint32_t childIndexStride = 2);

    Octree(int maxDepth, glm::vec3 minBound, glm::vec3 maxBound, bool drawable);

    void Draw(const PerspectiveCamera& camera, Shader& shader) override;

private:

    int mDepth;
    glm::vec3 mMinBound;
    glm::vec3 mMaxBound;
    std::optional<std::array<std::optional<std::unique_ptr<Octree>>, 8>> mChildren;
    std::optional<WireframeBox> mBoxMesh;


};


#endif //VISUAL_OCTREE_H
