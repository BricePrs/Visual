//
// Created by brice on 12/18/23.
//

#include "Octree.h"

Octree::Octree(int maxDepth, glm::vec3 minBound, glm::vec3 maxBound, bool drawable)
    : Octree(maxDepth, minBound, maxBound, drawable, {})
{}

void Octree::Draw(const PerspectiveCamera &camera, Shader& shader) {

    if (mBoxMesh.has_value()) {
        mBoxMesh.value().Draw(camera, shader);
    }
    if (mChildren.has_value()) {
        for (auto &child: mChildren.value()) {
            if (child.has_value()) {
                child.value()->Draw(camera, shader);
            }
        }
    }
}

Octree::Octree(int maxDepth, glm::vec3 minBound, glm::vec3 maxBound, bool drawable,
               const std::optional<std::vector<std::vector<bool>>> &pruningTable, glm::ivec3 location, uint32_t childIndexStride)
        : mMinBound(minBound), mMaxBound(maxBound), mDepth(maxDepth)
{
    glm::vec3 center = (maxBound+minBound)*.5f;
    glm::vec3 halfWidth = (maxBound-minBound)*.5f;

    if (drawable || (maxDepth == 0)) {
        mBoxMesh = std::make_optional(WireframeBox(center, halfWidth, glm::vec3(1.f)));
    }

    if (maxDepth == 0) {
        return;
    }


    //for (int i = 3; i > maxDepth; --i) {
    //    printf("\t");
    //}
    //printf("Depth(%i) location(%i,%i,%i)\n", maxDepth, location.x, location.y, location.z);
    //std::flush(std::cout);
    //if (maxDepth == 2) {
    //    for (int i = 0; i < pruningTable->size(); ++i) {
    //        printf("Searching depth %i\n", i);
    //        for (int j = 0; j < pruningTable.value()[i].size(); ++j) {
    //            if (pruningTable.value()[i][j]) {
    //                glm::ivec3 coords;
    //                int d = 2 << (2-i);
    //                coords.x = j / d / d;
    //                coords.y = (j-coords.x*d*d) / d;
    //                coords.z = (j - coords.y*d - coords.x*d*d);
    //                //printf("%i (%i %i %i) is true\n", j, coords.x, coords.y, coords.z);
    //            }
    //        }
    //    }
    //    std::cout << std::endl;
    //}

    auto children = std::array<std::optional<std::unique_ptr<Octree>>, 8>();
    for (size_t i = 0; i < 2; ++i) {
        for (size_t j = 0; j < 2; ++j) {
            for (size_t k = 0; k < 2; ++k) {
                glm::ivec3 childLocation = location*2+glm::ivec3(i, j, k);
                size_t childIndex = childLocation.x * childIndexStride * childIndexStride + childLocation.y * childIndexStride + childLocation.z;
                for (int i = 3; i > maxDepth-1; --i) {
                    printf("\t");
                }
                //printf("Testing childLocation(%i,%i,%i) at array loc %i\n", childLocation.x, childLocation.y, childLocation.z, childIndex);
                //std::flush(std::cout);
                if (pruningTable.has_value() && !pruningTable.value()[static_cast<size_t>(maxDepth)-1][childIndex]) { continue; }
                //for (int i = 3; i > maxDepth-1; --i) {
                //    printf("\t");
                //}
                //printf("Kept\n");
                //std::flush(std::cout);
                children[4*i+2*j+k] = std::make_optional(std::make_unique<Octree>(
                        mDepth - 1,
                        minBound + glm::vec3(i, j, k) * halfWidth,
                        minBound + glm::vec3(i + 1, j + 1, k + 1) * halfWidth,
                        drawable,
                        pruningTable,
                        childLocation,
                        childIndexStride*2
                ));
            }
        }
    }
    mChildren = std::make_optional(std::move(children));
}
