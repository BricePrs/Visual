//
// Created by brice on 12/18/23.
//

#ifndef VISUAL_SIGNEDDISTANCEFIELD_H
#define VISUAL_SIGNEDDISTANCEFIELD_H


#include <cstdint>
#include <vector>
#include <SoftBody.h>
#include "Mesh.h"
#include "Octree.h"

class SignedDistanceField : public Drawable {
public:

    glm::vec3  CoordsToSpace(glm::uvec3 coords) const;
    glm::ivec3  LinearToCoords(uint32_t linear_coords);
    uint32_t    CoordsToLinear(glm::uvec3 coords);

    SignedDistanceField(uint32_t resolution, float size);

    void AddSphere(glm::vec3 center, float radius);
    void AddCapsule(glm::vec3 center, glm::vec3 dir, float length, float radius);

    void BuildMesh();
    std::shared_ptr<SoftBody> BuildSoftbody(PhysicsParam params, SoftBody::DisplayMode displayMode);

    void Draw(const PerspectiveCamera& camera, Shader& shader) override;

private:

    uint32_t                            mResolution;
    float                               mSize;
    std::vector<bool>                   mGrid;
    std::vector<float>                  mField;
    WireframeBox                        mBoundingBox;
    std::optional<Mesh<SimpleVertex>>   mMesh;

};


#endif //VISUAL_SIGNEDDISTANCEFIELD_H
