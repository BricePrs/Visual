//
// Created by brice on 1/8/24.
//

#ifndef VISUAL_ANIMATEDJOINT_H
#define VISUAL_ANIMATEDJOINT_H

#include <optional>
#include <memory>
#include "Drawable.h"
#include "Mesh.h"

class AnimatedJoint : public Drawable {
public:

    AnimatedJoint(const std::string &fileName, glm::vec3 position, const std::string &name);

    void Update(double dt);
    void Draw(const PerspectiveCamera &camera) override;
    std::shared_ptr<AnimatedJoint> AddChildren(const std::string& fileName, glm::vec3 relativePosition, const std::string &name = {});

    void SetPosition(const glm::vec3 &position);
    void SetEnd(glm::vec3 v);

    void BuildMesh();
    void ExportBVH(const std::string &filename);

    static inline float ARROW_SIZE = 0.3f;

private:

    void BuildMeshRec(std::vector<SimpleVertex>& vertices, std::vector<uint32_t>& indices, glm::vec3 position);
    void WriteJoint(std::ofstream &outStream, const std::string& type, const std::string &prefix = "");
    void WriteKeyFrame(std::ofstream &outStream, uint32_t frame, glm::quat parentOrientation = glm::quat(glm::vec3(0.f)));


    std::string mName;

    uint32_t mFrameCount;
    double mFrameTime;

    std::vector<glm::quat> mOrientations;
    std::optional<std::vector<glm::vec3>> mPos;
    glm::vec3 mDefaultAcc;

    Arrow3D mArrowX;
    Arrow3D mArrowY;
    Arrow3D mArrowZ;

    glm::vec3 mRelativePosition;
    glm::vec3 mPosition;
    glm::quat mCurrentOrientation;
    std::optional<glm::vec3> mEndPos;
    std::optional<Mesh<SimpleVertex>> mMesh;

    std::vector<std::shared_ptr<AnimatedJoint>> mChildren;
};


#endif //VISUAL_ANIMATEDJOINT_H
