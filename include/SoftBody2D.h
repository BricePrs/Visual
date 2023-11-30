//
// Created by brice on 11/28/23.
//

#ifndef VISUAL_SOFTBODY2D_H
#define VISUAL_SOFTBODY2D_H

#include "Mesh.h"

struct PhysicsParam {
    float g;
    float a;
    float k;
    float maxT;
};

struct DampedSpring {

    bool enabled = true;

    float k;
    float a;
    float l0;

    uint32_t i;
    uint32_t j;

    uint32_t n;

    DampedSpring(float k, float a, float l0, uint32_t i, uint32_t j, uint32_t n) : k(k), a(a), l0(l0), i(i), j(j), n(n) {}

    [[nodiscard]]
    float ComputeForce(float l, float rel_speed) const {
        float spring = -k*(l-l0);
        float dampling = a*rel_speed;
        //return spring - dampling;
        return spring*std::max(0., std::abs(l-l0)-0.1*l0) - dampling;
    }
};

class SoftBody2D : public InteractiveObject {
public:

    static SoftBody2D Rectangle(uint32_t l1, uint32_t l2, PhysicsParam params);
    static SoftBody2D Cube(uint32_t l1, uint32_t l2, uint32_t l3, PhysicsParam params);

    void PhysicsStep(float dt, const std::vector<Collider*> &colliders);
    void Draw(const PerspectiveCamera &camera) override;


private:

    SoftBody2D(const std::vector<SimpleColorVertex> &vertices, const std::vector<uint32_t> &indices, const std::vector<uint32_t> &constaints, PhysicsParam params);

    void InitMesh();
    void BuildMesh();

    void ResetForces();
    void ComputeForces();
    void ApplyConstaints(float dt);
    void MoveMasses(float dt);
    void SolveCollisions(const std::vector<Collider*> &colliders, float dt);

    void InitMeshData();
    void UpdateMeshVertices();


    std::vector<SimpleColorVertex>      mVertices;
    std::vector<glm::vec3>              mForces;
    std::vector<glm::vec3>              mVelocities;
    std::vector<uint32_t>               mIndices;
    std::vector<uint32_t>               mConstraints;

    std::vector<SimpleColorVertex>      mMeshVertices;
    std::vector<uint32_t>               mMeshIndices;

    float                               mVertexMass;

    std::vector<DampedSpring>           mSprings;

    Mesh<SimpleColorVertex>*            mMesh;
    WireframeBox*                       mBoundingBox;

    PhysicsParam                        mPhysicsParams;
    glm::vec3                           mAABBLower;
    glm::vec3                           mAABBUpper;
};


#endif //VISUAL_SOFTBODY2D_H
