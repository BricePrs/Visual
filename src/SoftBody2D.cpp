//
// Created by brice on 11/28/23.
//

#include <chrono>
#include <softbody2D.cuh>
#include "SoftBody2D.h"
#include "imgui/imgui.h"

SoftBody2D::SoftBody2D(
        const std::vector<glm::vec3> &positions,
        const std::vector<std::tuple<std::vector<uint32_t>, DampedSpringParams>> &springGroupsIndices,
        const std::vector<ConstraintSet> &constraintSets,
        const std::vector<uint32_t> &hullIndices,
        PhysicsParam params,
        DisplayMode mode
) : InteractiveObject(false),
        mPositions(positions), mConstraintSets(constraintSets), mVertexMass(1.), mStructureMesh(nullptr), mPhysicsParams(params),
        mHullIndices(hullIndices), mDisplayMode(mode)
{

    uint32_t meshVertexIndex = 0;
    for (auto &springGroup: springGroupsIndices) {
        std::vector<DampedSpring> springs; springs.reserve(std::get<0>(springGroup).size()/2);
        for (auto index = std::get<0>(springGroup).begin(); index < std::get<0>(springGroup).end(); index+=2) {
            springs.emplace_back(*index, *(index+1), meshVertexIndex);
            meshVertexIndex+=2;
        }
        mSpringGroups.emplace_back(springs, std::get<1>(springGroup));
    }

    mColors.reserve(mPositions.size());
    mForces.reserve(mPositions.size());
    mVelocities.reserve(mPositions.size());
    for ([[maybe_unused]] auto &vertex: mPositions) {
        mColors.emplace_back(1);
        mForces.emplace_back(0);
        mVelocities.emplace_back(0.);
    }

    InitMesh();
}

void BuildFaceIndices(std::vector<uint32_t> &indices,
                      uint32_t strideA,
                      uint32_t strideB,
                      uint32_t maxA,
                      uint32_t maxB,
                      uint32_t fixedC)
{

    for (uint32_t i = 0; i < maxA-strideA; i+=strideA) {
        for (uint32_t j = 0; j < maxB-strideB; j+=strideB) {
            indices.push_back(i+j+fixedC);
            indices.push_back(i+j+fixedC+strideA);
            indices.push_back(i+j+fixedC+strideA+strideB);

            indices.push_back(i+j+fixedC);
            indices.push_back(i+j+fixedC+strideA+strideB);
            indices.push_back(i+j+fixedC+strideB);
        }
    }

}

SoftBody2D SoftBody2D::Rectangle(uint32_t l1, uint32_t l2, PhysicsParam params, DisplayMode displayMode) {

    std::vector<glm::vec3> vertices;
    std::vector<uint32_t> indices1D;
    std::vector<uint32_t> indices2D;
    std::vector<uint32_t> constraints;

    uint32_t vertexCount = l1*l2;

    vertices.reserve(vertexCount);
    for (int i = 0; i < l1; i++) {
        for (int j = 0; j < l2; j++) {
            vertices.emplace_back((float)i, (float)j, 0);
        }
    }

    for (int i = 0; i < l1-1; i++) {
        for (int j = 0; j < l2-1; j++) {
            indices1D.push_back(i*l2+j);
            indices1D.push_back((i+1)*l2+j);

            indices2D.push_back(i*l2+j);
            indices2D.push_back((i+1)*l2+(j+1));

            indices1D.push_back(i*l2+j);
            indices1D.push_back(i*l2+(j+1));

            indices2D.push_back((i+1)*l2+j);
            indices2D.push_back(i*l2+(j+1));
        }
    }

    for (int i = 0; i < l1-1; i++) {
        indices1D.push_back(i*l2+l2-1);
        indices1D.push_back((i+1)*l2+l2-1);
    }

    for (int j = 0; j < l2-1; j++) {
        indices1D.push_back((l1-1)*l2+j);
        indices1D.push_back((l1-1)*l2+j+1);
    }

#if 0
    for (int j = 0; j < l2; j++) {
        constraints.push_back(j);
    }
#elif 0
    for (int j = 0; j < l1; j++) {
        constraints.push_back(j*l2-1+l2);
    }
#elif 0
    constraints.push_back(0*l2-1+l2);
    constraints.push_back((l1-1)*l2-1+l2);
#endif

    DampedSpringParams sp1D = {1., 0., 1., 0.0, 10.};
    DampedSpringParams sp2D = {1., 0., 1.414, 0., 10.};
    std::vector<std::tuple<std::vector<uint32_t>, DampedSpringParams>> springGroups = {{indices1D, sp1D}, {indices2D, sp2D}};
    std::vector<ConstraintSet> constraintSets = { constraints };

    std::vector<uint32_t> hullIndices; hullIndices.reserve((l1-1)*(l2-1));
    BuildFaceIndices(hullIndices, l2, 1, l1*l2, l2, 0);

    return { vertices, springGroups, constraintSets, hullIndices, params, displayMode };
}

SoftBody2D SoftBody2D::Cube(uint32_t l1, uint32_t l2, uint32_t l3, PhysicsParam params, DisplayMode displayMode) {
    std::vector<glm::vec3> vertices;
    std::vector<uint32_t> indices1D;
    std::vector<uint32_t> indices2D;
    std::vector<uint32_t> indices3D;
    std::vector<uint32_t> constraints;
    uint32_t vertexCount = l1*l2*l3;

    vertices.reserve(vertexCount);
    for (int i = 0; i < l1; i++) {
        for (int j = 0; j < l2; j++) {
            for (int k = 0; k < l3; k++) {
                vertices.emplace_back(i, j+10., k);
            }
        }
    }

    for (int i = 0; i < l1-1; i++) {
        for (int j = 0; j < l2-1; j++) {
            for (int k = 0; k < l3-1; k++) {

                indices1D.push_back(i*l3*l2+j*l3+k);
                indices1D.push_back((i+1)*l3*l2+j*l3+k);

                indices1D.push_back(i*l3*l2+j*l3+k);
                indices1D.push_back(i*l3*l2+(j+1)*l3+k);

                indices1D.push_back(i*l3*l2+j*l3+k);
                indices1D.push_back(i*l3*l2+j*l3+(k+1));


                indices2D.push_back(i*l3*l2+j*l3+k);
                indices2D.push_back(i*l3*l2+(j+1)*l3+(k+1));

                indices2D.push_back(i*l3*l2+j*l3+k);
                indices2D.push_back((i+1)*l3*l2+j*l3+(k+1));

                indices2D.push_back(i*l3*l2+j*l3+k);
                indices2D.push_back((i+1)*l3*l2+(j+1)*l3+k);


                indices2D.push_back(i*l3*l2+j*l3+(k+1));
                indices2D.push_back(i*l3*l2+(j+1)*l3+k);

                indices2D.push_back((i+1)*l3*l2+j*l3+k);
                indices2D.push_back(i*l3*l2+(j+1)*l3+k);

                indices2D.push_back((i+1)*l3*l2+j*l3+k);
                indices2D.push_back(i*l3*l2+j*l3+(k+1));



                // TODO: Try without

                indices3D.push_back(i*l3*l2+j*l3+k);
                indices3D.push_back((i+1)*l3*l2+(j+1)*l3+(k+1));

                indices3D.push_back(i*l3*l2+j*l3+(k+1));
                indices3D.push_back((i+1)*l3*l2+(j+1)*l3+k);

                indices3D.push_back((i+1)*l3*l2+j*l3+k);
                indices3D.push_back(i*l3*l2+(j+1)*l3+(k+1));

                indices3D.push_back(i*l3*l2+(j+1)*l3+k);
                indices3D.push_back((i+1)*l3*l2+j*l3+(k+1));

            }
        }
    }

    for (int i = 0; i < l1-1; i++) {
        for (int j = 0; j < l2-1; j++) {
            indices1D.push_back(i*l2*l3 + j*l3 + l3-1);
            indices1D.push_back((i+1)*l2*l3 + j*l3 + l3-1);

            indices1D.push_back(i*l2*l3 + (j+1)*l3 + l3-1);
            indices1D.push_back(i*l2*l3 + j*l3 + l3-1);

            indices2D.push_back(i*l2*l3 + j*l3 + l3-1);
            indices2D.push_back((i+1)*l2*l3 + (j+1)*l3 + l3-1);

            indices2D.push_back(i*l2*l3 + (j+1)*l3 + l3-1);
            indices2D.push_back((i+1)*l2*l3 + j*l3 + l3-1);
        }
    }

    for (int j = 0; j < l2-1; j++) {
        for (int k = 0; k < l3-1; k++) {

            indices1D.push_back((l1-1)*l2*l3 + j*l3 + k);
            indices1D.push_back((l1-1)*l2*l3 + j*l3 + (k+1));

            indices1D.push_back((l1-1)*l2*l3 + j*l3 + k);
            indices1D.push_back((l1-1)*l2*l3 + (j+1)*l3 + k);

            indices2D.push_back((l1-1)*l2*l3 + j*l3 + k);
            indices2D.push_back((l1-1)*l2*l3 + (j+1)*l3 + (k+1));

            indices2D.push_back((l1-1)*l2*l3 + (j+1)*l3 + k);
            indices2D.push_back((l1-1)*l2*l3 + j*l3 + (k+1));
        }
    }

    for (int i = 0; i < l1-1; i++) {
        for (int k = 0; k < l3-1; k++) {

            indices1D.push_back(i*l2*l3 + (l2-1)*l3 + k);
            indices1D.push_back(i*l2*l3 + (l2-1)*l3 + (k+1));

            indices1D.push_back(i*l2*l3 + (l2-1)*l3 + k);
            indices1D.push_back((i+1)*l2*l3 + (l2-1)*l3 + k);

            indices2D.push_back(i*l2*l3 + (l2-1)*l3 + k);
            indices2D.push_back((i+1)*l2*l3 + (l2-1)*l3 + (k+1));

            indices2D.push_back((i+1)*l2*l3 + (l2-1)*l3 + k);
            indices2D.push_back(i*l2*l3 + (l2-1)*l3 + (k+1));
        }
    }


#if 0
    for (int i = 0; i < l1; ++i) {
        for (int k = 0; k < l3; ++k) {
            constraints.push_back(i*l2*l3+k);
        }
    }
#endif


    DampedSpringParams sp1D = {1., 0., 1., 0.0, 10.};
    DampedSpringParams sp2D = {1., 0., 1.414, 0., 10.};
    DampedSpringParams sp3D = {1., 0., std::sqrt(3.f), 0., 10.};
    std::vector<std::tuple<std::vector<uint32_t>, DampedSpringParams>> springGroups = {{indices1D, sp1D}, {indices2D, sp2D}, {indices3D, sp3D}};
    std::vector<ConstraintSet> constraintSets = { constraints };

    std::vector<uint32_t> hullIndices; hullIndices.reserve((l1-1)*(l2-1)*(l3-1));
    BuildFaceIndices(hullIndices, l3, l2*l3, l2*l3, l1*l2*l3, 0);
    BuildFaceIndices(hullIndices, l2*l3, l3, l1*l2*l3, l2*l3, l3-1);
    BuildFaceIndices(hullIndices, 1, l3, l3, l2*l3, 0);
    BuildFaceIndices(hullIndices, l3, 1, l2*l3, l3, (l1-1)*l2*l3);
    BuildFaceIndices(hullIndices, l2*l3, 1, l1*l2*l3, l3, 0);
    BuildFaceIndices(hullIndices, 1, l2*l3, l3, l1*l2*l3, (l2-1)*l3);


    return { vertices, springGroups, constraintSets, hullIndices, params, displayMode };
}

void SoftBody2D::InitMesh() {

    std::vector<uint32_t> hullMeshIndices; hullMeshIndices.reserve(mHullIndices.size());
    InitMeshData(hullMeshIndices);

    mHullMesh = new Mesh<SimpleNormalVertex>(mHullMeshVertices, hullMeshIndices);
    mHullMesh->SetPrimitiveMode(GL_TRIANGLES);
    mHullMesh->SetDrawMode(GL_FILL);
    mHullMesh->Translate({1, 1., 0.});

    mStructureMesh = new Mesh<SimpleColorVertex>(mStructureMeshVertices, mStructureMeshIndices);
    mStructureMesh->SetPrimitiveMode(GL_LINES);
    mStructureMesh->SetDrawMode(GL_LINE);
    mStructureMesh->Translate({1, 1., 0.});

    mBoundingBox = new WireframeBox(glm::vec3(0.), glm::vec3(0.), glm::vec3(1, 1, .5));

}

void SoftBody2D::BuildMesh() {
    if (!mStructureMesh) {
        InitMesh();
        return;
    }
    UpdateMeshVertices();
    mStructureMesh->ChangeVertices(mStructureMeshVertices);
    mHullMesh->ChangeVertices(mHullMeshVertices);
    mHullMesh->RecomputeVerticesAttributes();
}

void SoftBody2D::ApplyConstraints() {
    for (auto &constraintSet : mConstraintSets) {
        constraintSet.ApplyConstraint(mForces);
    }
}

void SoftBody2D::PhysicsStep(const std::vector<Collider*> &colliders) {
    ResetForces();
    ComputeForces();
    ApplyConstraints();
    SolveCollisions(colliders);
    MoveMasses();
    BuildMesh();
}

void SoftBody2D::ResetForces() {
    for (auto& force: mForces) {
        force = glm::vec3(0., mPhysicsParams.g, 0.);
    }
}

void SoftBody2D::ComputeForces() {
    for (auto &springGroup: mSpringGroups) {
        for (auto &spring: springGroup.GetSprings()) {
            auto pi = mPositions[spring.i];
            auto pj = mPositions[spring.j];
            auto n = glm::normalize(pi-pj);
            float relSpeed = glm::dot(n, mVelocities[spring.i]-mVelocities[spring.j]);
            float f = 0.;
            if (spring.enabled){
                f = DampedSpring::ComputeForce(glm::length(pi-pj), relSpeed, springGroup.params);
                if (f > springGroup.params.maxT) {
                    spring.enabled = false;
                    f = 0.;
                }
            }
            mForces[spring.i] += f*n;
            mForces[spring.j] += -f*n;
            mStructureMeshVertices[spring.n].color.y = 1. - std::abs(f) * 5.;
            mStructureMeshVertices[spring.n + 1].color.y = 1. - std::abs(f) * 5.;
            mStructureMeshVertices[spring.n].color.z = 1. - std::abs(f) * 5.;
            mStructureMeshVertices[spring.n + 1].color.z = 1. - std::abs(f) * 5.;
        }
    }
}

float smstep(float x, float a, float b) {
    float t = (x-a)/(b-a);
    if (t < 0) t = 0;
    if (t > 1) t = 1;
    return t*t*(3-2*t);
}

void SoftBody2D::MoveMasses() {

    // Reseting AABB bounds
    mAABBLower = mPositions[0];
    mAABBUpper = mPositions[0];

    // Updating velocities and positions and AABB bounds
    for (uint32_t i = 0; i < mPositions.size(); ++i) {
        mVelocities[i] += mForces[i] * mPhysicsParams.dt / mVertexMass;
        mPositions[i] += mVelocities[i]*mPhysicsParams.dt;
        //mVertices[i].color = glm::vec3(0., smstep(glm::length(mVelocities[i]), 0., .4), 0.1)*0.f;
        mColors[i] = glm::vec3(0.);

        mAABBUpper = glm::max(mAABBUpper, mPositions[i]);
        mAABBLower = glm::min(mAABBLower, mPositions[i]);
        if (mPositions[i].y < -1.f) {
            mVelocities[i].y = (-1.f-mPositions[i].y)/mPhysicsParams.dt;
        }
    }
    //printf("x(%f, %f) y(%f, %f) z(%f, %f)\n", mAABBLower.x, mAABBUpper.x, mAABBLower.y, mAABBUpper.y, mAABBLower.z, mAABBUpper.z);

    mAABBLower += mStructureMesh->GetPosition();
    mAABBUpper += mStructureMesh->GetPosition();

    glm::vec3 centerAABB = (mAABBUpper+mAABBLower)*.5f;
    glm::vec3 sidesAABB = mAABBUpper-centerAABB;

    mBoundingBox->UpdateBox(centerAABB, sidesAABB);

}

void SoftBody2D::Draw(const PerspectiveCamera &camera) {
    if (mDisplayMode == Hull) {
        mHullMesh->Draw(camera);
    } else if (mDisplayMode == Structure) {
        mStructureMesh->Draw(camera);
    }
    mBoundingBox->Draw(camera);
}

void SoftBody2D::InitMeshData(std::vector<uint32_t> &hullMeshIndices) {

    // -- Building Hull rendering mesh data -- //

    mHullMeshVertices.reserve(mHullIndices.size());
    for (int i = 0; i < mHullIndices.size(); ++i) {
        mHullMeshVertices.emplace_back(mPositions[mHullIndices[i]], glm::vec3(1.));
        hullMeshIndices.push_back(i);
    }

    // -- Building Structure rendering mesh data -- //

    size_t meshVertexCount = 0;
    for (auto &springGroups: mSpringGroups) {
        meshVertexCount += springGroups.GetSprings().size()*2;
    }
    mStructureMeshVertices.reserve(meshVertexCount);
    mStructureMeshIndices.reserve(meshVertexCount);
    uint32_t count = 0;
    for (auto &springGroups: mSpringGroups) {
        for (auto &spring: springGroups.GetSprings()) {
            mStructureMeshVertices.emplace_back(mPositions[spring.i], glm::vec3(1.));
            mStructureMeshVertices.emplace_back(mPositions[spring.j], glm::vec3(1.));
            mStructureMeshIndices.emplace_back(count++);
            mStructureMeshIndices.emplace_back(count++);
        }
    }
}

void SoftBody2D::UpdateMeshVertices() {

    if (mDisplayMode == Hull) {
        for (int i = 0; i < mHullIndices.size(); ++i) {
            mHullMeshVertices[i] = SimpleNormalVertex(mPositions[mHullIndices[i]], glm::vec3(0.));
        }
    } else if (mDisplayMode == Structure) {
        for (auto &springGroup: mSpringGroups) {
            for (auto &spring: springGroup.GetSprings()) {
                mStructureMeshVertices[spring.n].position = mPositions[spring.i];
                mStructureMeshVertices[spring.n + 1].position = mPositions[spring.j];
                //mMeshVertices[i].color.g = mVertices[i].color.g;
            }
        }
        for (auto &springGroup: mSpringGroups) {
            for (auto &spring: springGroup.GetSprings()) {
                if (!spring.enabled) {
                    mStructureMeshVertices[spring.n + 1].position = mStructureMeshVertices[spring.n].position;
                }
            }
        }
    }
}

void SoftBody2D::SolveCollisions(const std::vector<Collider*> &colliders) {
    for (auto &collider: colliders) {
        if (!collider->Intersect(mAABBLower, mAABBUpper)) {
            continue;
        }

//#define PARALLEL
#ifdef PARALLEL
        ParallelSolveCollision(mPositions, mVelocities, collider->GetSphere(), mMesh->GetPosition(), mPhysicsParams.dt);
#else
        for (int i = 0; i < mPositions.size(); ++i) {
            if (collider->Intersect(mPositions[i] + mStructureMesh->GetPosition())) { // TODO : use world coords
                auto path = collider->ShortestSurfacePoint(mPositions[i] + mStructureMesh->GetPosition());
                mPositions[i] += path;
                mVelocities[i] = path/mPhysicsParams.dt; // TODO : not working for many colliders
            }
            //mForces[i] += collider->ComputeCollisionForce(mPositions[i]+mMesh->GetPosition())*.1f;
        }
#endif
    }
}

void SoftBody2D::DrawWindow() {


    bool shouldReset = false;

    ImGui::Begin("Menu");

    shouldReset = shouldReset || ImGui::SliderFloat("g", &mPhysicsParams.g, -.1, .1);
    shouldReset = shouldReset || ImGui::SliderFloat("dt", &mPhysicsParams.dt, -.1, .1);


    for (int i = 0; i < mSpringGroups.size(); ++i) {

        char buff[100];
        snprintf(buff, sizeof(buff), "Spring %i", i);

        if (ImGui::CollapsingHeader(buff)) {
            shouldReset = shouldReset || ImGui::SliderFloat("k", &mSpringGroups[i].params.k, 0., 30.);
            shouldReset = shouldReset || ImGui::SliderFloat("l0", &mSpringGroups[i].params.l0Mult, -1, 3.);
            shouldReset = shouldReset || ImGui::SliderFloat("a", &mSpringGroups[i].params.a, -1., 1.);
            shouldReset = shouldReset || ImGui::SliderFloat("Relaxation", &mSpringGroups[i].params.rDist, -1., 1.);
            shouldReset = shouldReset || ImGui::SliderFloat("Max Tension", &mSpringGroups[i].params.maxT, 0.000001, 1.);
        }
    }

    ImGui::Text("Average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
    ImGui::End();

}
