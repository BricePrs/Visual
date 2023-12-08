//
// Created by brice on 11/28/23.
//

#include <chrono>
#include <softbody2D.cuh>
#include <unordered_map>
#include "SoftBody2D.h"
#include "imgui/imgui.h"

SoftBody2D::SoftBody2D(
        const std::vector<glm::vec3> &positions,
        const std::vector<std::tuple<std::vector<uint32_t>, DampedSpringParams>> &springGroupsIndices,
        const std::vector<ConstraintSet> &constraintSets,
        const std::vector<std::vector<uint32_t>> &hullIndices,
        PhysicsParam params,
        DisplayMode mode
) : InteractiveObject(false),
        mPositions(positions), mConstraintSets(constraintSets), mVertexMass(1.), mStructureMesh(nullptr), mPhysicsParams(params),
        mHullIndices(hullIndices), mDisplayMode(mode), mVertexForcesSpots(mPositions.size()), mHullVerticesLUT(hullIndices.size())

{

    uint32_t meshVertexIndex = 0;
    for (auto &springGroup: springGroupsIndices) {
        std::vector<DampedSpring> springs; springs.reserve(std::get<0>(springGroup).size()/2);
        for (auto index = std::get<0>(springGroup).begin(); index < std::get<0>(springGroup).end(); index+=2) {
            springs.emplace_back(*index, *(index+1), meshVertexIndex, mVertexForcesSpots[*index], mVertexForcesSpots[*(index+1)]);
            ++mVertexForcesSpots[*index];
            ++mVertexForcesSpots[*(index+1)];
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

    for (int i = 0; i < 4; ++i) {
        mRK_kForces.emplace_back();
        mRK_kForces[i].reserve(mPositions.size());
        mRK_kVelocities.emplace_back();
        mRK_kVelocities[i].reserve(mPositions.size());
    }

#ifdef ENABLE_CUDA
    InitCudaResources();
#endif
}

void SoftBody2D::InitCudaResources() {
    mCuda_Resources = {};
    mCuda_Resources.MassCount = mPositions.size();
    for (auto& springGroup: mSpringGroups) {
        mCuda_Resources.SpringCount += springGroup.GetSprings().size();
        mCuda_Resources.SpringGroups.emplace_back(springGroup.params, nullptr, 0);
        mCuda_Resources.SpringGroups[mCuda_Resources.SpringGroups.size() - 1].springCount = springGroup.GetSprings().size();
        cudaMalloc(reinterpret_cast<void **>(&mCuda_Resources.SpringGroups[mCuda_Resources.SpringGroups.size() - 1].springs), springGroup.GetSprings().size() * sizeof (DampedSpring));
        cudaMemcpy(mCuda_Resources.SpringGroups[mCuda_Resources.SpringGroups.size() - 1].springs, springGroup.GetSprings().data(), springGroup.GetSprings().size() * sizeof (DampedSpring), cudaMemcpyHostToDevice);
    }

    mCuda_Resources.ForcesPerMass = 26;
    cudaMalloc(reinterpret_cast<void **>(&mCuda_Resources.Forces), mCuda_Resources.ForcesPerMass * mCuda_Resources.MassCount * sizeof (glm::vec3)); // 1D: 6, 2D: 12, 3D: 8, Total 26 / 100 000 vertex / force 12 Bytes

    cudaMalloc(reinterpret_cast<void **>(&mCuda_Resources.TotalForces), mCuda_Resources.MassCount * sizeof (glm::vec3));
    cudaMalloc(reinterpret_cast<void **>(&mCuda_Resources.Velocities), mCuda_Resources.MassCount * sizeof (glm::vec3));
    cudaMalloc(reinterpret_cast<void **>(&mCuda_Resources.Positions), mCuda_Resources.MassCount * sizeof (glm::vec3));
    cudaMalloc(reinterpret_cast<void **>(&mCuda_Resources.Colors), mCuda_Resources.MassCount * sizeof (glm::vec3));

    cudaMemcpy(mCuda_Resources.Positions, mPositions.data(), mCuda_Resources.MassCount * sizeof (glm::vec3), cudaMemcpyHostToDevice);
    cudaMemcpy(mCuda_Resources.Colors, mColors.data(), mCuda_Resources.MassCount * sizeof (glm::vec3), cudaMemcpyHostToDevice);

    for (auto &constraintSet: mConstraintSets) {
        mCuda_Resources.ConstraintSets.emplace_back(constraintSet.GetConstraints());
    }

}

std::vector<uint32_t> BuildFaceIndices(
        uint32_t strideA,
        uint32_t strideB,
        uint32_t maxA,
        uint32_t maxB,
        uint32_t fixedC
    )
{
    std::vector<uint32_t> indices; indices.reserve(maxA/strideA*maxB/strideB);

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
    return indices;
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

    std::vector<std::vector<uint32_t>> hullIndices = { BuildFaceIndices(l2, 1, l1 * l2, l2, 0)};

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

    for (int i = 0; i < l1-1; i++) {
        indices1D.push_back(l1*l2*l3-1-i*l2*l3);
        indices1D.push_back(l1*l2*l3-1-(i+1)*l2*l3);
    }
    for (int i = 0; i < l2-1; i++) {
        indices1D.push_back(l1*l2*l3-1-i*l3);
        indices1D.push_back(l1*l2*l3-1-(i+1)*l3);
    }
    for (int i = 0; i < l3-1; i++) {
        indices1D.push_back(l1*l2*l3-1-i);
        indices1D.push_back(l1*l2*l3-1-(i+1));
    }


#if 0
    for (int i = 0; i < l1; ++i) {
        for (int k = 0; k < l3; ++k) {
            constraints.push_back(i*l2*l3+k);
        }
    }
#endif


    DampedSpringParams sp1D = {1., 0., 1., 0.0, 10.};
    DampedSpringParams sp2D = {1., 0., std::sqrt(2.f), 0., 10.};
    DampedSpringParams sp3D = {1., 0., std::sqrt(3.f), 0., 10.};
    std::vector<std::tuple<std::vector<uint32_t>, DampedSpringParams>> springGroups = {{indices1D, sp1D}, {indices2D, sp2D}, {indices3D, sp3D}};
    std::vector<ConstraintSet> constraintSets = { constraints };

    std::vector<std::vector<uint32_t>> hullIndices = {
            BuildFaceIndices(l3, l2 * l3, l2 * l3, l1 * l2 * l3, 0),
            BuildFaceIndices(l2*l3, l3, l1*l2*l3, l2*l3, l3-1),
            BuildFaceIndices(1, l3, l3, l2*l3, 0),
            BuildFaceIndices(l3, 1, l2*l3, l3, (l1-1)*l2*l3),
            BuildFaceIndices(l2*l3, 1, l1*l2*l3, l3, 0),
            BuildFaceIndices(1, l2*l3, l3, l1*l2*l3, (l2-1)*l3)
    };


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

void SoftBody2D::CudaBuildMesh() {
    if (!mStructureMesh) {
        InitMesh();
        return;
    }
    CudaUpdateMeshVertices();
    mStructureMesh->ChangeVertices(mStructureMeshVertices);
    mHullMesh->ChangeVertices(mHullMeshVertices);
    mHullMesh->RecomputeVerticesAttributes();
}


void SoftBody2D::ApplyConstraints() {
    for (auto &constraintSet : mConstraintSets) {
        constraintSet.ApplyConstraint(mPositions, mVelocities, mForces, mPhysicsParams.t);
    }
}

void SoftBody2D::CudaApplyConstraints() {
    Parallel_CudaApplyConstraints(mCuda_Resources, mPhysicsParams.t);
}

void SoftBody2D::PhysicsStep(const std::vector<Collider*> &colliders) {
    SolveODE(colliders);
    BuildMesh();
    mPhysicsParams.t += mPhysicsParams.dt;
}

void SoftBody2D::RkPhysicsStep(const std::vector<Collider*> &colliders) {
    ComputeRKCoefs(colliders);
    BuildMesh();
    mPhysicsParams.t += mPhysicsParams.dt;
}

void SoftBody2D::SolveODE(const std::vector<Collider*> &colliders) {
    ResetForces();
    ComputeForces();
    ApplyConstraints();
    SolveCollisions(colliders);
    MoveMasses();
}

void SoftBody2D::CudaPhysicsStep(const std::vector<Collider*> &colliders) {
    CudaResetForces();
    CudaComputeForces();
    CudaApplyConstraints();
    //CudaSolveCollisions(colliders);
    CudaMoveMasses();
    CudaBuildMesh();
    mPhysicsParams.t += mPhysicsParams.dt;
}

void SoftBody2D::ResetForces() {
    for (auto& force: mForces) {
        force = glm::vec3(0., mPhysicsParams.g, 0.);
    }
}

void SoftBody2D::CudaResetForces() {
    Parallel_CudaSetForces(mCuda_Resources, glm::vec3(0., mPhysicsParams.g, 0.));
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

void SoftBody2D::CudaComputeForces() {
    Parallel_CudaComputeSpringMassForces(mCuda_Resources);
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

void SoftBody2D::BackupSystem() {
    mPositionsBackup = mPositions;
    mVelocitiesBackup = mVelocities;
}

void SoftBody2D::ApplyBackup() {
    mPositions = mPositionsBackup;
    mVelocities = mVelocitiesBackup;
}

void SoftBody2D::ResetRkCoefs() {
    for (int i = 0; i < 4; ++i) {
        mRK_kForces[i] = std::vector<glm::vec3>(mPositions.size());
        mRK_kVelocities[i] = std::vector<glm::vec3>(mPositions.size());
    }
}

void SoftBody2D::OffsetODESystem(int i, float offsetCoef) {
    for (uint32_t j = 0; j < mVelocities.size(); ++j) {
        mVelocities[j] += offsetCoef * mRK_kForces[i][j];
    }
    for (uint32_t j = 0; j < mPositions.size(); ++j) {
        mPositions[j] += offsetCoef * mRK_kVelocities[i][j];
    }
}

void SoftBody2D::ComputeRKCoefs(const std::vector<Collider*> &colliders) {

    ResetRkCoefs();

    std::vector<float> stepCoefs = { 0., mPhysicsParams.dt*.5f, mPhysicsParams.dt*.5f, mPhysicsParams.dt };

    for (int i = 0; i < 4; ++i) {
        BackupSystem();
        OffsetODESystem(i, stepCoefs[i]);
        SolveODE(colliders);
        mRK_kForces[i] = mForces;
        mRK_kVelocities[i] = mVelocities;
        ApplyBackup();
    }

    // Reseting AABB bounds
    mAABBLower = mPositions[0];
    mAABBUpper = mPositions[0];

    // Updating velocities and positions and AABB bounds
    for (uint32_t i = 0; i < mPositions.size(); ++i) {
        glm::vec3 k1 = mForces[i] / mVertexMass;
        mVelocities[i] += mPhysicsParams.dt / 6.f * (mRK_kForces[0][i] + 2.f*mRK_kForces[1][i] + 2.f*mRK_kForces[2][i] + mRK_kForces[3][i]);
        mPositions[i] += mPhysicsParams.dt / 6.f * (mRK_kVelocities[0][i] + 2.f * mRK_kVelocities[1][i] + 2.f * mRK_kVelocities[2][i] + mRK_kVelocities[3][i]);
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


void SoftBody2D::CudaMoveMasses() {

    // TODO Compute AABB

    Parallel_CudaMoveMassesKernel(mCuda_Resources, mPhysicsParams.dt, mVertexMass);
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
    uint32_t counter = 0;
    for (uint32_t i = 0; i < mHullIndices.size(); ++i) {
        for (uint32_t &mHullIndex: mHullIndices[i]) {
            if (mHullVerticesLUT[i].find(mHullIndex) == mHullVerticesLUT[i].end()) {
                mHullMeshVertices.emplace_back(mPositions[mHullIndex], glm::vec3(1.));
                mHullVerticesLUT[i].insert({mHullIndex, counter});
                hullMeshIndices.push_back(counter++);
            } else {
                hullMeshIndices.push_back(mHullVerticesLUT[i][mHullIndex]);
            }
        }
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
        for (auto & face : mHullVerticesLUT) {
            for (const auto &vertexLU: face) {
                mHullMeshVertices[vertexLU.second] = SimpleNormalVertex(mPositions[vertexLU.first], glm::vec3(0.));
            }
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

void SoftBody2D::CudaUpdateMeshVertices() {
    cudaMemcpy(mPositions.data(), mCuda_Resources.Positions, mPositions.size() * sizeof (glm::vec3), cudaMemcpyDeviceToHost);
    UpdateMeshVertices();
}

#define MEASURE_TIME_START auto start_time = std::chrono::high_resolution_clock::now();
#define MEASURE_TIME_END auto end_time = std::chrono::high_resolution_clock::now(); \
                           auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time); \
                           std::cout << "Execution Time: " << duration.count() << " microseconds" << std::endl;


void SoftBody2D::SolveCollisions(const std::vector<Collider*> &colliders) {
    for (auto &collider: colliders) {
        if (!collider->Intersect(mAABBLower, mAABBUpper)) {
            continue;
        }

//#define PARALLEL
#ifdef PARALLEL
        //MEASURE_TIME_START
        ParallelSolveCollision(mStructureMesh, collider->GetSphere(), mStructureMesh->GetPosition(), mPhysicsParams.dt);
        //MEASURE_TIME_END
#else
        MEASURE_TIME_START
        for (int i = 0; i < mPositions.size(); ++i) {
            if (collider->Intersect(mPositions[i] + mStructureMesh->GetPosition())) { // TODO : use world coords
                auto path = collider->ShortestSurfacePoint(mPositions[i] + mStructureMesh->GetPosition());
                mPositions[i] += path;
                mVelocities[i] = path/mPhysicsParams.dt; // TODO : not working for many colliders
            }
            //mForces[i] += collider->ComputeCollisionForce(mPositions[i]+mMesh->GetPosition())*.1f;
        }
        MEASURE_TIME_END
#endif
    }
}


void SoftBody2D::DrawWindow() {


    bool shouldReset = false;


    shouldReset = shouldReset || ImGui::SliderFloat("g", &mPhysicsParams.g, -.1, 0);
    shouldReset = shouldReset || ImGui::SliderFloat("dt", &mPhysicsParams.dt, 0, .1);
    shouldReset = shouldReset || ImGui::SliderFloat("frequency", &mConstraintSets[0].freq, 0., .1);
    shouldReset = shouldReset || ImGui::SliderInt("Display mode", reinterpret_cast<int *>(&mDisplayMode), 0, 1);


    for (int i = 0; i < mSpringGroups.size(); ++i) {

        char buff[100];
        snprintf(buff, sizeof(buff), "Spring %i (%zu)", i, mSpringGroups[i].GetSprings().size());
        if (ImGui::CollapsingHeader(buff)) {
            bool shouldUpdateSpringGroup = false;
            shouldUpdateSpringGroup = shouldUpdateSpringGroup || ImGui::SliderFloat("k", &mSpringGroups[i].params.k, 0., 30.);
            shouldUpdateSpringGroup = shouldUpdateSpringGroup || ImGui::SliderFloat("l0", &mSpringGroups[i].params.l0Mult, -1, 3.);
            shouldUpdateSpringGroup = shouldUpdateSpringGroup || ImGui::SliderFloat("a", &mSpringGroups[i].params.a, -1., 1.);
            shouldUpdateSpringGroup = shouldUpdateSpringGroup || ImGui::SliderFloat("Relaxation", &mSpringGroups[i].params.rDist, -1., 1.);
            shouldUpdateSpringGroup = shouldUpdateSpringGroup || ImGui::SliderFloat("Max Tension", &mSpringGroups[i].params.maxT, 0.000001, 1.);
            if (shouldUpdateSpringGroup) {
#ifdef ENABLE_CUDA
                mCuda_Resources.SpringGroups[i].params = mSpringGroups[i].params;
#endif
            }
        }
    }

}

