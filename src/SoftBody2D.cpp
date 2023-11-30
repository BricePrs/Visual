//
// Created by brice on 11/28/23.
//

#include <chrono>
#include "SoftBody2D.h"

static uint32_t RADIUS = 15.;
static float ANGULAR_SPEED = 1.;

SoftBody2D::SoftBody2D(const std::vector<SimpleColorVertex> &vertices, const std::vector<uint32_t> &indices, const std::vector<uint32_t> &constaints, PhysicsParam params) : InteractiveObject(false),
    mVertices(vertices), mIndices(indices), mConstraints(constaints), mVertexMass(1.), mMesh(nullptr), mPhysicsParams(params)
{
    mSprings.reserve(mIndices.size()/2);
    uint32_t meshVertexIndex = 0;
    for (auto index = indices.begin(); index < indices.end(); index+=2) {
        auto pi = vertices[*index].position;
        auto pj = vertices[*(index+1)].position;
        mSprings.emplace_back(mPhysicsParams.k, mPhysicsParams.a, glm::length(pi-pj), *index, *(index+1), meshVertexIndex);
        mSprings.emplace_back(mPhysicsParams.k, mPhysicsParams.a, glm::length(pi-pj), *index, *(index+1), meshVertexIndex);
        meshVertexIndex+=2;
    }

    mForces.reserve(vertices.size());
    mVelocities.reserve(vertices.size());
    for ([[maybe_unused]] auto &vertex: vertices) {
        mForces.emplace_back(0);
        mVelocities.emplace_back(0.);
    }

    InitMesh();
}

SoftBody2D SoftBody2D::Rectangle(uint32_t l1, uint32_t l2, PhysicsParam params) {
    std::vector<SimpleColorVertex> vertices; std::vector<uint32_t> indices; std::vector<uint32_t> constraints;
    uint32_t vertexCount = l1*l2;

    vertices.reserve(vertexCount);
    for (int i = 0; i < l1; i++) {
        for (int j = 0; j < l2; j++) {
            vertices.emplace_back(glm::vec3((float)i, (float)j, 0), glm::vec3(1.f));
        }
    }

    for (int i = 0; i < l1-1; i++) {
        for (int j = 0; j < l2-1; j++) {
            indices.push_back(i*l2+j);
            indices.push_back((i+1)*l2+j);

            indices.push_back(i*l2+j);
            indices.push_back((i+1)*l2+(j+1));

            indices.push_back(i*l2+j);
            indices.push_back(i*l2+(j+1));

            indices.push_back((i+1)*l2+j);
            indices.push_back(i*l2+(j+1));
        }
    }

    for (int i = 0; i < l1-1; i++) {
        indices.push_back(i*l2+l2-1);
        indices.push_back((i+1)*l2+l2-1);
    }

    for (int j = 0; j < l2-1; j++) {
        indices.push_back((l1-1)*l2+j);
        indices.push_back((l1-1)*l2+j+1);
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

    return { vertices, indices, constraints, params };
}

SoftBody2D SoftBody2D::Cube(uint32_t l1, uint32_t l2, uint32_t l3, PhysicsParam params) {
    std::vector<SimpleColorVertex> vertices; std::vector<uint32_t> indices; std::vector<uint32_t> constraints;
    uint32_t vertexCount = l1*l2*l3;

    vertices.reserve(vertexCount);
    for (int i = 0; i < l1; i++) {
        for (int j = 0; j < l2; j++) {
            for (int k = 0; k < l3; k++) {
                vertices.emplace_back(glm::vec3(i, j+10., k), glm::vec3(1.f));
            }
        }
    }

    for (int i = 0; i < l1-1; i++) {
        for (int j = 0; j < l2-1; j++) {
            for (int k = 0; k < l3-1; k++) {

                indices.push_back(i*l3*l2+j*l3+k);
                indices.push_back((i+1)*l3*l2+j*l3+k);

                indices.push_back(i*l3*l2+j*l3+k);
                indices.push_back(i*l3*l2+(j+1)*l3+k);

                indices.push_back(i*l3*l2+j*l3+k);
                indices.push_back(i*l3*l2+j*l3+(k+1));


                indices.push_back(i*l3*l2+j*l3+k);
                indices.push_back(i*l3*l2+(j+1)*l3+(k+1));

                indices.push_back(i*l3*l2+j*l3+k);
                indices.push_back((i+1)*l3*l2+j*l3+(k+1));

                indices.push_back(i*l3*l2+j*l3+k);
                indices.push_back((i+1)*l3*l2+(j+1)*l3+k);


                indices.push_back(i*l3*l2+j*l3+(k+1));
                indices.push_back(i*l3*l2+(j+1)*l3+k);

                indices.push_back((i+1)*l3*l2+j*l3+k);
                indices.push_back(i*l3*l2+(j+1)*l3+k);

                indices.push_back((i+1)*l3*l2+j*l3+k);
                indices.push_back(i*l3*l2+j*l3+(k+1));



                // TODO: Try without

                indices.push_back(i*l3*l2+j*l3+k);
                indices.push_back((i+1)*l3*l2+(j+1)*l3+(k+1));

                indices.push_back(i*l3*l2+j*l3+(k+1));
                indices.push_back((i+1)*l3*l2+(j+1)*l3+k);

                indices.push_back((i+1)*l3*l2+j*l3+k);
                indices.push_back(i*l3*l2+(j+1)*l3+(k+1));

                indices.push_back(i*l3*l2+(j+1)*l3+k);
                indices.push_back((i+1)*l3*l2+j*l3+(k+1));

            }
        }
    }

    for (int i = 0; i < l1-1; i++) {
        for (int j = 0; j < l2-1; j++) {
            indices.push_back(i*l2*l3 + j*l3 + l3-1);
            indices.push_back((i+1)*l2*l3 + j*l3 + l3-1);

            indices.push_back(i*l2*l3 + (j+1)*l3 + l3-1);
            indices.push_back(i*l2*l3 + j*l3 + l3-1);

            indices.push_back(i*l2*l3 + j*l3 + l3-1);
            indices.push_back((i+1)*l2*l3 + (j+1)*l3 + l3-1);

            indices.push_back(i*l2*l3 + (j+1)*l3 + l3-1);
            indices.push_back((i+1)*l2*l3 + j*l3 + l3-1);
        }
    }

    for (int j = 0; j < l2-1; j++) {
        for (int k = 0; k < l3-1; k++) {

            indices.push_back((l1-1)*l2*l3 + j*l3 + k);
            indices.push_back((l1-1)*l2*l3 + j*l3 + (k+1));

            indices.push_back((l1-1)*l2*l3 + j*l3 + k);
            indices.push_back((l1-1)*l2*l3 + (j+1)*l3 + k);

            indices.push_back((l1-1)*l2*l3 + j*l3 + k);
            indices.push_back((l1-1)*l2*l3 + (j+1)*l3 + (k+1));

            indices.push_back((l1-1)*l2*l3 + (j+1)*l3 + k);
            indices.push_back((l1-1)*l2*l3 + j*l3 + (k+1));
        }
    }

    for (int i = 0; i < l1-1; i++) {
        for (int k = 0; k < l3-1; k++) {

            indices.push_back(i*l2*l3 + (l2-1)*l3 + k);
            indices.push_back(i*l2*l3 + (l2-1)*l3 + (k+1));

            indices.push_back(i*l2*l3 + (l2-1)*l3 + k);
            indices.push_back((i+1)*l2*l3 + (l2-1)*l3 + k);

            indices.push_back(i*l2*l3 + (l2-1)*l3 + k);
            indices.push_back((i+1)*l2*l3 + (l2-1)*l3 + (k+1));

            indices.push_back((i+1)*l2*l3 + (l2-1)*l3 + k);
            indices.push_back(i*l2*l3 + (l2-1)*l3 + (k+1));
        }
    }


#if 0
    for (int i = 0; i < l1; ++i) {
        for (int k = 0; k < l3; ++k) {
            constraints.push_back(i*l2*l3+k);
        }
    }
#endif

    return { vertices, indices, constraints, params };
}

void SoftBody2D::InitMesh() {
    InitMeshData();
    mMesh = new Mesh<SimpleColorVertex>(mMeshVertices, mMeshIndices);
    mMesh->SetPrimitiveMode(GL_LINES);
    mMesh->SetDrawMode(GL_LINE);
    mMesh->Translate({1, 1., 0.});
    mBoundingBox = new WireframeBox(glm::vec3(0.), glm::vec3(0.), glm::vec3(1, 1, .5));

}

void SoftBody2D::BuildMesh() {
    if (!mMesh) {
        InitMesh();
        return;
    }
    UpdateMeshVertices();
    mMesh->ChangeMeshVertexData(mMeshVertices);
}

void SoftBody2D::ApplyConstaints(float dt) {
    static auto StartTime = std::chrono::high_resolution_clock::now();
    auto time = std::chrono::high_resolution_clock::now();
    double elapsed = std::chrono::duration<double>(time-StartTime).count();
    //float radius = RADIUS;
    for (auto index: mConstraints) {
        //glm::vec3 newPos = glm::vec3(glm::cos(elapsed*0.1), mVertices[index].position.y, mVertices[index].position.z);
        mForces[index] = glm::vec3(0.);
        //mForces[index].x = glm::cos(elapsed*3.)*.2;
        //mForces[index] = glm::vec3(0.);
        //mVelocities[index] = (newPos-mVertices[index].position)/dt;
    }
}

void SoftBody2D::PhysicsStep(float dt, const std::vector<Collider*> &colliders) {
    ResetForces();
    ComputeForces();
    ApplyConstaints(dt);
    MoveMasses(dt);
    SolveCollisions(colliders, dt);
    BuildMesh();
}

void SoftBody2D::ResetForces() {
    for (auto& force: mForces) {
        force = glm::vec3(0., mPhysicsParams.g, 0.);
    }
}

void SoftBody2D::ComputeForces() {
    for (auto &spring: mSprings) {
        auto pi = mVertices[spring.i].position;
        auto pj = mVertices[spring.j].position;
        auto n = glm::normalize(pi-pj);
        float relSpeed = glm::dot(n, mVelocities[spring.i]-mVelocities[spring.j]);
        float f = 0.;
        if (spring.enabled){
            f = spring.ComputeForce(glm::length(pi-pj), relSpeed);
            if (f > mPhysicsParams.maxT) {
                spring.enabled = false;
                f = 0.;
            }
        }
        mForces[spring.i] += f*n;
        mForces[spring.j] += -f*n;
        mMeshVertices[spring.n].color.y = 1.-std::abs(f) * 5.;
        mMeshVertices[spring.n+1].color.y = 1.-std::abs(f) * 5.;
        mMeshVertices[spring.n].color.z = 1.-std::abs(f) * 5.;
        mMeshVertices[spring.n+1].color.z = 1.-std::abs(f) * 5.;
    }
}

float smstep(float x, float a, float b) {
    float t = (x-a)/(b-a);
    if (t < 0) t = 0;
    if (t > 1) t = 1;
    return t*t*(3-2*t);
}

void SoftBody2D::MoveMasses(float dt) {

    // Reseting AABB bounds
    mAABBLower = mVertices[0].position;
    mAABBUpper = mVertices[0].position;

    // Updating velocities and positions and AABB bounds
    for (uint32_t i = 0; i < mVertices.size(); ++i) {
        mVelocities[i] += mForces[i] * dt / mVertexMass;
        mVertices[i].position += mVelocities[i]*dt;
        //mVertices[i].color = glm::vec3(0., smstep(glm::length(mVelocities[i]), 0., .4), 0.1)*0.f;
        mVertices[i].color = glm::vec3(0.);

        mAABBUpper = glm::max(mAABBUpper, mVertices[i].position);
        mAABBLower = glm::min(mAABBLower, mVertices[i].position);
        if (mVertices[i].position.y < -1.f) {
            mVelocities[i].y = (-1.f-mVertices[i].position.y)/dt;
        }
    }
    //printf("x(%f, %f) y(%f, %f) z(%f, %f)\n", mAABBLower.x, mAABBUpper.x, mAABBLower.y, mAABBUpper.y, mAABBLower.z, mAABBUpper.z);

    mAABBLower += mMesh->GetPosition();
    mAABBUpper += mMesh->GetPosition();

    glm::vec3 centerAABB = (mAABBUpper+mAABBLower)*.5f;
    glm::vec3 sidesAABB = mAABBUpper-centerAABB;

    mBoundingBox->UpdateBox(centerAABB, sidesAABB);

}

void SoftBody2D::Draw(const PerspectiveCamera &camera) {
    mMesh->Draw(camera);
    mBoundingBox->Draw(camera);
}

void SoftBody2D::InitMeshData() {
    mMeshVertices.reserve(mIndices.size());
    mMeshIndices.reserve(mIndices.size());
    for (int i = 0; i < mIndices.size(); ++i) {
        mMeshVertices.emplace_back(mVertices[mIndices[i]]);
        mMeshIndices.emplace_back(i);
    }
}

void SoftBody2D::UpdateMeshVertices() {
    for (unsigned int i = 0; i < mMeshIndices.size(); ++i) {
        mMeshVertices[i].position = mVertices[mIndices[i]].position;
        //mMeshVertices[i].color.g = mVertices[i].color.g;
    }
    for (auto &spring: mSprings)
    {
        if (!spring.enabled) {
            mMeshVertices[spring.n + 1].position = mMeshVertices[spring.n].position;
        }
    }
}

void SoftBody2D::SolveCollisions(const std::vector<Collider*> &colliders, float dt) {
    for (auto &collider: colliders) {
        if (!collider->Intersect(mAABBLower, mAABBUpper)) {
            continue;
        }
        for (int i = 0; i < mVertices.size(); ++i) {
            if (collider->Intersect(mVertices[i].position+mMesh->GetPosition())) {
                auto path = collider->ShortestSurfacePoint(mVertices[i].position+mMesh->GetPosition());
                mVertices[i].position += path;
                mVelocities[i] = path/dt; // TODO : not working for many colliders
            }
        }
    }
}
