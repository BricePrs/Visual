//
// Created by brice on 12/18/23.
//

#include "SignedDistanceField.h"

void SignedDistanceField::Draw(const PerspectiveCamera &camera, Shader& shader) {
    mBoundingBox.Draw(camera, shader);
    if (mMesh.has_value()) {
        mMesh.value().Draw(camera, shader);
    }
}

SignedDistanceField::SignedDistanceField(uint32_t resolution, float size)
    : mSize(size), mResolution(resolution),
    mBoundingBox(WireframeBox(glm::vec3(0.), glm::vec3(size*0.5), glm::vec3(1., .8, .6)))
{
    mGrid = std::vector<bool>(resolution*resolution*resolution, true);
    mField = std::vector<float>(resolution*resolution*resolution, 10.f*size);
}

glm::ivec3 SignedDistanceField::LinearToCoords(uint32_t linear_coords) {
    uint32_t x = linear_coords / mResolution / mResolution;
    uint32_t y = (linear_coords - x * mResolution * mResolution) / mResolution;
    uint32_t z = linear_coords - x * mResolution * mResolution - y * mResolution;

    return {x, y, z};
}

uint32_t SignedDistanceField::CoordsToLinear(glm::uvec3 coords) {
    return coords.x*mResolution*mResolution+coords.y*mResolution+coords.z;
}

glm::vec3 SignedDistanceField::CoordsToSpace(glm::uvec3 coords) const {
    return ((glm::vec3(coords)/(float)mResolution)-glm::vec3(.5)) * mSize;
}

float SphereSDF(glm::vec3 p, glm::vec3 c, float r) {
    return glm::length(p-c)-r;
}

float CapsuleSDF(glm::vec3 p, glm::vec3 c, glm::vec3 dir, float l, float r) {
    glm::vec3 pp = c + dir * glm::clamp(glm::dot(p-c, dir), -l/2.f, l/2.f);
    return glm::length(p-pp)-r;
}

void SignedDistanceField::AddSphere(glm::vec3 center, float radius) {
    for (uint32_t i = 0; i < mResolution*mResolution*mResolution; ++i) {
        mField[i] = std::min(mField[i], SphereSDF(CoordsToSpace(LinearToCoords(i)), center, radius));
        mGrid[i] = mField[i] < 0.;
    }
}

void SignedDistanceField::AddCapsule(glm::vec3 center, glm::vec3 dir, float length, float radius) {
    for (uint32_t i = 0; i < mResolution*mResolution*mResolution; ++i) {
        mField[i] = std::min(mField[i], CapsuleSDF(CoordsToSpace(LinearToCoords(i)), center, dir, length, radius));
        mGrid[i] = mField[i] < 0.;
    }
}

void SignedDistanceField::BuildMesh() {
    std::vector<SimpleVertex> vertices;
    std::vector<uint32_t> indices;
    int32_t l1 = mResolution;
    int32_t l2 = mResolution;
    int32_t l3 = mResolution;
    uint32_t vertexCount = l1*l2*l3;

    vertices.reserve(vertexCount);
    for (int32_t i = 0; i < l1; i++) {
        for (int32_t j = 0; j < l2; j++) {
            for (int32_t k = 0; k < l3; k++) {
                vertices.emplace_back(
                        static_cast<float>(i-(l1-1)*.5)/static_cast<float>(l1-1)*mSize,
                        static_cast<float>(j-(l2-1)*.5)/static_cast<float>(l2-1)*mSize,
                        static_cast<float>(k-(l3-1)*.5)/static_cast<float>(l3-1)*mSize
                );
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

    for (int i = 0; i < l1-1; i++) {
        indices.push_back(l1*l2*l3-1-i*l2*l3);
        indices.push_back(l1*l2*l3-1-(i+1)*l2*l3);
    }
    for (int i = 0; i < l2-1; i++) {
        indices.push_back(l1*l2*l3-1-i*l3);
        indices.push_back(l1*l2*l3-1-(i+1)*l3);
    }
    for (int i = 0; i < l3-1; i++) {
        indices.push_back(l1*l2*l3-1-i);
        indices.push_back(l1*l2*l3-1-(i+1));
    }

    std::vector<uint32_t> filteredIndices;
    for (uint32_t i = 0; i < indices.size() / 2; ++i) {
        if (mGrid[indices[2*i]] && mGrid[indices[2*i+1]]) {
            filteredIndices.push_back(indices[2*i]);
            filteredIndices.push_back(indices[2*i+1]);
        }
    }

    mMesh = std::make_optional<Mesh<SimpleVertex>>(vertices, filteredIndices);
    mMesh->SetPrimitiveMode(GL_LINES);
    mMesh->SetDrawMode(GL_LINE);

}

bool IsTrianglePresent(std::unordered_set<uint32_t> set, uint32_t a, uint32_t b, uint32_t c) {
    return set.find(a) != set.end() && set.find(b) != set.end() && set.find(c) != set.end();
}

std::shared_ptr<SoftBody> SignedDistanceField::BuildSoftbody(PhysicsParam params, SoftBody::DisplayMode displayMode) {

    std::vector<glm::vec3> vertices;
    std::vector<uint32_t> indices1D;
    std::vector<uint32_t> indices2D;
    std::vector<uint32_t> indices3D;
    int32_t l1 = mResolution;
    int32_t l2 = mResolution;
    int32_t l3 = mResolution;
    uint32_t vertexCount = l1*l2*l3;

    vertices.reserve(vertexCount);
    for (int32_t i = 0; i < l1; i++) {
        for (int32_t j = 0; j < l2; j++) {
            for (int32_t k = 0; k < l3; k++) {
                vertices.emplace_back(
                        static_cast<float>(i-(l1-1)*.5)/static_cast<float>(l1-1)*mSize,
                        static_cast<float>(j-(l2-1)*.5)/static_cast<float>(l2-1)*mSize,
                        static_cast<float>(k-(l3-1)*.5)/static_cast<float>(l3-1)*mSize
                );
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

    std::vector<uint32_t> filteredIndices1D; filteredIndices1D.reserve(indices1D.size());
    std::vector<uint32_t> filteredIndices2D; filteredIndices2D.reserve(indices2D.size());
    std::vector<uint32_t> filteredIndices3D; filteredIndices3D.reserve(indices3D.size());
    for (uint32_t i = 0; i < indices1D.size() / 2; ++i) {
        if (mGrid[indices1D[2*i]] && mGrid[indices1D[2*i+1]]) {
            filteredIndices1D.push_back(indices1D[2*i]);
            filteredIndices1D.push_back(indices1D[2*i+1]);
        }
    }
    for (uint32_t i = 0; i < indices2D.size() / 2; ++i) {
        if (mGrid[indices2D[2*i]] && mGrid[indices2D[2*i+1]]) {
            filteredIndices2D.push_back(indices2D[2*i]);
            filteredIndices2D.push_back(indices2D[2*i+1]);
        }
    }
    for (uint32_t i = 0; i < indices3D.size() / 2; ++i) {
        if (mGrid[indices3D[2*i]] && mGrid[indices3D[2*i+1]]) {
            filteredIndices3D.push_back(indices3D[2*i]);
            filteredIndices3D.push_back(indices3D[2*i+1]);
        }
    }

    std::vector<uint32_t> constraintsBottomIndices = {}; // BuildFaceIndices(l3, l2 * l3, l2 * l3, l1 * l2 * l3, 0);
    std::vector<uint32_t> constraintsTopIndices = {}; // BuildFaceIndices(l2*l3, l3, l1*l2*l3, l2*l3, l3-1);


    std::vector<std::vector<uint32_t>> hullIndices = {{}};

    std::unordered_set<uint32_t> borderIndices = {};
    for (int i = 1; i < l1-1; ++i) {
        for (int j = 1; j < l2-1; ++j) {
            for (int k = 1; k < l3-1; ++k) {
                if (!mGrid[CoordsToLinear(glm::uvec3(i, j, k))]) {
                    continue;
                }
                if (!mGrid[CoordsToLinear(glm::uvec3(i-1, j, k))] ||
                    !mGrid[CoordsToLinear(glm::uvec3(i+1, j, k))] ||
                    !mGrid[CoordsToLinear(glm::uvec3(i, j-1, k))] ||
                    !mGrid[CoordsToLinear(glm::uvec3(i, j+1, k))] ||
                    !mGrid[CoordsToLinear(glm::uvec3(i, j, k-1))] ||
                    !mGrid[CoordsToLinear(glm::uvec3(i, j, k+1))]) {
                    borderIndices.insert(CoordsToLinear(glm::uvec3(i, j, k)));
                }
            }
        }
    }

    for (int i = 0; i < l1-1; ++i) {
        for (int j = 0; j < l2-1; ++j) {
            for (int k = 0; k < l3 - 1; ++k) {
                std::vector<uint32_t> points = {};
                points.push_back(CoordsToLinear(glm::uvec3(i, j, k)));
                points.push_back(CoordsToLinear(glm::uvec3(i+1, j, k)));
                points.push_back(CoordsToLinear(glm::uvec3(i, j+1, k)));
                points.push_back(CoordsToLinear(glm::uvec3(i, j, k+1)));
                points.push_back(CoordsToLinear(glm::uvec3(i+1, j+1, k)));
                points.push_back(CoordsToLinear(glm::uvec3(i, j+1, k+1)));
                points.push_back(CoordsToLinear(glm::uvec3(i+1, j, k+1)));
                points.push_back(CoordsToLinear(glm::uvec3(i+1, j+1, k+1)));
                int count = 0;
                for (auto point: points) {
                    if (borderIndices.find(point)!=borderIndices.end()) {
                        ++count;
                    }
                }
                if (count < 3) { continue; }
                for (int l = 0; l < 6; ++l) {
                    for (int m = l+1; m < 7; ++m) {
                        for (int n = m+1; n < 8; ++n) {
                            if (IsTrianglePresent(borderIndices, points[l], points[m], points[n])) {
                                hullIndices[0].push_back(points[l]);
                                hullIndices[0].push_back(points[m]);
                                hullIndices[0].push_back(points[n]);
                            }
                        }
                    }
                }
            }
        }
    }




    DampedSpringParams sp1D = {1., 0., 1.f*mSize/static_cast<float>(mResolution-1), 0.0, 100.};
    DampedSpringParams sp2D = {1., 0., std::sqrt(2.f)*mSize/static_cast<float>(mResolution-1), 0., 100.};
    DampedSpringParams sp3D = {1., 0., std::sqrt(3.f)*mSize/static_cast<float>(mResolution-1), 0., 100.};
    std::vector<std::tuple<std::vector<uint32_t>, DampedSpringParams>> springGroups = {{filteredIndices1D, sp1D}, {filteredIndices2D, sp2D}, {filteredIndices3D, sp3D}};


    std::vector<ConstraintSet> constraintSets = {
            //{constraintsBottomIndices, callback, bottomCallbackInfo},
            //{constraintsTopIndices, callback, topCallbackInfo}
    };

    return std::make_shared<SoftBody>(vertices, springGroups, constraintSets, hullIndices, params, displayMode, nullptr);

}


