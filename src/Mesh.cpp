//
// Created by brice on 11/12/23.
//

#include <iostream>
#include <GlobalVar.h>
#include <cstring>

#include "Mesh.h"

template<class TVertex>
void Mesh<TVertex>::OnClick() {
    mIsSelected = !mIsSelected;
}

template<class TVertex>
void Mesh<TVertex>::OnHover() {
    mIsHovered = true;
}

template<class TVertex>
void Mesh<TVertex>::OnHoverQuit() {
    mIsHovered = false;
}

template<class TVertex>
void Mesh<TVertex>::SetPrimitiveMode(GLenum mode) {
    mPrimitiveMode = mode;
}


template<class TVertex>
void Mesh<TVertex>::SetDrawMode(GLenum mode) {
    mDrawMode = mode;
}

template <class TVertex>
Mesh<TVertex>::Mesh(std::vector<TVertex> vertices, std::vector<uint32_t> indices, bool interactive)
        :   InteractiveObject(interactive),
        mPosition(0.), mOrientation({1., 0., 0., 0.}), mColor(1.)
        , mIsHovered(false), mIsSelected(false)
{

    glGenVertexArrays(1, &mVao);
    glGenBuffers(1, &mVbo);
    glBindVertexArray(mVao);
    glBindBuffer(GL_ARRAY_BUFFER, mVbo);

    SetVaoAttrib();

    glBindBuffer(GL_ARRAY_BUFFER, mVbo);
    glBufferData(GL_ARRAY_BUFFER, vertices.size()*sizeof(TVertex), vertices.data(), GL_STATIC_DRAW);

    glGenBuffers(1, &mEbo);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mEbo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size()*sizeof(uint32_t), indices.data(), GL_STATIC_DRAW);

    SelectShaderProgram();

    indicesCount = indices.size();
}

template <class TVertex>
void Mesh<TVertex>::ChangeMeshVertexData(std::vector<TVertex> vertices) {
    glBindVertexArray(mVao);
    glBindBuffer(GL_ARRAY_BUFFER, mVbo);
    glBufferData(GL_ARRAY_BUFFER, vertices.size()*sizeof(TVertex), vertices.data(), GL_STATIC_DRAW);

    void* bufferData = glMapBuffer(GL_ARRAY_BUFFER, GL_WRITE_ONLY);

    if (bufferData != nullptr) {
        std::memcpy(bufferData, vertices.data(), vertices.size()*sizeof(TVertex));
        glUnmapBuffer(GL_ARRAY_BUFFER);
    }

}


template <class TVertex>
void Mesh<TVertex>::SelectShaderProgram() {
    if constexpr (std::is_same<TVertex, SimpleVertex>::value) {
        mProgram = {"default.vsh", "default.fsh"};
    } else if constexpr (std::is_same<TVertex, SimpleColorVertex>::value) {
        mProgram = {"defaultVertexColor.vsh", "defaultVertexColor.fsh"};
    } else {
        // Do nothing
    }
}

template <class TVertex>
void Mesh<TVertex>::SetVaoAttrib() {
    if constexpr (std::is_same<TVertex, SimpleVertex>::value) {
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(SimpleVertex), (void *)0);
    } else if constexpr (std::is_same<TVertex, SimpleColorVertex>::value) {
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(SimpleColorVertex), (void *)0);
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(SimpleColorVertex), (void *)sizeof(SimpleColorVertex::position));
    } else {
        // Do nothing
    }
}


template <class TVertex>
void Mesh<TVertex>::Draw(const PerspectiveCamera &camera) {

    InteractiveObject::Draw(camera);

    glPolygonMode(GL_FRONT_AND_BACK, mDrawMode);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);


    mProgram.use();
    mProgram.setMat4("perspective", camera.getProjMatrix());
    mProgram.setMat4("view", camera.getViewMatrix());
    mProgram.setMat4("model", glm::translate(glm::scale(glm::mat4(1.), mScale), mPosition)*glm::mat4_cast(mOrientation));

    mProgram.setVec3("cameraPosition", camera.getPosition()); // Could be retrieved from view matrix
    mProgram.setVec3("backgroundColor", {.08, .05, 0.05}); // Could be retrieved from view matrix

    mProgram.setBool("bOverrideColor", mIsHovered || mIsSelected);
    if (mIsSelected) {
        mProgram.setVec3("overrideColor",mSelectedColor);
    } else if (mIsHovered) {
        mProgram.setVec3("overrideColor",mHoverColor);
    }
    mProgram.setVec3("objectColor",mColor);

    glBindVertexArray(mVao);
    glDrawElements(mPrimitiveMode, indicesCount, GL_UNSIGNED_INT, nullptr);

}

template <class TVertex>
void Mesh<TVertex>::SetPosition(glm::vec3 x) {
    mPosition = x;
}

template <class TVertex>
void Mesh<TVertex>::SetRotation(glm::vec3 x) {
    mOrientation = glm::quat(x);
}

template <class TVertex>
void Mesh<TVertex>::SetScale(glm::vec3 s) {
    mScale = s;
}

template <class TVertex>
void Mesh<TVertex>::Rotate(glm::vec3 x) {
    mOrientation = mOrientation * glm::quat(x);
}

template <class TVertex>
void Mesh<TVertex>::Translate(glm::vec3 x) {
    mPosition += x;
}

template <class TVertex>
void Mesh<TVertex>::SetColor(glm::vec3 color) {
    mColor = color;
}

std::vector<uint32_t> Tube::ConstructIndices(uint32_t resolution) {
    std::vector<uint32_t> indices;
    for (int i = 0; i < resolution; i++) {
        indices.emplace_back(2*i);
        indices.emplace_back((2*i+2)%(2*resolution));
        indices.emplace_back(2*i+1);

        indices.emplace_back((2*i+2)%(2*resolution));
        indices.emplace_back(2*i+1);
        indices.emplace_back((2*i+3)%(2*resolution));
    }
    return indices;
}

std::vector<SimpleVertex> Tube::ConstructVertices(double radius, double length, uint32_t resolution) {
    std::vector<SimpleVertex> vertices;
    for (int i = 0.; i < resolution; i++) {
        double th = glm::radians(360.*static_cast<double>(i)/resolution);
        vertices.emplace_back(radius*glm::cos(th), length, radius*glm::sin(th));
        vertices.emplace_back(radius*glm::cos(th), .0, radius*glm::sin(th));
    }
    return vertices;
}

Tube::Tube(double radius, double length, uint32_t resolution)
        : Mesh(ConstructVertices(radius, length, resolution), ConstructIndices(resolution), true), mRadius(radius), mLength(length), mResolution(resolution)
{

}


std::vector<uint32_t> Sphere::ConstructIndices(uint32_t resolution) {
    std::vector<uint32_t> indices;

    for (int i = 1.; i < resolution; i++) {
        for (int j = 0.; j < resolution; j++) {
            indices.emplace_back(i*resolution+j);
            indices.emplace_back((i-1)*resolution+(j+1)%resolution);
            indices.emplace_back((i-1)*resolution+j);

            indices.emplace_back(i*resolution+j);
            indices.emplace_back((i-1)*resolution+(j+1)%resolution);
            indices.emplace_back(i*resolution+(j+1)%resolution);
        }
    }
    return indices;
}

std::vector<SimpleVertex> Sphere::ConstructVertices(double radius, uint32_t resolution) {
    std::vector<SimpleVertex> vertices;
    for (int i = 0; i < resolution; i++) {
        double pitch = glm::radians(180.*static_cast<double>(i)/(resolution-1));
        for (int j = 0.; j < resolution; j++) {
            double yaw = glm::radians(360.*static_cast<double>(j)/resolution);
            vertices.emplace_back(
                    radius * glm::sin(yaw) * glm::sin(pitch),
                    radius * glm::cos(pitch),
                    radius * glm::cos(yaw) * glm::sin(pitch)
            );
        }
    }
    return vertices;
}

Sphere::Sphere(double radius, uint32_t resolution, bool interactive)
        : Mesh(ConstructVertices(radius, resolution), ConstructIndices(resolution), interactive), mRadius(radius), mResolution(resolution)
{}

Sphere::Sphere(double radius, uint32_t resolution)
        : Sphere(radius, resolution, true)
{}

Grid::Grid(uint16_t resolution, double scale) {
    double halfSize = scale*resolution*.5;
    std::vector<SimpleVertex>   gridVertices;
    std::vector<uint32_t>       gridIndices;
    int index = 0;
    for (int32_t i = -static_cast<int32_t >(resolution)/2; i < resolution/2; ++i) {
        if (i == 0) continue;
        gridVertices.emplace_back(-halfSize, 0., i*scale);
        gridVertices.emplace_back(+halfSize, 0., i*scale);
        gridIndices.push_back(index++);
        gridIndices.push_back(index++);
    }
    for (int32_t i = -static_cast<int32_t >(resolution)/2; i < resolution/2; ++i) {
        if (i == 0) continue;
        gridVertices.emplace_back(i*scale, 0., -halfSize);
        gridVertices.emplace_back(i*scale, 0., +halfSize);
        gridIndices.push_back(index++);
        gridIndices.push_back(index++);
    }
    mScaleGrid = {gridVertices, gridIndices, false};
    mScaleGrid.SetPrimitiveMode(GL_LINES);
    mScaleGrid.SetColor(glm::vec3(.7, .7, 0.7));

    std::vector<SimpleColorVertex>  referentialVertices;
    std::vector<uint32_t>           referentialIndices;

    referentialVertices.emplace_back(SimpleColorVertex({halfSize, 0., 0.}, {1., 0.3, .4}));
    referentialVertices.emplace_back(SimpleColorVertex({-halfSize, 0., 0.}, {1., 0.3, .4}));
    referentialVertices.emplace_back(SimpleColorVertex({0., halfSize, 0.}, {0.4, 1., 0.3}));
    referentialVertices.emplace_back(SimpleColorVertex({0., -halfSize, 0.}, {0.4, 1., 0.3}));
    referentialVertices.emplace_back(SimpleColorVertex({0., 0., halfSize}, {0.3, 0.4, 1.}));
    referentialVertices.emplace_back(SimpleColorVertex({0., 0., -halfSize}, {0.3, 0.4, 1.}));

    referentialIndices.push_back(0);
    referentialIndices.push_back(1);
    referentialIndices.push_back(2);
    referentialIndices.push_back(3);
    referentialIndices.push_back(4);
    referentialIndices.push_back(5);

    mReferentialLines = {referentialVertices, referentialIndices, false};
    mReferentialLines.SetPrimitiveMode(GL_LINES);
}

void Grid::Draw(const PerspectiveCamera &camera) {
    mScaleGrid.Draw(camera);
    mReferentialLines.Draw(camera);
}

//TODO : Refactor update != get
bool InteractiveObject::GetHovered(int32_t x, int32_t y, InteractiveObject* &hoveredObject) {
    uint8_t hoverId;
    if (x < 0 || x >= WINDOW_WIDTH || y < 0 || y >= WINDOW_HEIGHT) {
        hoverId = 0;
    } else {
        glReadPixels(x, y, 1, 1, GL_STENCIL_INDEX, GL_UNSIGNED_BYTE, &hoverId);
    }
    if (currentlyHoveredObject && currentlyHoveredObject->mId != hoverId) {
        currentlyHoveredObject->OnHoverQuit();
    }
    if (hoverId == 0) { return false; }
    hoveredObject = InteractiveObjects[hoverId-1];
    currentlyHoveredObject = hoveredObject;

    return true;
}

void InteractiveObject::Draw(const PerspectiveCamera &camera) {
    glEnable(GL_STENCIL_TEST);
    glStencilMask(0xFF);
    glStencilFunc(GL_ALWAYS, mId, 0xFF);
    glStencilOp(GL_REPLACE, GL_KEEP, GL_REPLACE);
}

WireframeBox::WireframeBox(glm::vec3 center, glm::vec3 sides, glm::vec3 color) {
    std::vector<SimpleVertex> vertices = {
            center - sides,                                   // 0

            center + sides*glm::vec3(-1, -1, 1),    // 1
            center + sides*glm::vec3(-1, 1, -1),    // 2
            center + sides*glm::vec3(1, -1, -1),    // 3

            center + sides*glm::vec3(-1, 1, 1),     // 4
            center + sides*glm::vec3(1, -1, 1),     // 5
            center + sides*glm::vec3(1, 1, -1),     // 6

            center + sides,                                   // 7
    };
    std::vector<uint32_t> indices = {
            0, 1, 0, 2, 0, 3,
            1, 4, 1, 5, 2, 4, 2, 6, 3, 5, 3, 6,
            4, 7, 5, 7, 6, 7
    };
    mMesh = {vertices, indices, false};
    mMesh.SetPrimitiveMode(GL_LINES);
    mMesh.SetColor(color);
}

void WireframeBox::Draw(const PerspectiveCamera &camera) {
    mMesh.Draw(camera);
}

Arrow3D::Arrow3D(glm::vec3 base, glm::vec3 direction, glm::vec3 color)
    : Mesh<SimpleVertex>(ConstructVertices(base, direction), ConstructIndices(), true), mColor(color),
    mDirection(direction), mOrigin(base)
{
    SetColor(color);
}

std::vector<SimpleVertex> Arrow3D::ConstructVertices(glm::vec3 base, glm::vec3 direction) {
    std::vector<SimpleVertex> vertices = { base };

    float scale = glm::length(direction);
    glm::vec3 normal = glm::vec3(-direction.y, direction.x, direction.z);
    if (glm::length(direction-normal) < scale*.5) {
        normal = glm::vec3(-direction.z, direction.y, direction.x);
    }
    glm::vec3 binormal = glm::cross(direction, normal)/scale;

    int res = 10;
    for (int i = 0; i < res; ++i) {
        float th = 2.f*3.1416f*static_cast<float>(i)/res;
        vertices.emplace_back(base + (normal * glm::cos(th) + binormal * glm::sin(th)) * scale*.05f);
        vertices.emplace_back(base + (normal * glm::cos(th) + binormal * glm::sin(th)) * scale*.05f + direction * 0.75f);
        vertices.emplace_back(base + (normal * glm::cos(th) + binormal * glm::sin(th)) * scale*.15f + direction * 0.75f);
    }

    vertices.emplace_back(base+direction);
    return vertices;
}

std::vector<uint32_t> Arrow3D::ConstructIndices() {
    std::vector<uint32_t> indices;

    int res = 10;
    for (int i = 0; i < res; ++i) {
        indices.push_back(0);
        indices.push_back(1 + 3*i);
        indices.push_back(1 + 3*((i+1)%res));


        indices.push_back(1 + 3*i);
        indices.push_back(1 + 3*i + 1);
        indices.push_back(1 + 3*((i+1)%res));
        indices.push_back(1 + 3*((i+1)%res) + 1);
        indices.push_back(1 + 3*i + 1);
        indices.push_back(1 + 3*((i+1)%res));

        indices.push_back(1 + 3*i + 1);
        indices.push_back(1 + 3*i + 1 + 1);
        indices.push_back(1 + 3*((i+1)%res) + 1);
        indices.push_back(1 + 3*((i+1)%res) + 1 + 1);
        indices.push_back(1 + 3*i + 1 + 1);
        indices.push_back(1 + 3*((i+1)%res) + 1);

        indices.push_back(1+3*res);
        indices.push_back(1 + 3*i + 2);
        indices.push_back(1 + 3*((i+1)%res) + 2);

    }
    indices.push_back(0);
    indices.push_back(1+3*res);
    return indices;
}


