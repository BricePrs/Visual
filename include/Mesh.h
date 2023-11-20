//
// Created by brice on 11/12/23.
//

#ifndef VISUAL_MESH_H
#define VISUAL_MESH_H

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <vector>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <Shader.h>
#include <PerspectiveCamera.h>
#include <iostream>
#include <unordered_set>
#include <glm/gtc/matrix_inverse.hpp>


struct SimpleVertex {
    glm::vec3 position;

    SimpleVertex(double x, double y, double z) : position(glm::vec3(x, y, z)) {}
    SimpleVertex(glm::vec3 pos) : position(pos) {}
};

struct SimpleColorVertex{
    glm::vec3 position;
    glm::vec3 color;

    SimpleColorVertex(glm::vec3 pos, glm::vec3 col) : position(pos), color(col) {}
};

class Drawable {
public:
    virtual void Draw(const PerspectiveCamera &camera) = 0;
};

class InteractiveObject : public Drawable {
public:

    InteractiveObject(const InteractiveObject &object) : mInteractive(object.mInteractive) {
        if (mInteractive) {
            InteractiveObject::InteractiveObjects.push_back(this); // TODO : Lifetime ??
            mId = InteractiveObjects.size();
            std::cout << "Copying " << static_cast<int32_t>(mId) << std::endl;
        }
    };


    explicit InteractiveObject(bool interactive) : mId(0), mInteractive(interactive) {
        if (interactive) {
            InteractiveObject::InteractiveObjects.push_back(this); // TODO : Lifetime ??
            mId = InteractiveObjects.size();
            std::cout << "Creating " << static_cast<int32_t>(mId) << std::endl;
        }
    };


    static bool GetHovered(int32_t x, int32_t y, InteractiveObject* &hoveredObject);
    void Draw(const PerspectiveCamera &camera) override;

    virtual void OnHover() {};
    virtual void OnHoverQuit() {};
    virtual void OnClick() {};
    virtual void OnActionStart(int32_t x, int32_t y) {};
    virtual void OnActionMove(const PerspectiveCamera &camera, double x, double y, double dx, double dy) {};

    virtual bool HasAction() { return false; };

protected:

private:
    uint8_t mId;
    bool mInteractive;

    static inline std::vector<InteractiveObject*> InteractiveObjects;
    static inline InteractiveObject* currentlyHoveredObject;
};



template <class TVertex>
class Mesh : public InteractiveObject {
public:

    Mesh(std::vector<TVertex> vertices, std::vector<uint32_t> indices, bool interactive);

    Mesh(std::vector<TVertex> vertices, std::vector<uint32_t> indices) : Mesh(vertices, indices, false) {}
    Mesh() : Mesh(std::vector<TVertex>(), std::vector<uint32_t>(), false) {}

    void Draw(const PerspectiveCamera &camera) override;

    void Translate(glm::vec3 x);
    void Rotate(glm::vec3 x);

    void SetPosition(glm::vec3 x);
    void SetRotation(glm::vec3 x);
    void SetColor(glm::vec3 color);
    void SetPrimitiveMode(GLenum mode);
    void SetDrawMode(GLenum mode);

    void OnHover() override;
    void OnHoverQuit() override;
    void OnClick() override;

private:

    void SetVaoAttrib();
    void SelectShaderProgram();

    glm::vec3 mPosition;
    glm::vec3 mColor;
    glm::vec3 mHoverColor = glm::vec3(1., 0.1, 0.05);
    glm::vec3 mSelectedColor = glm::vec3(0.3, 0.4, 1.0);
    bool mIsHovered;
    bool mIsSelected;
    glm::quat mOrientation;

    uint32_t indicesCount;

    GLuint mVao;
    GLuint mVbo;
    GLuint mEbo;

    Shader mProgram;
    GLenum mPrimitiveMode = GL_TRIANGLES;
    GLenum mDrawMode = GL_FILL;
};


template class Mesh<SimpleVertex>;
template class Mesh<SimpleColorVertex>;


class Tube : public Mesh<SimpleVertex> {
public:

    Tube(double radius, double length, uint32_t resolution);

private:

    static std::vector<SimpleVertex> ConstructVertices(double radius, double length, uint32_t resolution);
    static std::vector<uint32_t> ConstructIndices(uint32_t mResolution);

    double mRadius;
    double mLength;
    uint32_t mResolution;
};

class Sphere : public Mesh<SimpleVertex> {
public:

    Sphere(double radius, uint32_t resolution, bool interactive);
    Sphere(double radius, uint32_t resolution);

private:

    static std::vector<SimpleVertex> ConstructVertices(double radius, uint32_t resolution);
    static std::vector<uint32_t> ConstructIndices(uint32_t mResolution);

    double mRadius;
    uint32_t mResolution;
};

class Grid : public Drawable {
public:

    Grid(uint16_t resolution, double scale);

    void Draw(const PerspectiveCamera &camera);

private:
    Mesh<SimpleColorVertex> mReferentialLines;
    Mesh<SimpleVertex> mScaleGrid;
};

class WireframeBox : public Drawable {
public:

    WireframeBox(glm::vec3 center, glm::vec3 sides, glm::vec3 color);

    void Draw(const PerspectiveCamera &camera) override;

private:

    Mesh<SimpleVertex> mMesh;

};

class Arrow3D : public Mesh<SimpleVertex> {
public:

    Arrow3D(glm::vec3 base, glm::vec3 direction, glm::vec3 color);

    void OnHover() override { SetColor(mColor + glm::vec3(.5)); }
    void OnHoverQuit() override { SetColor(mColor); }
    void OnClick() override {}

protected:

    static std::vector<SimpleVertex> ConstructVertices(glm::vec3 base, glm::vec3 direction);
    static std::vector<uint32_t> ConstructIndices();

    glm::vec3 mColor;
    glm::vec3 mOrigin;
    glm::vec3 mDirection;

};

class IArrow3D : public Arrow3D {
public:

    using Arrow3D::Arrow3D;

    bool HasAction() override { return true; };
    void OnActionStart(int32_t x, int32_t y) override {};
    void OnActionMove(const PerspectiveCamera &camera, double x, double y, double dx, double dy) override {
        std::cout << "x = " << x << " y = " << y << " dx = " << dx << " dy = " << dy << std::endl;
        glm::mat4 iProj = glm::inverse(camera.getProjMatrix());
        glm::mat4 iView = glm::inverse(camera.getViewMatrix());
        double near = camera.getNear(), far = camera.getFar();
        double dt = AxisPosition(iProj, iView, camera.getPosition(), near, far, x+dx, y+dy)-AxisPosition(iProj, iView, camera.getPosition(), near, far, x, y);
        std::cout << dt << std::endl;
        Translate(mDirection*(float)dt);
    };

private:

    double AxisPosition(glm::mat4 iPersp, glm::mat4 iView, glm::vec3 cameraPosition, double near, double far, double x, double y) {
        glm::vec4 Ap = glm::vec4(x, y, -1., 1.) * (float)near;
        glm::vec4 Bp = glm::vec4(x, y, -1., 1.) * (float)far;
        glm::vec4 Ai = iView * glm::vec4(glm::vec3(iPersp * Ap), 1.);
        glm::vec4 Bi = iView * glm::vec4(glm::vec3(iPersp * Bp), 1.);
        glm::vec3 A = glm::vec3(Ai);
        glm::vec3 B = glm::vec3(Bi);

        glm::vec3 r1 = glm::normalize(B-A);
        glm::vec3 r2 = glm::normalize(mDirection);
        glm::vec3 oo = cameraPosition-mOrigin;
        float t2 = (glm::dot(oo, r2)-glm::dot(oo, r1)*glm::dot(r1, r2))/(1.-glm::dot(r1, r2)*glm::dot(r1, r2));
        return t2;
    }

};



#endif //VISUAL_MESH_H
