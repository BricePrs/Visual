//
// Created by brice on 11/12/23.
//

#ifndef VISUAL_PERSPECTIVECAMERA_H
#define VISUAL_PERSPECTIVECAMERA_H

#include "glm/glm.hpp"

class PerspectiveCamera {
public:

    PerspectiveCamera(double aspect) : mPitch(0.), mYaw(0.), mYFov(glm::radians(95.)), mPosition(glm::vec3(0.)), mNear(0.01), mFar(100.),
        mAspect(aspect), mFront(glm::vec3(0., 0., 1.)), mUp(glm::vec3(0., 1., 0.)), mRight(glm::vec3(1., 0., 0.)) {}

    PerspectiveCamera(uint32_t width, uint32_t height) : mPitch(0.), mYaw(0.), mYFov(glm::radians(95.)), mPosition(glm::vec3(0.)), mNear(0.01), mFar(100.),
               mAspect(static_cast<double>(width)/height), mFront(glm::vec3(0., 0., 1.)), mUp(glm::vec3(0., 1., 0.)), mRight(glm::vec3(1., 0., 0.)) {}

    [[nodiscard]] glm::mat4 getViewMatrix() const;
    [[nodiscard]] glm::mat4 getProjMatrix() const;
    [[nodiscard]] double getNear() const;
    [[nodiscard]] double getFar() const;

    virtual void translate(glm::vec3 t);
    virtual void rotate(double dPitch, double dYaw);

    glm::vec3 getPosition() const;
    glm::vec3 getUp() const;
    glm::vec3 getRight() const;
    glm::vec3 getForward() const;


private:

    double mPitch;
    double mYaw;
    double mYFov;
    double mNear;
    double mFar;
    double mAspect;
    glm::vec3 mPosition;
    glm::vec3 mFront;
    glm::vec3 mRight;
    glm::vec3 mUp;

    inline const static glm::vec3 GlobalUP = glm::vec3(0., 1., 0.);

};




#endif //VISUAL_PERSPECTIVECAMERA_H
