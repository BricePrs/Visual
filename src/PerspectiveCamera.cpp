//
// Created by brice on 11/12/23.
//

#include <iostream>
#include "PerspectiveCamera.h"
#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtc/matrix_inverse.hpp"

glm::mat4 PerspectiveCamera::getViewMatrix() const {
    return glm::lookAt(mPosition, mPosition + mFront, glm::vec3(0., 1., 0.));
}

glm::mat4 PerspectiveCamera::getProjMatrix() const {
    return glm::perspective(mYFov, mAspect, mNear, mFar);
}

void PerspectiveCamera::rotate(double dPitch, double dYaw) {
    mPitch += dPitch;
    mYaw += dYaw;
    if (mPitch > 89.9f) {
        mPitch = 89.9f;
    } else if (mPitch < -89.9f) {
        mPitch = -89.9f;
    }
    mFront = {glm::sin(glm::radians(mYaw))*glm::cos(glm::radians(mPitch)),
              glm::sin(glm::radians(mPitch)),
              glm::cos(glm::radians(mYaw))*glm::cos(glm::radians(mPitch))};
    mRight = glm::normalize(glm::cross(mFront, GlobalUP));
    mUp = glm::normalize(glm::cross(mRight, mFront));
}

void PerspectiveCamera::translate(glm::vec3 t) {
    this->mPosition += t.z * mFront + t.x * mRight + t.y * mUp;
}

glm::vec3 PerspectiveCamera::getPosition() const {
    return mPosition;
}

glm::vec3 PerspectiveCamera::getUp() const {
    return mUp;
}

glm::vec3 PerspectiveCamera::getRight() const {
    return mRight;
}

glm::vec3 PerspectiveCamera::getForward() const {
    return mFront;
}

double PerspectiveCamera::getNear() const {
    return mNear;
}
double PerspectiveCamera::getFar() const {
    return mFar;
}
