//
// Created by brice on 11/12/23.
//

#include <iostream>
#include "Camera.h"
#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtc/matrix_inverse.hpp"

glm::mat4 Camera::getViewMatrix() const {
    return glm::lookAt(mPosition, mPosition + mFront, glm::vec3(0., 1., 0.));
}

glm::mat4 Camera::getProjMatrix() const {
    return glm::perspective(mYFov, mAspect, mNear, mFar);
}

void Camera::rotate(double dPitch, double dYaw) {
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

void Camera::translate(glm::vec3 t) {
    this->mPosition += t.z * mFront + t.x * mRight + t.y * mUp;
}

glm::vec3 Camera::getPosition() const {
    return mPosition;
}

glm::vec3 Camera::getUp() const {
    return mUp;
}

glm::vec3 Camera::getRight() const {
    return mRight;
}

glm::vec3 Camera::getForward() const {
    return mFront;
}

double Camera::getNear() const {
    return mNear;
}
double Camera::getFar() const {
    return mFar;
}
