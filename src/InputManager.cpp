//
// Created by brice on 11/18/23.
//

#include "InputManager.h"
#include "GlobalVar.h"
#include <glm/ext.hpp>
#include "glm/gtx/string_cast.hpp"

void printvec3(glm::vec3 v) {
    std::cout << "x = " << v.x << " y = " << v.y << " z = " << v.z << std::endl;
}

void InputManager::ProcessKeys() {
    if (glfwGetKey(mWindow, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
        glfwSetWindowShouldClose(mWindow, GLFW_TRUE);
    }
    if (glfwGetKey(mWindow, GLFW_KEY_W) == GLFW_PRESS) {
        mCamera.translate(glm::vec3(0., 0., mCameraVelocity));
    }
    if (glfwGetKey(mWindow, GLFW_KEY_S) == GLFW_PRESS) {
        mCamera.translate(glm::vec3(0., 0., -mCameraVelocity));
    }
    if (glfwGetKey(mWindow, GLFW_KEY_A) == GLFW_PRESS) {
        mCamera.translate(glm::vec3(-mCameraVelocity, 0., 0.));
    }
    if (glfwGetKey(mWindow, GLFW_KEY_D) == GLFW_PRESS) {
        mCamera.translate(glm::vec3(mCameraVelocity, 0., 0.));
    }
    if (glfwGetKey(mWindow, GLFW_KEY_E) == GLFW_PRESS) {
        mCamera.translate(glm::vec3( 0., mCameraVelocity, 0.));
    }
    if (glfwGetKey(mWindow, GLFW_KEY_Q) == GLFW_PRESS) {
        mCamera.translate(glm::vec3(0., -mCameraVelocity, 0.));
    }

    if (glfwGetKey(mWindow, GLFW_KEY_SPACE) == GLFW_PRESS) {
        double x, y;
        double near = mCamera.getNear();
        double far = mCamera.getFar();
        glfwGetCursorPos(mWindow, &x, &y);
        x/=WINDOW_WIDTH;
        y/=WINDOW_HEIGHT;
        glm::mat4 iPersp = glm::inverse(mCamera.getProjMatrix());
        glm::mat4 iView = glm::inverse(mCamera.getViewMatrix());
        std::cout << "proj matrix is "<< glm::to_string(mCamera.getProjMatrix())<<std::endl;
        std::cout << "iproj matrix is "<< glm::to_string(iPersp)<<std::endl;

        glm::vec4 Ap = glm::vec4(x*2.-1., -y*2.+1., 1., 1.) * (float)(near+0.001);
        glm::vec4 Bp = glm::vec4(x*2.-1., -y*2.+1., -1., 1.) * (float)far;
        glm::vec4 Ai = iView * glm::vec4(glm::vec3(iPersp * Ap), 1.);
        glm::vec4 Bi = iView * glm::vec4(glm::vec3(iPersp * Bp), 1.);
        glm::vec3 A = glm::vec3(Ai);
        glm::vec3 B = glm::vec3(Bi);
        std::cout << "A\n";
        printvec3(A);
        std::cout << "B\n";
        printvec3(B);
        static Mesh mesh = Mesh<SimpleVertex>();
        mesh = Mesh<SimpleVertex>({glm::vec3(A), glm::vec3(B)}, {0, 1}, false);
        mesh.SetColor({1., 1., 1.});
        mesh.SetPrimitiveMode(GL_LINES);
        mWorld.AddObject(&mesh);
    }
}

void InputManager::ProcessMouse() {
    if (glfwGetMouseButton(mWindow, GLFW_MOUSE_BUTTON_1) == GLFW_PRESS) {
        if (!mMouseFirstLeftPress) {
            if (mState == ActionMode) {
                // Do something ?
            }
            return;
        }
        mMouseFirstLeftPress = false;
        if (mState == HoverDetection) {
            // Register Clicked object
            if (mHoveredObject) {
                mState = ObjectSelection;
                mSelectionCandidate = mHoveredObject;
                if (mSelectionCandidate->HasAction()) {
                    mState = ActionMode;
                    mActionObjects = mSelectionCandidate;
                    double x, y;
                    glfwGetCursorPos(mWindow, &x, &y);
                    mActionObjects->OnActionStart(x, WINDOW_HEIGHT-y+1);
                }
            } else {
                mState = CameraMove;
                glfwSetInputMode(mWindow, GLFW_CURSOR, GLFW_CURSOR_HIDDEN);
            }
            return;
        }
        std::cout << "Input::Should not happen" << std::endl;
        return;
    } else if (glfwGetMouseButton(mWindow, GLFW_MOUSE_BUTTON_1) == GLFW_RELEASE) {
        mMouseFirstLeftPress = true;
        if (mState == CameraMove) {
            glfwSetInputMode(mWindow, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
        }
        if (mState == ObjectSelection) {
            // Select Object Callback
            if (mHoveredObject == mSelectionCandidate) {
                mSelectedObject = mSelectionCandidate;
                mSelectedObject->OnClick();
            }
        }
        mState = HoverDetection;
    }
}

void InputManager::ProcessInputs() {
    ProcessKeys();
    ProcessMouse();
}

InputManager::InputManager(GLFWwindow* window, Scene &world)
    : mWindow(window), mWorld(world), mState(HoverDetection), mCamera(ASPECT), mCameraVelocity(.025), mCameraAngularSpeed(0.2),
      mMouseFirstLeftPress(true)
{
    glfwSetWindowUserPointer(window, reinterpret_cast<void *>(this)); // TODO : Move in application class
    glfwSetCursorPosCallback(window, [](GLFWwindow* window, double x, double y) {
        InputManager *instance = static_cast<InputManager *>(glfwGetWindowUserPointer(window));
        if (instance) {
            instance->MouseMoveCallback(x, y);
        }
    });
}

void InputManager::MouseMoveCallback(double x, double y) {

    double dx = x-mCursorLastX;
    double dy = y-mCursorLastY;
    mCursorLastX = x;
    mCursorLastY = y;

    if (mState == HoverDetection || mState == ObjectSelection) {
        // Update hovered object
        InteractiveObject *object;
        if (InteractiveObject::GetHovered(static_cast<int32_t>(x), static_cast<int32_t>(WINDOW_HEIGHT - y + 1), object)) {
            object->OnHover();
            mHoveredObject = object;
        } else {
            mHoveredObject = nullptr;
        }
    } else if (mState == ActionMode) {
        mActionObjects->OnActionMove(mCamera, (x / WINDOW_WIDTH) * 2. - .5,
                                     (WINDOW_HEIGHT - y + 1) / WINDOW_HEIGHT * 2. - .5,
                                     (dx / WINDOW_WIDTH * 2.),
                                     (dy / WINDOW_HEIGHT * 2.));
    } else if (mState == CameraMove) {
        mCamera.rotate(-dy*mCameraAngularSpeed, -dx*mCameraAngularSpeed);
    }
}

const PerspectiveCamera &InputManager::GetCamera() {
    return mCamera;
}
