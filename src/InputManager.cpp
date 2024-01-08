//
// Created by brice on 11/18/23.
//

#include "InputManager.h"
#include "GlobalVar.h"
#include "imgui_dock/imgui.h"
#include <chrono>


void InputManager::ProcessKeys(double deltaTime) {
    if (glfwGetKey(mWindow, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
        glfwSetWindowShouldClose(mWindow, GLFW_TRUE);
    }
    if (glfwGetKey(mWindow, GLFW_KEY_W) == GLFW_PRESS) {
        mCamera->translate(glm::vec3(0., 0., mCameraVelocity*deltaTime));
    }
    if (glfwGetKey(mWindow, GLFW_KEY_S) == GLFW_PRESS) {
        mCamera->translate(glm::vec3(0., 0., -mCameraVelocity*deltaTime));
    }
    if (glfwGetKey(mWindow, GLFW_KEY_A) == GLFW_PRESS) {
        mCamera->translate(glm::vec3(-mCameraVelocity*deltaTime, 0., 0.));
    }
    if (glfwGetKey(mWindow, GLFW_KEY_D) == GLFW_PRESS) {
        mCamera->translate(glm::vec3(mCameraVelocity*deltaTime, 0., 0.));
    }
    if (glfwGetKey(mWindow, GLFW_KEY_E) == GLFW_PRESS) {
        mCamera->translate(glm::vec3( 0., mCameraVelocity*deltaTime, 0.));
    }
    if (glfwGetKey(mWindow, GLFW_KEY_Q) == GLFW_PRESS) {
        mCamera->translate(glm::vec3(0., -mCameraVelocity*deltaTime, 0.));
    }

    if (glfwGetKey(mWindow, GLFW_KEY_SPACE) == GLFW_PRESS) {
        double x, y;
        double near = mCamera->getNear();
        double far = mCamera->getFar();
        glfwGetCursorPos(mWindow, &x, &y);
        x/=WINDOW_WIDTH;
        y/=WINDOW_HEIGHT;
        glm::mat4 iPersp = glm::inverse(mCamera->getProjMatrix());
        glm::mat4 iView = glm::inverse(mCamera->getViewMatrix());

        glm::vec4 Ap = glm::vec4(x*2.-1., -y*2.+1., -1., 1.) * (float)(near);
        glm::vec4 Bp = glm::vec4(x*2.-1., -y*2.+1., -1., 1.) * (float)far;
        glm::vec4 Ai = iView * glm::vec4(glm::vec3(iPersp * Ap), 1.);
        glm::vec4 Bi = iView * glm::vec4(glm::vec3(iPersp * Bp), 1.);
        glm::vec3 A = glm::vec3(Ai);
        glm::vec3 B = glm::vec3(Bi);
        static bool IsSphereInit = false;
        static Mesh mesh = Mesh<SimpleVertex>();
        static std::shared_ptr<Sphere> sphere = std::make_shared<Sphere>(Sphere(3., 15));
        sphere->SetPosition(glm::vec3(A+glm::normalize(B-A)*20.f));
        auto sphereShader = mWorld.AddShader("default.vsh", "default.fsh");
        if (!IsSphereInit) {
            mWorld.AddObject(sphere, sphereShader);
            mWorld.AddCollider(sphere);
            IsSphereInit = true;
        }
    }
}

void InputManager::ProcessMouse(double deltaTime) {

    if (glfwGetMouseButton(mWindow, GLFW_MOUSE_BUTTON_1) == GLFW_PRESS) {

        if (!mMouseFirstLeftPress) {
            if (mState == ActionMode) {
                // Do something ?
            }
            return;
        }
        mMouseFirstLeftPress = false;

        if (mState == HoverDetection) {

            if (mImGuiIO.WantCaptureMouse) {
                // Handle mouse input manually
                double mouseX, mouseY;
                glfwGetCursorPos(mWindow, &mouseX, &mouseY);
                mImGuiIO.AddMouseButtonEvent(0, true);
                return;
            }

            // Register Clicked object
            if (mHoveredObject) {
                // TODO : Fix hover detection
                //mState = ObjectSelection;
                //mSelectionCandidate = mHoveredObject;
                //if (mSelectionCandidate->HasAction()) {
                //    mState = ActionMode;
                //    mActionObjects = mSelectionCandidate;
                //    double x, y;
                //    glfwGetCursorPos(mWindow, &x, &y);
                //    mActionObjects->OnActionStart(x, WINDOW_HEIGHT-y+1);
                //}
            } else {
                mState = CameraMove;
                glfwSetInputMode(mWindow, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
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
    static auto lastCallTime = std::chrono::high_resolution_clock::now();
    auto currentTime = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration<double>(currentTime - lastCallTime).count();
    lastCallTime = currentTime;
    ProcessKeys(duration);
    ProcessMouse(duration);
}

InputManager::InputManager(GLFWwindow* window, Scene &world, PerspectiveCamera* camera)
    : mWindow(window), mWorld(world), mState(HoverDetection), mCamera(camera), mCameraVelocity(30.), mCameraAngularSpeed(0.2),
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

    mImGuiIO.MousePos = ImVec2((float) x, (float) y);


    if (mState == HoverDetection || mState == ObjectSelection) {
        // Update hovered object
        InteractiveObject *object;
        //TODO: fix hover detection
        //if (InteractiveObject::GetHovered(static_cast<int32_t>(x), static_cast<int32_t>(WINDOW_HEIGHT - y + 1), object)) {
        //    object->OnHover();
        //    mHoveredObject = object;
        //} else {
        //    mHoveredObject = nullptr;
        //}
    } else if (mState == ActionMode) {
        mActionObjects->OnActionMove(mWorld , *mCamera, (x / WINDOW_WIDTH) * 2. - 1.,
                                     (WINDOW_HEIGHT - y + 1) / WINDOW_HEIGHT * 2. - 1.,
                                     (dx / WINDOW_WIDTH * 2.),
                                     (dy / WINDOW_HEIGHT * 2.));
    } else if (mState == CameraMove) {
        mCamera->rotate(-dy*mCameraAngularSpeed, -dx*mCameraAngularSpeed);
    }
}

const PerspectiveCamera &InputManager::GetCamera() {
    return *mCamera;
}

