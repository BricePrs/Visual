//
// Created by brice on 11/18/23.
//

#ifndef VISUAL_INPUTMANAGER_H
#define VISUAL_INPUTMANAGER_H

#include <imgui/imgui.h>
#include "ProjectIncludes.h"
#include "Scene.h"
#include "Mesh.h"


enum InputManagerState {
    HoverDetection,
    ObjectSelection,
    CameraMove,
    ActionMode,
};

class InputManager {
public:

    InputManager(GLFWwindow* window, Scene &world, Camera *camera);

    void ProcessInputs(double deltaTime);

    [[nodiscard]]
    const Camera &GetCamera() const;

private:

    void ProcessKeys(double deltaTime);
    void ProcessMouse(double deltaTime);


    void MouseMoveCallback(double x, double y);

    GLFWwindow* mWindow;
    InputManagerState mState;
    Scene &mWorld;

    bool mMouseFirstLeftPress;

    // Action attributes
    InteractiveObject *mActionObjects;

    // Select attributes
    InteractiveObject *mHoveredObject = nullptr;
    InteractiveObject *mSelectionCandidate;
    InteractiveObject *mSelectedObject;

    // Camera Move
    Camera* mCamera;

    double mCursorLastX;
    double mCursorLastY;

    double mCameraVelocity;
    double mCameraAngularSpeed;

    ImGuiIO& mImGuiIO = ImGui::GetIO();
};


#endif //VISUAL_INPUTMANAGER_H
