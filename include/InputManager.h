//
// Created by brice on 11/18/23.
//

#ifndef VISUAL_INPUTMANAGER_H
#define VISUAL_INPUTMANAGER_H

#include "Scene.h"

enum InputManagerState {
    HoverDetection,
    ObjectSelection,
    CameraMove,
    ActionMode,
};

class InputManager {
public:

    InputManager(GLFWwindow* window, Scene &world);

    void ProcessInputs();
    const PerspectiveCamera &GetCamera();

private:

    void ProcessKeys();
    void ProcessMouse();


    void MouseMoveCallback(double x, double y);

    GLFWwindow* mWindow;
    InputManagerState mState;
    Scene &mWorld;

    bool mMouseFirstLeftPress;

    // Action attributes
    InteractiveObject *mActionObjects;

    // Select attributes
    InteractiveObject *mHoveredObject;
    InteractiveObject *mSelectionCandidate;
    InteractiveObject *mSelectedObject;

    // Camera Move
    PerspectiveCamera mCamera;

    double mCursorLastX;
    double mCursorLastY;

    double mCameraVelocity;
    double mCameraAngularSpeed;

};


#endif //VISUAL_INPUTMANAGER_H
