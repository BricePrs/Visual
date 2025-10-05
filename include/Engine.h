//
// Created by brice on 10/4/25.
//

#ifndef VISUAL_ENGINE_H
#define VISUAL_ENGINE_H

#include "InputManager.h"
#include "Scene.h"


class Engine {
public:

    // ------------------------ Constructors and Destructors ------------------------ //

    Engine(GLFWwindow *window);
    ~Engine() = default;

    // -------------------------------- Main methods -------------------------------- //

    void Update(double deltaTime);
    void Render();
    void RenderImGui();
    void ProcessInputs(double deltaTime);

    // ---------------------------- Getters and Setters ---------------------------- //

    Scene &GetScene();


    // TEMP
    std::unique_ptr<ShaderId> _defaultShader;
    std::unique_ptr<ShaderId> _defaultColorShader;
    std::unique_ptr<ShaderId> _defaultTextureShader;
    std::unique_ptr<ShaderId> _defaultNormalShader;

private:

    std::unique_ptr<Scene>           _scene;
    std::unique_ptr<InputManager>    _inputManager;




};


#endif //VISUAL_ENGINE_H