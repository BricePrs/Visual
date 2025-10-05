//
// Created by brice on 10/4/25.
//

#include "Engine.h"

#include <chrono>

#include "imgui/imgui_impl_glfw.h"
#include "imgui/imgui_impl_opengl3.h"

Engine::Engine(GLFWwindow *window)
{
    auto camera = Camera(WINDOW_WIDTH, WINDOW_HEIGHT);
    _scene = std::make_unique<Scene>(camera);
    _inputManager = std::make_unique<InputManager>(window, *_scene, &_scene->_camera), // TODO

    _defaultShader          = std::make_unique<ShaderId>(_scene->AddShader("default.vsh", "default.fsh"));
    _defaultColorShader     = std::make_unique<ShaderId>(_scene->AddShader("defaultVertexColor.vsh", "defaultVertexColor.fsh"));
    _defaultTextureShader   = std::make_unique<ShaderId>(_scene->AddShader("default_texture.vsh", "default_texture.fsh"));
    _defaultNormalShader    = std::make_unique<ShaderId>(_scene->AddShader("defaultVertexNormal.vsh", "defaultVertexNormal.fsh"));


    Mesh<SimpleVertex>::MESH_SHADER         = std::make_optional(_scene->GetShader(*_defaultShader));
    Mesh<SimpleColorVertex>::MESH_SHADER    = std::make_optional(_scene->GetShader(*_defaultColorShader));
    Mesh<SimpleNormalVertex>::MESH_SHADER   = std::make_optional(_scene->GetShader(*_defaultNormalShader));
    Mesh<SimpleUvVertex>::MESH_SHADER       = std::make_optional(_scene->GetShader(*_defaultTextureShader));

}

void Engine::Update(double deltaTime) {
    //rtCam->DrawScene(_inputManager.GetCamera());
}

void Engine::Render() {

    glClearColor(.08, .05, 0.05, 1.);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
    glStencilMask(0x00);
    _scene->Draw();
}

void Engine::RenderImGui() {
    _scene->DrawWindows();
}

void Engine::ProcessInputs(double deltaTime) {
    _inputManager->ProcessInputs(deltaTime);
}


// ---------------------------- Getters and Setters ---------------------------- //


Scene& Engine::GetScene() {
    return *_scene;
}
