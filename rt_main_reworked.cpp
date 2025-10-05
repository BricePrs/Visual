#include <iostream>
#include <ProjectIncludes.h>
#include <vector>
#include <sstream>
#include <string>
#include <thread>
#include <chrono>
#include "DoublePendulum.h"
#include "Mesh.h"
#include "Scene.h"
#include "InputManager.h"
#include "Grid.h"
#include "RayTracingCamera.h"
#include "imgui/imgui.h"
#include "imgui/imgui_impl_glfw.h"
#include "imgui/imgui_impl_opengl3.h"
#include <GlobalVar.h>
#include <ParticleSystem.h>
#include <joint.h>
#include <glm/gtc/type_ptr.hpp>

#include "Application.h"


int main() {

    Application app = Application();
    Engine &engine = app.GetEngine();
    Scene &scene = engine.GetScene();

    scene.AddObject<Arrow3D>(*engine._defaultColorShader, glm::vec3(0., 2., 4.), glm::vec3(1., 0., 0.)*0.6f, glm::vec3(1., 0., 0.));
    scene.AddObject<Arrow3D>(*engine._defaultColorShader, glm::vec3(0., 2., 4.), glm::vec3(0., 1., 0.)*0.6f, glm::vec3(0., 1., 0.));
    scene.AddObject<Arrow3D>(*engine._defaultColorShader, glm::vec3(0., 2., 4.), glm::vec3(0., 0., 1.)*0.6f, glm::vec3(0., 0., 1.));
    scene.AddObject<GraphGrid>(*engine._defaultColorShader, 100, 1);

    // Render quad
    auto quad = scene.AddObject<Quad>(*engine._defaultColorShader);
    quad->SetTexture(scene._camera->GetTexture());
    quad->Translate({0., 0., 1.});
    quad->SetScale(glm::vec3(20.f));

    app.Run();

    return EXIT_SUCCESS;
}
