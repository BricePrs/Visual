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


int main() {

    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_SAMPLES, 16);

    GLFWwindow *window = glfwCreateWindow(WINDOW_WIDTH, WINDOW_HEIGHT, "Visual", nullptr, nullptr);
    if (!window) {
        glfwTerminate();
        throw std::runtime_error("Failed to create window !");
    }

    glfwMakeContextCurrent(window);

    if (!gladLoadGLLoader((GLADloadproc) glfwGetProcAddress)) {
        glfwTerminate();
        throw std::runtime_error("Failed to get proc address !");
    }

    glfwSwapInterval(0);


    RayTracingCamera* rtCam = new RayTracingCamera(2048, 2048);
    rtCam->translate({0., 0., -4.});
    quad->SetTexture(rtCam->GetTexture());

    quad->Translate({0., 0., 1.});
    quad->SetScale(glm::vec3(20.f));

    Scene world;

    auto defaultShader          = world.AddShader("default.vsh", "default.fsh");
    auto defaultColorShader     = world.AddShader("defaultVertexColor.vsh", "defaultVertexColor.fsh");
    auto defaultTextureShader   = world.AddShader("default_texture.vsh", "default_texture.fsh");
    auto defaultNormalShader    = world.AddShader("defaultVertexNormal.vsh", "defaultVertexNormal.fsh");

    Mesh<SimpleVertex>::MESH_SHADER         = std::make_optional(world.GetShader(defaultShader));
    Mesh<SimpleColorVertex>::MESH_SHADER    = std::make_optional(world.GetShader(defaultColorShader));
    Mesh<SimpleNormalVertex>::MESH_SHADER   = std::make_optional(world.GetShader(defaultNormalShader));
    Mesh<SimpleUvVertex>::MESH_SHADER       = std::make_optional(world.GetShader(defaultTextureShader));


    world.AddObject(arrowx, defaultColorShader);
    world.AddObject(arrowy, defaultColorShader);
    world.AddObject(arrowz, defaultColorShader);
    world.AddObject(quad, defaultColorShader);

    rtCam->SetMesh(nullptr);
    //rtCam->SetMesh("models/teapot_wt.ply");
    auto displayCamera = new Camera(WINDOW_WIDTH, WINDOW_HEIGHT);
    displayCamera->translate({2., 0., -6.5});
    rtCam->SetEnvMap("textures/pisa.png");
    rtCam->SetGroundTex("textures/grace-new.png");
    //rtCam->SetSphereTex("textures/earth1.png");
    //rtCam->SetSphereNormalMap("textures/earth3.png");


    // --- ImGui Set Up --- //

    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls

 
    // Setup Dear ImGui style
    ImGui::StyleColorsDark();
    ImGui::PushStyleColor(ImGuiCol_TitleBgActive, ImVec4(.5, 0.1, 0.05, 1.));
    ImGui::PushStyleColor(ImGuiCol_CheckMark, ImVec4(.5, 0.1, 0.05, 1.));
    ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(.5, 0.1, 0.05, 1.));
    ImGui::PushStyleColor(ImGuiCol_SliderGrab, ImVec4(.5, 0.1, 0.05, 1.));
    ImGui::PushStyleColor(ImGuiCol_SliderGrabActive, ImVec4(.9, 0.7, 0.05, 1.));
    ImGui::PushStyleColor(ImGuiCol_FrameBg, ImVec4(.2, 0.1, 0.05, 1.));

    //for (int i = 0; i < static_cast<int>(ImGuiCol_COUNT)-1; ++i) {
    //    if (i < 11 || i > 11) {
    //        continue;
    //    }
    //    ImGui::PushStyleColor(i, ImVec4(.3, 0.05, 0.07, 1.));
    //}
    ImGui::GetStyle().FrameRounding = 4;
    ImGui::GetStyle().ChildRounding = 4;
    ImGui::GetStyle().GrabRounding = 4;
    ImGui::GetStyle().WindowRounding = 4;


    const char* glsl_version = "#version 430 core";
    // Setup Platform/Renderer backends
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);


    InputManager inputManager(window, world, rtCam);

    // --- Main Loop --- //

    while (!glfwWindowShouldClose(window)) {
        auto startFrameTime = std::chrono::high_resolution_clock::now();
        glClearColor(.08, .05, 0.05, 1.);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
        glStencilMask(0x00);


        static auto StartTime = std::chrono::high_resolution_clock::now();
        auto time = std::chrono::high_resolution_clock::now();
        rtCam->DrawScene(inputManager.GetCamera());

        world.Draw(*displayCamera);


        // Start the Dear ImGui frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        rtCam->DrawWindow();

        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());


        glfwPollEvents();
        inputManager.ProcessInputs();
        glfwSwapBuffers(window);
        std::this_thread::sleep_until(startFrameTime+std::chrono::duration<double, std::ratio<1, 300>>(1));
    }

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwTerminate();

    return EXIT_SUCCESS;
}
