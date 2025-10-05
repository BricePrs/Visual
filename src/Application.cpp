//
// Created by brice on 10/4/25.
//

#include "Application.h"

#include "ProjectIncludes.h"

#include "GlobalVar.h"
#include "imgui/imgui.h"
#include "imgui/imgui_impl_glfw.h"
#include "imgui/imgui_impl_opengl3.h"

#include <chrono>
#include <thread>

// ------------------------ Constructors and Destructors ------------------------ //

Application::Application() {
    _window = CreateWindow();
    InitImGUI();

    _engine = std::make_unique<Engine>(_window);
}


void Application::InitImGUI() const {
    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls


    // Setup Dear ImGui style
    ImGui::StyleColorsDark();
    ImGui::PushStyleColor(ImGuiCol_TitleBgActive, ImVec4(.5f, 0.1f, 0.05f, 1.f));
    ImGui::PushStyleColor(ImGuiCol_CheckMark, ImVec4(.5f, 0.1f, 0.05f, 1.f));
    ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(.5f, 0.1f, 0.05f, 1.f));
    ImGui::PushStyleColor(ImGuiCol_SliderGrab, ImVec4(.5f, 0.1f, 0.05f, 1.f));
    ImGui::PushStyleColor(ImGuiCol_SliderGrabActive, ImVec4(.9f, 0.7f, 0.05f, 1.f));
    ImGui::PushStyleColor(ImGuiCol_FrameBg, ImVec4(.2f, 0.1f, 0.05f, 1.f));

    ImGui::GetStyle().FrameRounding = 4;
    ImGui::GetStyle().ChildRounding = 4;
    ImGui::GetStyle().GrabRounding = 4;
    ImGui::GetStyle().WindowRounding = 4;

    const char* glsl_version = "#version 430 core";
    // Setup Platform/Renderer backends
    ImGui_ImplGlfw_InitForOpenGL(_window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);
}

GLFWwindow * Application::CreateWindow() {
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    glfwWindowHint(GLFW_SAMPLES, 16);
    glfwWindowHint(GLFW_RED_BITS, 32);
    glfwWindowHint(GLFW_GREEN_BITS, 32);
    glfwWindowHint(GLFW_BLUE_BITS, 32);
    glfwWindowHint(GLFW_ALPHA_BITS, 32);

    GLFWwindow *window = glfwCreateWindow(WINDOW_WIDTH, WINDOW_HEIGHT, "Visual", nullptr, nullptr);
    if (!window) {
        glfwTerminate();
        throw std::runtime_error("Failed to create window !");
    }

    glfwMakeContextCurrent(window);

    if (!gladLoadGLLoader(reinterpret_cast<GLADloadproc>(glfwGetProcAddress))) {
        glfwTerminate();
        throw std::runtime_error("Failed to get proc address !");
    }

    glfwSwapInterval(0);

    return window;
}


// -------------------------------- Main methods -------------------------------- //


void Application::Run() {

    while (!glfwWindowShouldClose(_window)) {
        auto startFrameTime = std::chrono::high_resolution_clock::now();
        auto currentFrameTime = std::chrono::high_resolution_clock::now();
        static auto lastFrameStartTime = std::chrono::high_resolution_clock::now();

        double deltaTime = std::chrono::duration<double>(currentFrameTime-lastFrameStartTime).count();

        _engine->Update(deltaTime);
        _engine->Render();

        // Start the Dear ImGui frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        _engine->RenderImGui();

        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());


        glfwPollEvents();
        _engine->ProcessInputs(deltaTime);
        glfwSwapBuffers(_window);
        std::this_thread::sleep_until(startFrameTime+std::chrono::duration<double, std::ratio<1, 300>>(1)); // Enforcing 300 fps limit
        lastFrameStartTime = currentFrameTime;
    }

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwTerminate();

}

// ------------------------ Getters and Setters ------------------------ //

Engine & Application::GetEngine() {
    return *_engine;
}



