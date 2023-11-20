cmake_minimum_required(VERSION 3.26)
project(Visual)

set(CMAKE_CXX_STANDARD 17)


include_directories(Visual vendors)
include_directories(Visual include)

file(GLOB HEADER_FILES "include/*.h")
file(GLOB SRC_FILES "src/*cpp")

add_executable(
        Visual
        vendors/glad/glad.c
        vendors/imgui/imgui.cpp
        vendors/imgui/imgui_impl_glfw.cpp
        vendors/imgui/imgui_impl_opengl3.cpp
        vendors/imgui/imgui_draw.cpp
        vendors/imgui/imgui_tables.cpp
        vendors/imgui/imgui_widgets.cpp

        ${SRC_FILES}

        ${HEADER_FILES}
)

target_include_directories(Visual PUBLIC src/main)

target_link_libraries(Visual -lglfw)
target_link_libraries(Visual -lX11)
target_link_libraries(Visual -lGL)
target_link_libraries(Visual -lpthread)
target_link_libraries(Visual -lXrandr)
target_link_libraries(Visual -lXi)
target_link_libraries(Visual -ldl)
