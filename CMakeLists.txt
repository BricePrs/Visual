cmake_minimum_required(VERSION 3.22)
project(Visual)

set(CMAKE_CXX_STANDARD 20)



project(Visual)
# Add debug flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0")

include_directories(Visual vendors)
include_directories(Visual include)

file(GLOB HEADER_FILES "include/*.h")
file(GLOB SRC_FILES "src/*.cpp")


add_executable(
        Visual
        vendors/happly/happly.h
        vendors/stb_image/stb_image.h
        vendors/glad/glad.c
        vendors/imgui/imgui.cpp
        vendors/imgui/imgui_impl_glfw.cpp
        vendors/imgui/imgui_impl_opengl3.cpp
        vendors/imgui/imgui_draw.cpp
        vendors/imgui/imgui_tables.cpp
        vendors/imgui/imgui_widgets.cpp

        ${SRC_FILES} ${HEADER_FILES}

        main.cpp
)


target_compile_options(Visual PRIVATE -Wextra -Wconversion -pedantic)

target_link_libraries(Visual -lglfw)
target_link_libraries(Visual -lX11)
target_link_libraries(Visual -lGL)
target_link_libraries(Visual -lpthread)
target_link_libraries(Visual -lXrandr)
target_link_libraries(Visual -lXi)
target_link_libraries(Visual -ldl)


add_executable(
        RTVisual
        vendors/happly/happly.h
        vendors/stb_image/stb_image.h
        vendors/glad/glad.c
        vendors/imgui/imgui.cpp
        vendors/imgui/imgui_impl_glfw.cpp
        vendors/imgui/imgui_impl_opengl3.cpp
        vendors/imgui/imgui_demo.cpp
        vendors/imgui/imgui_draw.cpp
        vendors/imgui/imgui_tables.cpp
        vendors/imgui/imgui_widgets.cpp

        ${SRC_FILES} ${HEADER_FILES}

        rt_main.cpp
)


target_compile_options(RTVisual PRIVATE -Wextra -Wconversion -pedantic)

target_link_libraries(RTVisual -lglfw)
target_link_libraries(RTVisual -lX11)
target_link_libraries(RTVisual -lGL)
target_link_libraries(RTVisual -lpthread)
target_link_libraries(RTVisual -lXrandr)
target_link_libraries(RTVisual -lXi)
target_link_libraries(RTVisual -ldl)


add_executable(
        Rework
        vendors/happly/happly.h
        vendors/stb_image/stb_image.h
        vendors/glad/glad.c
        vendors/imgui/imgui.cpp
        vendors/imgui/imgui_impl_glfw.cpp
        vendors/imgui/imgui_impl_opengl3.cpp
        vendors/imgui/imgui_demo.cpp
        vendors/imgui/imgui_draw.cpp
        vendors/imgui/imgui_tables.cpp
        vendors/imgui/imgui_widgets.cpp

        ${SRC_FILES} ${HEADER_FILES}

        rt_main_reworked.cpp
)


target_compile_options(Rework PRIVATE -Wextra -Wconversion -pedantic)

target_link_libraries(Rework -lglfw)
target_link_libraries(Rework -lX11)
target_link_libraries(Rework -lGL)
target_link_libraries(Rework -lpthread)
target_link_libraries(Rework -lXrandr)
target_link_libraries(Rework -lXi)
target_link_libraries(Rework -ldl)





