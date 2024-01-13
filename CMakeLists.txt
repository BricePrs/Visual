cmake_minimum_required(VERSION 3.26)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CUDA_STANDARD 17)

if(NOT DEFINED ${CMAKE_CUDA_ARCHITECTURES})
    set(CMAKE_CUDA_ARCHITECTURES 86)
endif()
message(STATUS "CUDA architectures set to ${CMAKE_CUDA_ARCHITECTURES}")

set(CMAKE_CUDA_COMPILER /usr/local/cuda-12.3/bin/nvcc)
set(CUDA_TOOLKIT_ROOT_DIR /usr/local/cuda-12.3)


project(Visual)

include_directories(Visual vendors)
include_directories(Visual include)
include_directories(Visual cuda)

file(GLOB HEADER_FILES "include/*.h")
file(GLOB SRC_FILES "src/*.cpp")
file(GLOB CUDA_SRC_FILES "cuda/*.cu")
file(GLOB CUDA_HEADER_FILES "cuda/*.cuh")

message("Cuda header files are ${CUDA_HEADER_FILES}")

set(CMAKE_CUDA_SEPARABLE_COMPILATION ON)

# Compile CUDA code
find_package(CUDA REQUIRED)


cuda_add_executable(
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

        ${CUDA_SRC_FILES} ${CUDA_HEADER_FILES}
        ${SRC_FILES} ${HEADER_FILES}

        main.cpp
)

set_target_properties(Visual PROPERTIES CUDA_SEPARABLE_COMPILATION ON)

target_compile_options(Visual PRIVATE -Wextra -Wconversion -pedantic)

target_link_libraries(Visual -lglfw)
target_link_libraries(Visual -lX11)
target_link_libraries(Visual -lGL)
target_link_libraries(Visual -lpthread)
target_link_libraries(Visual -lXrandr)
target_link_libraries(Visual -lXi)
target_link_libraries(Visual -ldl)


cuda_add_executable(
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

        ${CUDA_SRC_FILES} ${CUDA_HEADER_FILES}
        ${SRC_FILES} ${HEADER_FILES}

        rt_main.cpp
)

set_target_properties(RTVisual PROPERTIES CUDA_SEPARABLE_COMPILATION ON)

target_compile_options(RTVisual PRIVATE -Wextra -Wconversion -pedantic)

target_link_libraries(RTVisual -lglfw)
target_link_libraries(RTVisual -lX11)
target_link_libraries(RTVisual -lGL)
target_link_libraries(RTVisual -lpthread)
target_link_libraries(RTVisual -lXrandr)
target_link_libraries(RTVisual -lXi)
target_link_libraries(RTVisual -ldl)


cuda_add_executable(
        SBVisual

        vendors/implot/implot.cpp
        vendors/implot/implot.h
        vendors/implot/implot_internal.h
        vendors/implot/implot_items.cpp

        vendors/gnuplot-iostream.h

        vendors/happly/happly.h

        vendors/stb_image/stb_image.h

        vendors/glad/glad.c
        vendors/imgui_dock/imgui.cpp
        vendors/imgui_dock/imgui_impl_glfw.cpp
        vendors/imgui_dock/imgui_impl_opengl3.cpp
        vendors/imgui_dock/imgui_draw.cpp
        vendors/imgui_dock/imgui_tables.cpp
        vendors/imgui_dock/imgui_widgets.cpp

        ${CUDA_SRC_FILES} ${CUDA_HEADER_FILES}
        ${SRC_FILES} ${HEADER_FILES}

        sb_main.cpp
)



set_target_properties(SBVisual PROPERTIES CUDA_SEPARABLE_COMPILATION ON)

target_compile_options(SBVisual PRIVATE -Wextra -Wconversion -pedantic)
target_link_libraries(SBVisual -lglfw)
target_link_libraries(SBVisual -lX11)
target_link_libraries(SBVisual -lGL)
target_link_libraries(SBVisual -lpthread)
target_link_libraries(SBVisual -lXrandr)
target_link_libraries(SBVisual -lXi)
target_link_libraries(SBVisual -ldl)
target_link_libraries(SBVisual -lboost_iostreams)

