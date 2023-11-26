cmake_minimum_required(VERSION 3.26)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CUDA_STANDARD 17)

if(NOT DEFINED ${CMAKE_CUDA_ARCHITECTURES})
    set(CMAKE_CUDA_ARCHITECTURES 75)
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
cuda_add_library(CudaLibrary STATIC ${CUDA_SRC_FILES} ${CUDA_HEADER_FILES})
set_target_properties(CudaLibrary PROPERTIES CUDA_SEPARABLE_COMPILATION ON)


add_executable(
        Visual
        vendors/glad/glad.c
        vendors/imgui/imgui.cpp
        vendors/imgui/imgui_impl_glfw.cpp
        vendors/imgui/imgui_impl_opengl3.cpp
        vendors/imgui/imgui_draw.cpp
        vendors/imgui/imgui_tables.cpp
        vendors/imgui/imgui_widgets.cpp

        ${HEADER_FILES}
        ${SRC_FILES}
)


target_link_libraries(Visual -lglfw)
target_link_libraries(Visual -lX11)
target_link_libraries(Visual -lGL)
target_link_libraries(Visual -lpthread)
target_link_libraries(Visual -lXrandr)
target_link_libraries(Visual -lXi)
target_link_libraries(Visual -ldl)
target_link_libraries(Visual CudaLibrary)



