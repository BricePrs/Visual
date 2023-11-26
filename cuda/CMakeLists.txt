cmake_minimum_required(VERSION 3.26)

if(NOT DEFINED ${CMAKE_CUDA_ARCHITECTURES})
    set(CMAKE_CUDA_ARCHITECTURES 52 61 75)
endif()
message(STATUS "CUDA architectures set to ${CMAKE_CUDA_ARCHITECTURES}")

set(CMAKE_CUDA_COMPILER /usr/local/cuda-12.3/bin/nvcc)
set(CUDA_TOOLKIT_ROOT_DIR /usr/local/cuda-12.3)

project(Cuda CXX CUDA)

set(CMAKE_CUDA_STANDARD 17)
set(CMAKE_CXX_STANDARD 17)

add_executable(Cuda src/main.cpp src/main.cu src/main.cu)

