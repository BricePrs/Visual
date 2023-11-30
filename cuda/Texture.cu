#include <Texture.h>
#include <iostream>
#include <cuda_runtime.h>

#define CUDA_CHECK_ERROR(err) \
    {                         \
        bool fail = false; \
        do { \
            cudaError_t cudaErr = err; \
            if (cudaErr != cudaSuccess) { \
                fail = true;      \
                fprintf(stderr, "CUDA error in %s at line %d: %s (%d)\n", \
                        __FILE__, __LINE__, cudaGetErrorString(cudaErr), cudaErr); \
            } \
        } while (0);               \
        if (fail) { \
            exit(EXIT_FAILURE);   \
        }                         \
    }\


void Texture::CudaRegister() {
    if (!mCudaGraphicsResource) {
        CUDA_CHECK_ERROR(cudaGraphicsGLRegisterImage(&mCudaGraphicsResource, mId, GL_TEXTURE_2D, cudaGraphicsMapFlagsWriteDiscard));
    } else {
        std::cout << "Texture is already registered\n";
    }
}

void *Texture::CudaMap() {
    if (mCudaGraphicsResource) {
        void *ptr;
        size_t bufferSize;
        CUDA_CHECK_ERROR(cudaGraphicsMapResources(1, &mCudaGraphicsResource, 0));
        CUDA_CHECK_ERROR(cudaGraphicsResourceGetMappedPointer(&ptr, &bufferSize, mCudaGraphicsResource));
        std::cout << bufferSize << " is the accessible memory size\n";
        return ptr;
    }
    throw std::runtime_error("Texture is not registered");
}

void Texture::CudaUnMap() {
    if (mCudaGraphicsResource) {
        CUDA_CHECK_ERROR(cudaGraphicsUnmapResources(1, &mCudaGraphicsResource, 0));
    } else {
        std::cout << "Texture is not registered\n";
    }
}
