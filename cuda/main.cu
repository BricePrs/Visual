#include <stdio.h>
#include <cmath>
#include "main.cuh"

#define CHECK_CUDA_ERROR() \
do { \
    cudaError_t error = cudaGetLastError(); \
    if (error != cudaSuccess) { \
        fprintf(stderr, "CUDA error: %s at %s:%d\n", cudaGetErrorString(error), __FILE__, __LINE__); \
        exit(EXIT_FAILURE); \
    } \
} while(0)

__global__
void saxpy(int n, float a, float *x, float *y)
{
    printf("lakz pakz ");
    int i = blockIdx.x*blockDim.x + threadIdx.x;
    if (i < n) y[i] = a*x[i] + y[i];
}

int oui(void)
{
    int N = 1000;
    float *x, *y, *d_x, *d_y;
    x = (float *) malloc(N * sizeof(float));
    y = (float *) malloc(N * sizeof(float));

    cudaMalloc(&d_x, N
                     *sizeof(float));
    cudaMalloc(&d_y, N
                     *sizeof(float));

    for (
            int i = 0;
            i<N;
            i++) {
        x[i] = 1.0f;
        y[i] = 2.0f;
    }

    cudaMemcpy(d_x, x, N
                       *sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(d_y, y, N
                       *sizeof(float), cudaMemcpyHostToDevice);
    CHECK_CUDA_ERROR();

    // Perform SAXPY on 1M elements
    saxpy<<<(N+255)/256, 256>>>(N, 2.0f, d_x, d_y);
    CHECK_CUDA_ERROR();

    cudaMemcpy(y, d_y, N
                       *sizeof(float), cudaMemcpyDeviceToHost);

    float maxError = 0.0f;
    for (
            int i = 0;
            i<N;
            i++)
        maxError = std::max(maxError, abs(y[i] - 4.0f));
    printf("Max error: %f\n", maxError);

    cudaFree(d_x);
    cudaFree(d_y);
    free(x);
    free(y);
    return 0;
}

