#include "ImageDistortion.h"
#include "cuda_runtime.h"
#include "device_launch_parameters.h"

__global__ void VecAdd(float* A, float* B, float* C)
{
    int i = threadIdx.x;
    C[i] = A[i] + B[i];
}
void ImageDistortion()
{
    float A[2] = { 0,1 };
    float B[2] = { 1,2 };
    float C[2] = {3,4 };
    VecAdd << <1, 2 >> > (A, B, C);
}
