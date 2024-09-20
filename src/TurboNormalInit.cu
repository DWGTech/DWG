#include "TurboNormalInit.h"

__global__
void GaussMapInitNormals(float3* __restrict__ point_pos,
	float3* __restrict__ point_normal,
	float3* __restrict__ bound_center,
	int point_num)
{
	int tid = blockIdx.x * blockDim.x + threadIdx.x;
	if (tid < point_num)
	{
		float3 normal;
		normal.x = point_pos[tid].x - bound_center[0].x;
		normal.y = point_pos[tid].y - bound_center[0].y;
		normal.z = point_pos[tid].z - bound_center[0].z;

		float length = __fsqrt_rn(normal.x * normal.x +
			normal.y * normal.y +
			normal.z * normal.z);

		float normal_scale = __frcp_rn(length + 1e-8);

		point_normal[tid] = make_float3(normal.x * normal_scale,
			normal.y * normal_scale,
			normal.z * normal_scale);
	}
}

void TurboNormalInitialization::GaussMapInit(float3* point_pos, float3* point_normal, float3* bound_upper, float3* bound_lower, int point_num)
{
	float3* bound_center = 0;
	cudaMallocManaged((char**)&bound_center, sizeof(*bound_center));

	bound_center[0].x = (bound_lower[0].x + bound_upper[0].x) / 2.0f;
	bound_center[0].y = (bound_lower[0].y + bound_upper[0].y) / 2.0f;
	bound_center[0].z = (bound_lower[0].z + bound_upper[0].z) / 2.0f;

	int threadsPerBlock = 128;
	int blocksPerGrid = (point_num + threadsPerBlock - 1) / threadsPerBlock;
	GaussMapInitNormals << < blocksPerGrid, threadsPerBlock >> > (point_pos, point_normal, bound_center, point_num);
	cudaDeviceSynchronize();

	cudaFree(bound_center); bound_center = 0;
}
