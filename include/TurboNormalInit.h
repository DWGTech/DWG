#pragma once
#include <cuda.h>
#include <cuda_runtime.h>

#include <device_launch_parameters.h>

#include <Eigen/Core>

class TurboNormalInitialization
{
public:
	TurboNormalInitialization() = default;
	void GaussMapInit(float3* point_pos, float3* point_normal, float3* bound_upper, float3* bound_lower, int point_num);
private:
};