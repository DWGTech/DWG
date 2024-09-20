# include "TurboDWG.h"

#define gpuErrchk(ans) { gpuAssert((ans), #ans, __FILE__, __LINE__); }
inline void gpuAssert(cudaError_t code, const char* command, const char* file, int line, bool abort = true)
{
	//printf("%s = %i\n", command, (int)code);
	if (code != cudaSuccess)
	{
		fprintf(stderr, "GPUassert: %s %s %d\n", cudaGetErrorString(code), file, line);
		if (abort) exit(code);
	}
}

TurboDWG::TurboDWG(Eigen::MatrixXf& octree_leaves, Eigen::MatrixXf& leaves_Ns, Eigen::VectorXf& leaves_As, Eigen::VectorXf radii, Eigen::MatrixXf& grid_points, int res_x, int res_y, int res_z, std::vector<float> bound_upper_kdtree, std::vector<float> bound_lower_kdtree)
{
	int point_cloud_size = octree_leaves.rows();
	cudaMallocManaged((char**)&this->m_Point_Cloud_GPU, point_cloud_size * sizeof(*this->m_Point_Cloud_GPU));
	std::unique_ptr<float3[]> point_cloud_cpu = std::make_unique<float3[]>(size_t(point_cloud_size));
	for (int i = 0; i < point_cloud_size; i++)
	{
		point_cloud_cpu[i] = make_float3(octree_leaves(i, 0), octree_leaves(i, 1), octree_leaves(i, 2));
	}
	gpuErrchk(cudaMemcpy(this->m_Point_Cloud_GPU, point_cloud_cpu.get(), point_cloud_size * sizeof(float3), cudaMemcpyHostToDevice));
	this->m_Point_Cloud_size = point_cloud_size;

	int leaves_Ns_size = leaves_Ns.rows();
	cudaMallocManaged((char**)&this->m_Normals_GPU, leaves_Ns_size * sizeof(*this->m_Normals_GPU));
	std::unique_ptr<float3[]> leaves_Ns_cpu = std::make_unique<float3[]>(size_t(leaves_Ns_size));
	for (int i = 0; i < leaves_Ns_size; i++)
	{
		leaves_Ns_cpu[i] = make_float3(leaves_Ns(i, 0), leaves_Ns(i, 1), leaves_Ns(i, 2));
	}
	gpuErrchk(cudaMemcpy(this->m_Normals_GPU, leaves_Ns_cpu.get(), leaves_Ns_size * sizeof(float3), cudaMemcpyHostToDevice));

	int As_size = leaves_As.size();
	//gpuErrchk(cudaMalloc(&i_As_gpu, i_As_size * sizeof(float)));
	cudaMallocManaged((char**)&this->m_As_GPU, As_size * sizeof(*this->m_As_GPU));
	std::unique_ptr<float[]> As_cpu = std::make_unique<float[]>(size_t(As_size));
	for (int i = 0; i < As_size; i++)
	{
		As_cpu[i] = leaves_As[i];
	}
	gpuErrchk(cudaMemcpy(this->m_As_GPU, As_cpu.get(), As_size * sizeof(float), cudaMemcpyHostToDevice));

	int radii_size = radii.size();
	//gpuErrchk(cudaMalloc(&i_As_gpu, i_As_size * sizeof(float)));
	cudaMallocManaged((char**)&this->m_Radii_GPU, radii_size * sizeof(*this->m_Radii_GPU));
	std::unique_ptr<float[]> radii_cpu = std::make_unique<float[]>(size_t(radii_size));
	for (int i = 0; i < radii_size; i++)
	{
		radii_cpu[i] = radii[i];
	}
	gpuErrchk(cudaMemcpy(this->m_Radii_GPU, radii_cpu.get(), radii_size * sizeof(float), cudaMemcpyHostToDevice));

	int grid_points_size = grid_points.rows();
	cudaMallocManaged((char**)&this->m_Grid_Points_GPU, grid_points_size * sizeof(*this->m_Grid_Points_GPU));
	std::unique_ptr<float3[]> grid_points_cpu = std::make_unique<float3[]>(size_t(grid_points_size));
	for (int i = 0; i < grid_points_size; i++)
	{
		grid_points_cpu[i] = make_float3(grid_points(i, 0), grid_points(i, 1), grid_points(i, 2));
	}
	gpuErrchk(cudaMemcpy(this->m_Grid_Points_GPU, grid_points_cpu.get(), grid_points_size * sizeof(float3), cudaMemcpyHostToDevice));
	this->m_Grid_Points_size = grid_points_size;

	this->m_Grid_Res_X = res_x;
	this->m_Grid_Res_Y = res_y;
	this->m_Grid_Res_Z = res_z;

	cudaMallocManaged((char**)&this->m_PC_WN_GPU, point_cloud_size * sizeof(*this->m_PC_WN_GPU));
	cudaMallocManaged((char**)&this->m_GP_WN_GPU, grid_points_size * sizeof(*this->m_GP_WN_GPU));

	cudaMallocManaged((char**)&this->m_Bound_Upper_KDT_GPU, sizeof(*this->m_Bound_Upper_KDT_GPU));
	this->m_Bound_Upper_KDT_GPU[0].x = bound_upper_kdtree[0];
	this->m_Bound_Upper_KDT_GPU[0].y = bound_upper_kdtree[1];
	this->m_Bound_Upper_KDT_GPU[0].z = bound_upper_kdtree[2];
	cudaMallocManaged((char**)&this->m_Bound_Lower_KDT_GPU, sizeof(*this->m_Bound_Lower_KDT_GPU));
	this->m_Bound_Lower_KDT_GPU[0].x = bound_lower_kdtree[0];
	this->m_Bound_Lower_KDT_GPU[0].y = bound_lower_kdtree[1];
	this->m_Bound_Lower_KDT_GPU[0].z = bound_lower_kdtree[2];

	std::cout << "Res X: " << this->m_Grid_Res_X << " " << "Res Y: " << this->m_Grid_Res_Y << " " << "Res Z: " << this->m_Grid_Res_Z << std::endl;
	std::cout << "GPU Point Cloud Size: " << this->m_Point_Cloud_size << std::endl;
}

void TurboDWG::PreTurboDWG(int point_cloud_size, Eigen::VectorXf& leaves_As, Eigen::MatrixXf& grid_points, int res_x, int res_y, int res_z)
{
	int As_size = point_cloud_size;
	//gpuErrchk(cudaMalloc(&i_As_gpu, i_As_size * sizeof(float)));
	cudaMallocManaged((char**)&this->m_As_GPU, As_size * sizeof(*this->m_As_GPU));
	std::unique_ptr<float[]> As_cpu = std::make_unique<float[]>(size_t(As_size));
	for (int i = 0; i < As_size; i++)
	{
		As_cpu[i] = leaves_As[i];
	}
	gpuErrchk(cudaMemcpy(this->m_As_GPU, As_cpu.get(), As_size * sizeof(float), cudaMemcpyHostToDevice));

	int radii_size = point_cloud_size;
	//gpuErrchk(cudaMalloc(&i_As_gpu, i_As_size * sizeof(float)));
	cudaMallocManaged((char**)&this->m_Radii_GPU, radii_size * sizeof(*this->m_Radii_GPU));

	int grid_points_size = grid_points.rows();
	cudaMallocManaged((char**)&this->m_Grid_Points_GPU, grid_points_size * sizeof(*this->m_Grid_Points_GPU));
	std::unique_ptr<float3[]> grid_points_cpu = std::make_unique<float3[]>(size_t(grid_points_size));
	for (int i = 0; i < grid_points_size; i++)
	{
		grid_points_cpu[i] = make_float3(grid_points(i, 0), grid_points(i, 1), grid_points(i, 2));
	}
	gpuErrchk(cudaMemcpy(this->m_Grid_Points_GPU, grid_points_cpu.get(), grid_points_size * sizeof(float3), cudaMemcpyHostToDevice));
	this->m_Grid_Points_size = grid_points_size;

	this->m_Grid_Res_X = res_x;
	this->m_Grid_Res_Y = res_y;
	this->m_Grid_Res_Z = res_z;

	cudaMallocManaged((char**)&this->m_PC_WN_GPU, point_cloud_size * sizeof(*this->m_PC_WN_GPU));
	cudaMallocManaged((char**)&this->m_GP_WN_GPU, grid_points_size * sizeof(*this->m_GP_WN_GPU));

	std::cout << "GPU Point Cloud Size: " << point_cloud_size << std::endl;
	std::cout << "Res X: " << this->m_Grid_Res_X << " " << "Res Y: " << this->m_Grid_Res_Y << " " << "Res Z: " << this->m_Grid_Res_Z << std::endl;
}

__global__
void AddWindNumbers(float* __restrict__ winding_numbers,
	float* __restrict__ winding_sum,
	int com_size)
{
	int tid = blockIdx.x * blockDim.x + threadIdx.x;
	if (tid < com_size)
	{
		atomicAdd(winding_sum, winding_numbers[tid]);
	}
}

__global__
void ResetZero(float3* __restrict__ vector,
	int com_size)
{
	int tid = blockIdx.x * blockDim.x + threadIdx.x;
	if (tid < com_size)
	{
		vector[tid] = make_float3(0.0f, 0.0f, 0.0f);
	}
}

__global__
void LoadZeroData(float3* __restrict__ zero_points,
	float3* __restrict__ nonzero_points,
	int* __restrict__ zero_points_index,
	int* __restrict__ nonzero_points_index,
	float3* __restrict__ nonzero_points_normal,
	float* __restrict__ tmp_lengths,
	float3* __restrict__ point_pos,
	float3* __restrict__ point_normal,
	int* c_zero_index,
	int* c_nonzero_index,
	int com_size)
{
	int tid = blockIdx.x * blockDim.x + threadIdx.x;
	if (tid < com_size)
	{
		if (tmp_lengths[tid] < 1e-10)
		{
			int c_ato_index = atomicAdd(c_zero_index, 1);

			zero_points[c_ato_index] = point_pos[tid];
			zero_points_index[c_ato_index] = tid;
		}
		else
		{
			int c_ato_index = atomicAdd(c_nonzero_index, 1);

			nonzero_points[c_ato_index] = point_pos[tid];
			nonzero_points_index[c_ato_index] = tid;
			nonzero_points_normal[c_ato_index] = point_normal[tid];
		}
	}
}

__global__
void UpdateNormals(float3* __restrict__ prev_normals,
	int* __restrict__ knn_indices,
	float3* __restrict__ normals,
	int knn_size, int com_size)
{
	int tid = blockIdx.x * blockDim.x + threadIdx.x;
	if (tid < com_size)
	{
		for (int j = 0; j < knn_size; j++)
		{
			int idx = knn_indices[tid * knn_size + j];

			/*if (idx == 666)
			{
				printf("prev_normals[idx].x: %f\n", prev_normals[idx].x);
				printf("normals[tid].x: %f\n", normals[tid].x);
			}*/
			/*printf("idx: %d\n", idx);*/

			atomicAdd(&(prev_normals[idx].x), normals[tid].x);
			atomicAdd(&(prev_normals[idx].y), normals[tid].y);
			atomicAdd(&(prev_normals[idx].z), normals[tid].z);
		}
	}
}

__global__
void LoadNormals(float3* __restrict__ prev_normals,
	int* __restrict__ knn_indices,
	float3* __restrict__ normals,
	int knn_size, int com_size)
{
	int tid = blockIdx.x * blockDim.x + threadIdx.x;
	if (tid < com_size)
	{
		float3 sum_normal = make_float3(0.0f, 0.0f, 0.0f);

		for (int j = 0; j < knn_size; j++)
		{
			int idx = knn_indices[tid * knn_size + j];
			sum_normal.x += prev_normals[idx].x;
			sum_normal.y += prev_normals[idx].y;
			sum_normal.z += prev_normals[idx].z;
		}
		sum_normal = make_float3(sum_normal.x / ((float)knn_size), sum_normal.y / ((float)knn_size), sum_normal.z / ((float)knn_size));

		float length = __fsqrt_rn(sum_normal.x * sum_normal.x +
			sum_normal.y * sum_normal.y +
			sum_normal.z * sum_normal.z);

		float normal_scale = __frcp_rn(length + 1e-8);

		normals[tid] = make_float3(sum_normal.x * normal_scale,
			sum_normal.y * normal_scale,
			sum_normal.z * normal_scale);
	}
}

__global__
void NormalizeNormals(float3* __restrict__ tmp_normals, float3* __restrict__ ori_normals, float* __restrict__ lengths, int* __restrict__ zero_size, int com_size)
{
	int tid = blockIdx.x * blockDim.x + threadIdx.x;
	if (tid < com_size)
	{
		float length = __fsqrt_rn(tmp_normals[tid].x * tmp_normals[tid].x +
			tmp_normals[tid].y * tmp_normals[tid].y +
			tmp_normals[tid].z * tmp_normals[tid].z);
		lengths[tid] = length;

		float normal_scale = __frcp_rn(length + 1e-8);

		/*normals[tid] = make_float3(normals[tid].x * normal_scale,
								   normals[tid].y * normal_scale,
								   normals[tid].z * normal_scale);*/

		if (length < 1e-10)
		{
			atomicAdd(zero_size, 1);
		}
		else
		{
			ori_normals[tid] = make_float3(tmp_normals[tid].x * normal_scale,
				tmp_normals[tid].y * normal_scale,
				tmp_normals[tid].z * normal_scale);
		}
	}
}

__inline__ __device__
float3 NormalAdd(const float3& a, const float3& b) {
	return make_float3(a.x + b.x, a.y + b.y, a.z + b.z);
}

__global__
void UpdateNonzeroNormals(
	int* __restrict__ knn_indices,
	float3* __restrict__ normals,
	float3* __restrict__ result,
	int com_size)
{
	int bid = blockIdx.x;

	if (bid >= com_size)
		return;

	__shared__ float3 shared_normals[10];
	int neighbor_start = 10 * bid;

	if (threadIdx.x < 10)
	{
		shared_normals[threadIdx.x] = normals[knn_indices[neighbor_start + threadIdx.x]];
	}
	__syncthreads();

	if (threadIdx.x < 10)
	{
		for (int stride = 1; stride < 10; stride *= 2)
		{
			int index = 2 * stride * threadIdx.x;
			if (index < 10 && (index + stride < 10))
			{
				shared_normals[index] = NormalAdd(shared_normals[index], shared_normals[index + stride]);
			}
			__syncthreads();
		}

		if (threadIdx.x == 0)
		{
			float length = __fsqrt_rn(shared_normals[0].x * shared_normals[0].x +
				shared_normals[0].y * shared_normals[0].y +
				shared_normals[0].z * shared_normals[0].z);

			float normal_scale = __frcp_rn(length + 1e-8);
			shared_normals[0] = make_float3(shared_normals[0].x * normal_scale,
				shared_normals[0].y * normal_scale,
				shared_normals[0].z * normal_scale);

			result[bid] = shared_normals[0];
		}
	}
}

__global__
void SimpleUpdateNonzeroNormals(
	int* __restrict__ knn_indices,
	int* __restrict__ zero_points_index,
	float3* __restrict__ nonzero_points_normal,
	float3* __restrict__ ori_normals,
	int k_size,
	int com_size)
{
	int tid = blockIdx.x * blockDim.x + threadIdx.x;
	if (tid < com_size)
	{
		float3 sum_normal = make_float3(0.0f, 0.0f, 0.0f);

		for (int i = 0; i < k_size; i++)
		{
			int c_index = knn_indices[tid * k_size + i];
			sum_normal.x += nonzero_points_normal[c_index].x;
			sum_normal.y += nonzero_points_normal[c_index].y;
			sum_normal.z += nonzero_points_normal[c_index].z;

			/*printf("knn_indices[i]: %d", knn_indices[i]);*/
		}

		float length = __fsqrt_rn(sum_normal.x * sum_normal.x +
			sum_normal.y * sum_normal.y +
			sum_normal.z * sum_normal.z);

		float normal_scale = __frcp_rn(length + 1e-8);

		int ori_index = zero_points_index[tid];
		ori_normals[ori_index] = make_float3(sum_normal.x * normal_scale,
			sum_normal.y * normal_scale,
			sum_normal.z * normal_scale);

		//printf("ori_normals[ori_index]: (%f, %f, %f)\n", ori_normals[ori_index].x, ori_normals[ori_index].y, ori_normals[ori_index].z);
	}
}

__global__
void SimpleUpdateRadii(
	int* __restrict__ knn_indices,
	float3* __restrict__ points,
	float* __restrict__ radii,
	int k_size,
	int com_size)
{
	int tid = blockIdx.x * blockDim.x + threadIdx.x;
	if (tid < com_size)
	{
		float radii_value = 0;
		for (int i = 0; i < k_size; i++)
		{
			float3 point_diff;
			int c_index = knn_indices[tid * k_size + i];
			point_diff.x += (points[tid].x - points[c_index].x);
			point_diff.y += (points[tid].y - points[c_index].y);
			point_diff.z += (points[tid].z - points[c_index].z);

			radii_value += (point_diff.x * point_diff.x + point_diff.y * point_diff.y + point_diff.z * point_diff.z);
		}

		radii_value /= ((float)k_size);
		radii_value = __fsqrt_rn(radii_value);
		radii_value = (radii_value < 0.0015f) ? 0.0015f : radii_value;
		radii_value = (radii_value > 0.015f) ? 0.015f : radii_value;

		radii[tid] = radii_value;
	}
}

/*__global__
void UpdateNonzeroNormals_15(
	int* __restrict__ knn_indices,
	float3* __restrict__ normals,
	float3* __restrict__ result,
	int com_size)
{
	int bid = blockIdx.x;

	if (bid >= com_size)
		return;

	__shared__ float3 shared_normals[15];
	int neighbor_start = 15 * bid;

	if (threadIdx.x < 15)
	{
		shared_normals[threadIdx.x] = normals[knn_indices[neighbor_start + threadIdx.x]];
	}
	__syncthreads();

	if (threadIdx.x < 15)
	{
		for (int stride = 1; stride < 15; stride *= 2)
		{
			int index = 2 * stride * threadIdx.x;
			if (index < 15 && (index + stride < 15))
			{
				shared_normals[index] = NormalAdd(shared_normals[index], shared_normals[index + stride]);
			}
			__syncthreads();
		}

		if (threadIdx.x == 0)
		{
			float length = __fsqrt_rn(shared_normals[0].x * shared_normals[0].x +
				shared_normals[0].y * shared_normals[0].y +
				shared_normals[0].z * shared_normals[0].z);

			float normal_scale = __frcp_rn(length + 1e-8);
			shared_normals[0] = make_float3(shared_normals[0].x * normal_scale,
				shared_normals[0].y * normal_scale,
				shared_normals[0].z * normal_scale);

			result[bid] = shared_normals[0];
		}
	}
}*/

__global__
void AsFiltertoZero(
	float* __restrict__ As,
	int* __restrict__ As_indices,
	int filter_num)
{
	int tid = blockIdx.x * blockDim.x + threadIdx.x;
	if (tid < filter_num)
	{
		int c_index = As_indices[tid];
		As[c_index] /= 1000.0f;
	}
}

clock_t TurboDWG::TurboScreenedDWGLaunch(std::vector<std::vector<int>>& O_PI, Eigen::MatrixXi& O_CH, Eigen::MatrixXd& O_CM, Eigen::VectorXd& O_R, Eigen::MatrixXd& O_EC, float beta, float sigma_in, int total_iters, bool use_radii, float As_threshold, int input_normal, clock_t start_o, std::string mesh_path, int test_iter)
{
	cudaMallocManaged((char**)&this->m_GWN_GPU, sizeof(*this->m_GWN_GPU));
	this->m_GWN_GPU->PreLibiGWN(O_PI, O_CH, O_CM, O_R, O_EC);

	float* winding_sum = 0;
	cudaMallocManaged((char**)&winding_sum, sizeof(*winding_sum));

	float3* mid_points = 0;
	float3* mid_normals = 0;

	int mesh_buffer_size = std::max((this->m_Grid_Res_X * this->m_Grid_Res_Y * this->m_Grid_Res_Z) / 2, 1000000);
	float3* mid_points_buffer = 0;
	float3* mid_normals_buffer = 0;
	cudaMallocManaged((char**)&mid_points_buffer, mesh_buffer_size * sizeof(*mid_points_buffer));
	cudaMallocManaged((char**)&mid_normals_buffer, mesh_buffer_size * sizeof(*mid_normals_buffer));

	int* mid_points_knn_gpu = 0;
	int* zero_points_knn_gpu = 0;

	float3* tmp_normals = 0;
	cudaMallocManaged((char**)&tmp_normals, this->m_Point_Cloud_size * sizeof(*tmp_normals));
	float* tmp_lengths = 0;
	cudaMallocManaged((char**)&tmp_lengths, this->m_Point_Cloud_size * sizeof(*tmp_lengths));
	int* zero_size = 0;
	cudaMallocManaged((char**)&zero_size, sizeof(*zero_size));

	int* c_zero_index = 0;
	cudaMallocManaged((char**)&c_zero_index, sizeof(*c_zero_index));
	int* c_nonzero_index = 0;
	cudaMallocManaged((char**)&c_nonzero_index, sizeof(*c_nonzero_index));

	float3* zero_points = 0;
	float3* nonzero_points = 0;
	int* zero_points_index = 0;
	int* nonzero_points_index = 0;
	float3* nonzero_points_normal = 0;

	float3* nonzero_points_upper = 0;
	float3* nonzero_points_lower = 0;
	cudaMallocManaged((char**)&nonzero_points_upper, sizeof(*nonzero_points_upper));
	cudaMallocManaged((char**)&nonzero_points_lower, sizeof(*nonzero_points_lower));

	float3* tmp_zero_normals = 0;

	clock_t end_o = clock();
	std::cout << "Prepare Time = " << (end_o - start_o) / CLOCKS_PER_SEC << std::endl;

	if (input_normal == 3)
	{
		TurboNormalInitialization turbo_normal_init;
		turbo_normal_init.GaussMapInit(this->m_Point_Cloud_GPU, this->m_Normals_GPU, this->m_Bound_Upper_KDT_GPU, this->m_Bound_Lower_KDT_GPU, this->m_Point_Cloud_size);

		std::cout << "Gauss Map Normal Initialized" << std::endl;
	}

	OutlierFilter(this->m_As_GPU, As_threshold, this->m_Point_Cloud_size);

	clock_t start_iter = clock();
	int c_iter = 0;
	while (c_iter < total_iters)
	{
		clock_t start, finish;
		start = clock();

		std::cout << "iter = " << c_iter << std::endl;

		clock_t start_gwn, finish_gwn;
		start_gwn = clock();

		float sigma = sigma_in;
		if (c_iter < 4) {
			sigma = 20.0f; // 20.0

		}
		if (c_iter > 0 && c_iter % 7 == 0) {
			sigma = 10.0f; // 10.0
		}

		this->m_GWN_GPU->UpdateMassNormals(this->m_Normals_GPU, this->m_As_GPU);
		this->m_GWN_GPU->TurboLibiScreenedGWNLaunch(this->m_Point_Cloud_GPU, this->m_Point_Cloud_GPU, this->m_Point_Cloud_size, this->m_Normals_GPU, this->m_As_GPU, this->m_Radii_GPU, beta, sigma, this->m_PC_WN_GPU, use_radii);
		this->m_GWN_GPU->TurboLibiScreenedGWNLaunch(this->m_Grid_Points_GPU, this->m_Point_Cloud_GPU, this->m_Grid_Points_size, this->m_Normals_GPU, this->m_As_GPU, this->m_Radii_GPU, beta, sigma, this->m_GP_WN_GPU, use_radii);
		finish_gwn = clock();
		std::cout << "GWN time = " << (finish_gwn - start_gwn) / CLOCKS_PER_SEC << std::endl;

		thrust::device_ptr<float> pc_wn_thrust_ptr(this->m_PC_WN_GPU);
		float pc_wn_sum = thrust::reduce(pc_wn_thrust_ptr, pc_wn_thrust_ptr + this->m_Point_Cloud_size, 0.0f, thrust::plus<float>());
		float mean_wn = pc_wn_sum / ((float)this->m_Point_Cloud_size);

		std::cout << "W_mean = " << mean_wn << std::endl;

		if (c_iter % 5 == 0 && c_iter != 0)
		{
			mean_wn *= 0.5;
		}

		TurboMC turbomc;
		std::string mesh_full_path = "nothing";
		int num_tri = turbomc.TurboMarchingCubes(this->m_GP_WN_GPU, this->m_Grid_Points_GPU, this->m_Grid_Res_X, this->m_Grid_Res_Y, this->m_Grid_Res_Z, mean_wn, c_iter, mesh_path, mesh_full_path, mid_points_buffer, mid_normals_buffer, test_iter);
		cudaMallocManaged((char**)&mid_points, num_tri * sizeof(*mid_points));
		cudaMallocManaged((char**)&mid_normals, num_tri * sizeof(*mid_normals));
		cudaMemcpy(mid_points, mid_points_buffer, num_tri * sizeof(*mid_points_buffer), cudaMemcpyDeviceToDevice);
		cudaMemcpy(mid_normals, mid_normals_buffer, num_tri * sizeof(*mid_normals_buffer), cudaMemcpyDeviceToDevice);

		int k_size = 15;
		cudaMallocManaged((void**)&mid_points_knn_gpu, num_tri * k_size * sizeof(*mid_points_knn_gpu));
		CudaKNNWithKDTreeforDWG(this->m_Point_Cloud_GPU,
			this->m_Point_Cloud_size,
			mid_points,
			num_tri,
			this->m_Bound_Upper_KDT_GPU, this->m_Bound_Lower_KDT_GPU,
			mid_points_knn_gpu,
			k_size
		);

		int threadsPerBlock_normalize = 128;
		int blocksPerGrid_normalize = (this->m_Point_Cloud_size + threadsPerBlock_normalize - 1) / threadsPerBlock_normalize;
		ResetZero << < blocksPerGrid_normalize, threadsPerBlock_normalize >> > (tmp_normals, this->m_Point_Cloud_size);
		cudaDeviceSynchronize();

		int threadsPerBlock = 128;
		int blocksPerGrid = (num_tri + threadsPerBlock - 1) / threadsPerBlock;
		UpdateNormals << < blocksPerGrid, threadsPerBlock >> > (tmp_normals, mid_points_knn_gpu, mid_normals, k_size, num_tri);
		cudaDeviceSynchronize();

		zero_size[0] = 0;
		NormalizeNormals << < blocksPerGrid_normalize, threadsPerBlock_normalize >> > (tmp_normals, this->m_Normals_GPU, tmp_lengths, zero_size, this->m_Point_Cloud_size);
		cudaDeviceSynchronize();

		int zero_k_size = 10;
		if (zero_size[0] > zero_k_size)
		{
			int nonzero_size = this->m_Point_Cloud_size - zero_size[0];
			cudaMallocManaged((char**)&zero_points, zero_size[0] * sizeof(*zero_points));
			cudaMallocManaged((char**)&nonzero_points, nonzero_size * sizeof(*nonzero_points));
			cudaMallocManaged((char**)&zero_points_index, zero_size[0] * sizeof(*zero_points_index));
			cudaMallocManaged((char**)&nonzero_points_index, nonzero_size * sizeof(*nonzero_points_index));
			cudaMallocManaged((char**)&nonzero_points_normal, nonzero_size * sizeof(*nonzero_points_normal));

			c_zero_index[0] = 0;
			c_nonzero_index[0] = 0;
			LoadZeroData << < blocksPerGrid_normalize, threadsPerBlock_normalize >> > (zero_points, nonzero_points, zero_points_index, nonzero_points_index, nonzero_points_normal, tmp_lengths,
				this->m_Point_Cloud_GPU, this->m_Normals_GPU, c_zero_index, c_nonzero_index, this->m_Point_Cloud_size);
			cudaDeviceSynchronize();

			GenerateKDTreeForDWG(nonzero_points, nonzero_points_normal, nonzero_size, nonzero_points_upper, nonzero_points_lower);

			cudaMallocManaged((void**)&zero_points_knn_gpu, zero_size[0] * zero_k_size * sizeof(*zero_points_knn_gpu));
			CudaKNNWithKDTreeforDWG(nonzero_points,
				nonzero_size,
				zero_points,
				zero_size[0],
				nonzero_points_upper, nonzero_points_lower,
				zero_points_knn_gpu,
				zero_k_size
			);

			std::cout << "zero_size[0] = " << zero_size[0] << std::endl;
			cudaMallocManaged((void**)&tmp_zero_normals, zero_size[0] * sizeof(*tmp_zero_normals));
			//int threadsPerBlock_zero = 32;
			//int blocksPerGrid_zero = (zero_size[0] + threadsPerBlock_zero - 1) / threadsPerBlock_zero;
			//UpdateNonzeroNormals << < blocksPerGrid_zero, threadsPerBlock_zero >> > (zero_points_knn_gpu, this->m_Normals_GPU, tmp_zero_normals, zero_size[0]);

			int threadsPerBlock_zero = 128;
			int blocksPerGrid_zero = (zero_size[0] + threadsPerBlock - 1) / threadsPerBlock;
			SimpleUpdateNonzeroNormals << < blocksPerGrid_zero, threadsPerBlock_zero >> > (zero_points_knn_gpu, zero_points_index, nonzero_points_normal, this->m_Normals_GPU, zero_k_size, zero_size[0]);
			cudaDeviceSynchronize();
			/*for (int i = 0; i < zero_size[0]; i++)
			{
				this->m_Normals_GPU[zero_points_index[i]] = tmp_zero_normals[i];
			}*/

			std::cout << "number of zero normals = " << zero_size[0] << std::endl;

			cudaFree(zero_points_knn_gpu);
			zero_points_knn_gpu = 0;
			cudaFree(zero_points);
			zero_points = 0;
			cudaFree(nonzero_points);
			nonzero_points = 0;
			cudaFree(zero_points_index);
			zero_points_index = 0;
			cudaFree(nonzero_points_index);
			nonzero_points_index = 0;
			cudaFree(nonzero_points_normal);
			nonzero_points_normal = 0;
			cudaFree(tmp_zero_normals);
			tmp_zero_normals = 0;
		}

		cudaFree(mid_points);
		mid_points = 0;
		cudaFree(mid_normals);
		mid_normals = 0;
		cudaFree(mid_points_knn_gpu);
		mid_points_knn_gpu = 0;

		c_iter++;

		finish = clock();
		std::cout << "iter time = " << (finish - start) / CLOCKS_PER_SEC << std::endl;
		std::cout << std::endl;
	}

	cudaFree(winding_sum); winding_sum = 0;
	cudaFree(tmp_normals); tmp_normals = 0;
	cudaFree(tmp_lengths); tmp_lengths = 0;
	cudaFree(zero_size); zero_size = 0;
	cudaFree(nonzero_points_upper); nonzero_points_upper = 0;
	cudaFree(nonzero_points_lower); nonzero_points_lower = 0;
	cudaFree(this->m_Grid_Points_GPU); this->m_Grid_Points_GPU = 0;
	cudaFree(this->m_GP_WN_GPU); this->m_GP_WN_GPU = 0;

	clock_t end_iter = clock();

	return end_iter - start_iter;
}

void TurboDWG::UpdateRadii(int k_size)
{
	int* knn_points = 0;
	cudaMallocManaged((void**)&knn_points, this->m_Point_Cloud_size * k_size * sizeof(*knn_points));
	CudaKNNWithKDTreeforDWG(this->m_Point_Cloud_GPU,
		this->m_Point_Cloud_size,
		this->m_Point_Cloud_GPU,
		this->m_Point_Cloud_size,
		this->m_Bound_Upper_KDT_GPU, this->m_Bound_Lower_KDT_GPU,
		knn_points,
		k_size
	);

	int threadsPerBlock = 128;
	int blocksPerGrid = (this->m_Point_Cloud_size + threadsPerBlock - 1) / threadsPerBlock;
	SimpleUpdateRadii << < blocksPerGrid, threadsPerBlock >> > (knn_points, this->m_Point_Cloud_GPU, this->m_Radii_GPU, k_size, this->m_Point_Cloud_size);
	cudaDeviceSynchronize();
}

void TurboDWG::UpdateAs(Eigen::VectorXd& leaves_As)
{
	std::unique_ptr<float[]> h_As = std::make_unique<float[]>(size_t(this->m_Point_Cloud_size));
	for (int i = 0; i < this->m_Point_Cloud_size; i++)
	{
		h_As[i] = (float)leaves_As[i];
	}
	gpuErrchk(cudaMemcpy(this->m_As_GPU, h_As.get(), this->m_Point_Cloud_size * sizeof(float), cudaMemcpyHostToDevice));
}

void TurboDWG::PreGeneration(Eigen::MatrixXf& cloud_points, Eigen::VectorXf& points_As, Eigen::MatrixXf& grid_points, int res_x, int res_y, int res_z, std::vector<float> bound_upper_kdtree, std::vector<float> bound_lower_kdtree, int k_normal_size, Eigen::MatrixXd& out_normal)
{
	cudaFree(this->m_PC_WN_GPU); this->m_PC_WN_GPU = 0;
	cudaFree(this->m_As_GPU); this->m_As_GPU = 0;
	cudaFree(this->m_Radii_GPU); this->m_Radii_GPU = 0;

	this->m_GWN_GPU->EarseMemory();

	PreTurboDWG(cloud_points.rows(), points_As, grid_points, res_x, res_y, res_z);

	float3* cloud_points_gpu = 0;
	int cloud_points_size = cloud_points.rows();
	cudaMallocManaged((char**)&cloud_points_gpu, cloud_points_size * sizeof(*cloud_points_gpu));
	std::unique_ptr<float3[]> cloud_points_cpu = std::make_unique<float3[]>(size_t(cloud_points_size));
	for (int i = 0; i < cloud_points_size; i++)
	{
		cloud_points_cpu[i] = make_float3(cloud_points(i, 0), cloud_points(i, 1), cloud_points(i, 2));
	}
	gpuErrchk(cudaMemcpy(cloud_points_gpu, cloud_points_cpu.get(), cloud_points_size * sizeof(float3), cudaMemcpyHostToDevice));

	int* knn_normal_points = 0;
	//int k_normal_size = 1; //DO NOT change it
	cudaMallocManaged((void**)&knn_normal_points, cloud_points_size * k_normal_size * sizeof(*knn_normal_points));
	CudaKNNWithKDTreeforDWG(this->m_Point_Cloud_GPU,
		this->m_Point_Cloud_size,
		cloud_points_gpu,
		cloud_points_size,
		this->m_Bound_Upper_KDT_GPU, this->m_Bound_Lower_KDT_GPU,
		knn_normal_points,
		k_normal_size
	);

	cudaFree(this->m_Point_Cloud_GPU); this->m_Point_Cloud_GPU = 0;
	float3* points_normal_gpu = 0;
	cudaMallocManaged((char**)&points_normal_gpu, cloud_points_size * sizeof(*points_normal_gpu));

	int threadsPerBlock = 128;
	int blocksPerGrid = (cloud_points_size + threadsPerBlock - 1) / threadsPerBlock;
	LoadNormals << < blocksPerGrid, threadsPerBlock >> > (this->m_Normals_GPU, knn_normal_points, points_normal_gpu, k_normal_size, cloud_points_size);
	cudaDeviceSynchronize();

	cudaFree(this->m_Normals_GPU); this->m_Normals_GPU = 0;

	this->m_Bound_Upper_KDT_GPU[0].x = bound_upper_kdtree[0];
	this->m_Bound_Upper_KDT_GPU[0].y = bound_upper_kdtree[1];
	this->m_Bound_Upper_KDT_GPU[0].z = bound_upper_kdtree[2];

	this->m_Bound_Lower_KDT_GPU[0].x = bound_lower_kdtree[0];
	this->m_Bound_Lower_KDT_GPU[0].y = bound_lower_kdtree[1];
	this->m_Bound_Lower_KDT_GPU[0].z = bound_lower_kdtree[2];

	this->m_Point_Cloud_GPU = cloud_points_gpu;
	this->m_Point_Cloud_size = cloud_points_size;
	this->m_Normals_GPU = points_normal_gpu;

	std::unique_ptr<float3[]> h_normal = std::make_unique<float3[]>(size_t(this->m_Point_Cloud_size));
	cudaMemcpy(h_normal.get(), this->m_Normals_GPU, this->m_Point_Cloud_size * sizeof(float3), cudaMemcpyDeviceToHost);
	for (int i = 0; i < this->m_Point_Cloud_size; ++i)
	{
		out_normal(i, 0) = (double)h_normal[i].x;
		out_normal(i, 1) = (double)h_normal[i].y;
		out_normal(i, 2) = (double)h_normal[i].z;
	}
}

void TurboDWG::GenerateMesh(Eigen::MatrixXf& grid_points, int res_x, int res_y, int res_z, float beta, float sigma, int k_size, bool use_radii, std::string mesh_full_path, clock_t iter_time, clock_t start)
{
	clock_t start_time = clock();
	float3* grid_points_gpu = 0;
	int grid_points_size = grid_points.rows();
	cudaMallocManaged((char**)&grid_points_gpu, grid_points_size * sizeof(*grid_points_gpu));
	std::unique_ptr<float3[]> grid_points_cpu = std::make_unique<float3[]>(size_t(grid_points_size));
	for (int i = 0; i < grid_points_size; i++)
	{
		grid_points_cpu[i] = make_float3(grid_points(i, 0), grid_points(i, 1), grid_points(i, 2));
	}
	gpuErrchk(cudaMemcpy(grid_points_gpu, grid_points_cpu.get(), grid_points_size * sizeof(float3), cudaMemcpyHostToDevice));

	float* grid_point_gwn = 0;
	cudaMallocManaged((char**)&grid_point_gwn, grid_points_size * sizeof(*grid_point_gwn));

	UpdateRadii(k_size);
	this->m_GWN_GPU->UpdateMassNormals(this->m_Normals_GPU, this->m_As_GPU);
	this->m_GWN_GPU->TurboLibiScreenedGWNLaunch(this->m_Point_Cloud_GPU, this->m_Point_Cloud_GPU, this->m_Point_Cloud_size, this->m_Normals_GPU, this->m_As_GPU, this->m_Radii_GPU, beta, sigma, this->m_PC_WN_GPU, use_radii);
	this->m_GWN_GPU->TurboLibiScreenedGWNLaunch(grid_points_gpu, this->m_Point_Cloud_GPU, grid_points_size, this->m_Normals_GPU, this->m_As_GPU, this->m_Radii_GPU, beta, sigma, grid_point_gwn, use_radii);

	thrust::device_ptr<float> pc_wn_thrust_ptr(this->m_PC_WN_GPU);
	float pc_wn_sum = thrust::reduce(pc_wn_thrust_ptr, pc_wn_thrust_ptr + this->m_Point_Cloud_size, 0.0f, thrust::plus<float>());
	float mean_wn = pc_wn_sum / ((float)this->m_Point_Cloud_size);

	int mesh_buffer_size = std::max((res_x * res_y * res_z) / 2, 1000000);
	float3* mid_points_buffer = 0;
	float3* mid_normals_buffer = 0;
	cudaMallocManaged((char**)&mid_points_buffer, mesh_buffer_size * sizeof(*mid_points_buffer));
	cudaMallocManaged((char**)&mid_normals_buffer, mesh_buffer_size * sizeof(*mid_normals_buffer));

	TurboMC turbomc;
	std::string mesh_path = "nothing";
	int num_tri = turbomc.TurboMarchingCubes(grid_point_gwn, grid_points_gpu, res_x, res_y, res_z, mean_wn, 123456, mesh_path, mesh_full_path, mid_points_buffer, mid_normals_buffer, 999999);

	clock_t end_time = clock();
	std::cout << "Iter Time = " << (end_time - start_time + iter_time) / CLOCKS_PER_SEC << std::endl;
	std::cout << "Total Time = " << (end_time - start) / CLOCKS_PER_SEC << std::endl;
}

void TurboDWG::GenerateMesh(Eigen::MatrixXf& cloud_points, Eigen::VectorXf& points_As, Eigen::MatrixXf& grid_points,
	std::vector<std::vector<int>>& O_PI, Eigen::MatrixXi& O_CH, Eigen::MatrixXd& O_CM, Eigen::VectorXd& O_R, Eigen::MatrixXd& O_EC,
	int res_x, int res_y, int res_z,
	std::vector<float> bound_upper_kdtree, std::vector<float> bound_lower_kdtree,
	float beta, float sigma, int k_size, int k_normal_size, bool use_radii, float As_threshold, std::string mesh_full_path, clock_t iter_time, clock_t start, int output_mode)
{
	clock_t start_time = clock();

	cudaFree(this->m_PC_WN_GPU); this->m_PC_WN_GPU = 0;
	cudaFree(this->m_As_GPU); this->m_As_GPU = 0;
	cudaFree(this->m_Radii_GPU); this->m_Radii_GPU = 0;

	this->m_GWN_GPU->EarseMemory();

	PreTurboDWG(cloud_points.rows(), points_As, grid_points, res_x, res_y, res_z);

	float3* cloud_points_gpu = 0;
	int cloud_points_size = cloud_points.rows();
	cudaMallocManaged((char**)&cloud_points_gpu, cloud_points_size * sizeof(*cloud_points_gpu));
	std::unique_ptr<float3[]> cloud_points_cpu = std::make_unique<float3[]>(size_t(cloud_points_size));
	for (int i = 0; i < cloud_points_size; i++)
	{
		cloud_points_cpu[i] = make_float3(cloud_points(i, 0), cloud_points(i, 1), cloud_points(i, 2));
	}
	gpuErrchk(cudaMemcpy(cloud_points_gpu, cloud_points_cpu.get(), cloud_points_size * sizeof(float3), cudaMemcpyHostToDevice));

	int* knn_normal_points = 0;
	//int k_normal_size = 1; //DO NOT change it
	cudaMallocManaged((void**)&knn_normal_points, cloud_points_size * k_normal_size * sizeof(*knn_normal_points));
	CudaKNNWithKDTreeforDWG(this->m_Point_Cloud_GPU,
		this->m_Point_Cloud_size,
		cloud_points_gpu,
		cloud_points_size,
		this->m_Bound_Upper_KDT_GPU, this->m_Bound_Lower_KDT_GPU,
		knn_normal_points,
		k_normal_size
	);

	cudaFree(this->m_Point_Cloud_GPU); this->m_Point_Cloud_GPU = 0;
	float3* points_normal_gpu = 0;
	cudaMallocManaged((char**)&points_normal_gpu, cloud_points_size * sizeof(*points_normal_gpu));

	int threadsPerBlock = 128;
	int blocksPerGrid = (cloud_points_size + threadsPerBlock - 1) / threadsPerBlock;
	LoadNormals << < blocksPerGrid, threadsPerBlock >> > (this->m_Normals_GPU, knn_normal_points, points_normal_gpu, k_normal_size, cloud_points_size);
	cudaDeviceSynchronize();

	cudaFree(this->m_Normals_GPU); this->m_Normals_GPU = 0;

	this->m_Bound_Upper_KDT_GPU[0].x = bound_upper_kdtree[0];
	this->m_Bound_Upper_KDT_GPU[0].y = bound_upper_kdtree[1];
	this->m_Bound_Upper_KDT_GPU[0].z = bound_upper_kdtree[2];

	this->m_Bound_Lower_KDT_GPU[0].x = bound_lower_kdtree[0];
	this->m_Bound_Lower_KDT_GPU[0].y = bound_lower_kdtree[1];
	this->m_Bound_Lower_KDT_GPU[0].z = bound_lower_kdtree[2];

	this->m_Point_Cloud_GPU = cloud_points_gpu;
	this->m_Point_Cloud_size = cloud_points_size;
	this->m_Normals_GPU = points_normal_gpu;

	TurboLibiGWN turbogwn;
	turbogwn.PreLibiGWN(O_PI, O_CH, O_CM, O_R, O_EC);

	UpdateRadii(k_size);

	turbogwn.UpdateMassNormals(this->m_Normals_GPU, this->m_As_GPU);

	OutlierFilter(this->m_As_GPU, As_threshold, this->m_Point_Cloud_size);

	std::cout << "Start GWN Computation" << std::endl;
	turbogwn.TurboLibiScreenedGWNLaunch(this->m_Point_Cloud_GPU, this->m_Point_Cloud_GPU, this->m_Point_Cloud_size, this->m_Normals_GPU, this->m_As_GPU, this->m_Radii_GPU, beta, sigma, this->m_PC_WN_GPU, use_radii);
	turbogwn.TurboLibiScreenedGWNLaunch(this->m_Grid_Points_GPU, this->m_Point_Cloud_GPU, this->m_Grid_Points_size, this->m_Normals_GPU, this->m_As_GPU, this->m_Radii_GPU, beta, sigma, this->m_GP_WN_GPU, use_radii);

	thrust::device_ptr<float> pc_wn_thrust_ptr(this->m_PC_WN_GPU);
	float pc_wn_sum = thrust::reduce(pc_wn_thrust_ptr, pc_wn_thrust_ptr + this->m_Point_Cloud_size, 0.0f, thrust::plus<float>());
	float mean_wn = pc_wn_sum / ((float)this->m_Point_Cloud_size);

	int mesh_buffer_size = std::max((res_x * res_y * res_z) / 2, 1000000);
	float3* mid_points_buffer = 0;
	float3* mid_normals_buffer = 0;
	cudaMallocManaged((char**)&mid_points_buffer, mesh_buffer_size * sizeof(*mid_points_buffer));
	cudaMallocManaged((char**)&mid_normals_buffer, mesh_buffer_size * sizeof(*mid_normals_buffer));

	if (output_mode == 1 || output_mode == 3)
	{
		std::string mesh_path = "nothing";
		TurboMC turbomc;
		int num_tri = turbomc.TurboMarchingCubes(this->m_GP_WN_GPU, this->m_Grid_Points_GPU, res_x, res_y, res_z, mean_wn, 123456, mesh_path, mesh_full_path, mid_points_buffer, mid_normals_buffer, 999999);

		clock_t end_time = clock();
		std::cout << "Iter Time = " << iter_time / CLOCKS_PER_SEC << std::endl;
		std::cout << "Total Time = " << (end_time - start) / CLOCKS_PER_SEC << std::endl;
	}
	
	if(output_mode == 2 || output_mode == 3)
	{
		if (output_mode == 2)
		{
			clock_t end_time = clock();
			std::cout << "Iter Time = " << iter_time / CLOCKS_PER_SEC << std::endl;
			std::cout << "Total Time = " << (end_time - start) / CLOCKS_PER_SEC << std::endl;
		}

		std::cout << "Start save" << std::endl;

		std::ofstream outFile(mesh_full_path + ".xyz");
		for (int i = 0; i < this->m_Point_Cloud_size; i++)
		{
			outFile << this->m_Point_Cloud_GPU[i].x << " " << this->m_Point_Cloud_GPU[i].y << " " << this->m_Point_Cloud_GPU[i].z << " " <<
				this->m_Normals_GPU[i].x << " " << this->m_Normals_GPU[i].y << " " << this->m_Normals_GPU[i].z << "\n";
		}

		outFile.close();
		std::cout << "Finish save" << std::endl;
	}
}

void TurboDWG::MeshGeneration(std::vector<std::vector<int>>& O_PI, Eigen::MatrixXi& O_CH, Eigen::MatrixXd& O_CM, Eigen::VectorXd& O_R, Eigen::MatrixXd& O_EC, int res_x, int res_y, int res_z, float beta, float sigma, int k_size, bool use_radii, float As_threshold, std::string mesh_full_path, clock_t start, std::chrono::high_resolution_clock::time_point start_time_s, int output_mode)
{
	TurboLibiGWN turbogwn;
	turbogwn.PreLibiGWN(O_PI, O_CH, O_CM, O_R, O_EC);

	turbogwn.UpdateGWNInfo(this->m_Normals_GPU, this->m_Point_Cloud_GPU, this->m_As_GPU);

	UpdateRadii(k_size);

	OutlierFilter(this->m_As_GPU, As_threshold, this->m_Point_Cloud_size);

	std::cout << "Start GWN Computation" << std::endl;
	turbogwn.TurboLibiScreenedGWNLaunch(this->m_Point_Cloud_GPU, this->m_Point_Cloud_GPU, this->m_Point_Cloud_size, this->m_Normals_GPU, this->m_As_GPU, this->m_Radii_GPU, beta, sigma, this->m_PC_WN_GPU, use_radii);
	turbogwn.TurboLibiScreenedGWNLaunch(this->m_Grid_Points_GPU, this->m_Point_Cloud_GPU, this->m_Grid_Points_size, this->m_Normals_GPU, this->m_As_GPU, this->m_Radii_GPU, beta, sigma, this->m_GP_WN_GPU, use_radii);

	thrust::device_ptr<float> pc_wn_thrust_ptr(this->m_PC_WN_GPU);
	float pc_wn_sum = thrust::reduce(pc_wn_thrust_ptr, pc_wn_thrust_ptr + this->m_Point_Cloud_size, 0.0f, thrust::plus<float>());
	float mean_wn = pc_wn_sum / ((float)this->m_Point_Cloud_size);

	int mesh_buffer_size = std::max((res_x * res_y * res_z) / 2, 1000000);
	float3* c_mid_points_buffer = 0;
	float3* c_mid_normals_buffer = 0;
	cudaMallocManaged((char**)&c_mid_points_buffer, mesh_buffer_size * sizeof(*c_mid_points_buffer));
	cudaMallocManaged((char**)&c_mid_normals_buffer, mesh_buffer_size * sizeof(*c_mid_normals_buffer));

	if (output_mode == 1 || output_mode == 3)
	{
		std::string mesh_path = "nothing";
		TurboMC turbomc;
		int num_tri = turbomc.TurboMarchingCubes(this->m_GP_WN_GPU, this->m_Grid_Points_GPU, res_x, res_y, res_z, mean_wn, 123456, mesh_path, mesh_full_path, c_mid_points_buffer, c_mid_normals_buffer, 999999);

		clock_t end_time = clock();
		std::chrono::high_resolution_clock::time_point end_time_s = std::chrono::high_resolution_clock::now();
		std::chrono::duration<double> elapsed_time = std::chrono::duration_cast<std::chrono::duration<double>>(end_time_s - start_time_s);

		std::cout << "Total Time = " << (end_time - start) / CLOCKS_PER_SEC << std::endl;
		std::cout << "Total Time (Wall time) = " << elapsed_time.count() << " s" << std::endl;
	}

	if (output_mode == 2 || output_mode == 3)
	{
		if (output_mode == 2)
		{
			clock_t end_time = clock();
			std::chrono::high_resolution_clock::time_point end_time_s = std::chrono::high_resolution_clock::now();
			std::chrono::duration<double> elapsed_time = std::chrono::duration_cast<std::chrono::duration<double>>(end_time_s - start_time_s);

			std::cout << "Total Time = " << (end_time - start) / CLOCKS_PER_SEC << std::endl;
			std::cout << "Total Time (Wall time) = " << elapsed_time.count() << " s" << std::endl;
		}

		std::cout << "Start save" << std::endl;

		std::ofstream outFile(mesh_full_path + ".xyz");
		for (int i = 0; i < this->m_Point_Cloud_size; i++)
		{
			outFile << this->m_Point_Cloud_GPU[i].x << " " << this->m_Point_Cloud_GPU[i].y << " " << this->m_Point_Cloud_GPU[i].z << " " <<
				this->m_Normals_GPU[i].x << " " << this->m_Normals_GPU[i].y << " " << this->m_Normals_GPU[i].z << "\n";
		}

		outFile.close();
		std::cout << "Finish save" << std::endl;
	}
}

void TurboDWG::OutlierFilter(float* As, float threshold, int point_num)
{
	thrust::device_vector<int> d_indices(point_num);
	thrust::sequence(d_indices.begin(), d_indices.end());

	thrust::device_vector<float> d_data(As, As + point_num);

	thrust::sort_by_key(d_data.begin(), d_data.end(), d_indices.begin(), thrust::greater<float>());

	int threshold_max_index = ceil((float)point_num * threshold);

	int threadsPerBlock = 128;
	int blocksPerGrid = (threshold_max_index + threadsPerBlock - 1) / threadsPerBlock;

	int* raw_indices = thrust::raw_pointer_cast(d_indices.data());

	AsFiltertoZero << < blocksPerGrid, threadsPerBlock >> > (As, raw_indices, threshold_max_index);
	cudaDeviceSynchronize();
}