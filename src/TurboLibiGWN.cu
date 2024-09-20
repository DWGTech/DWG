#include "Turbo_GWN_l.h"

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

inline dim3 computeNbBlocks(unsigned int nbThreads, unsigned int nbThreadsPerBlock) {
	/*dim3 nbBlocks = ceil((float)(nbThreads) / (float)nbThreadsPerBlock);
	if (nbBlocks.x > 65535) {
		nbBlocks.y = ceil((float)nbBlocks.x / (float)65535);
		nbBlocks.x = 65535;
	}*/
	dim3 nbBlocks = ceil(((float)(nbThreads)+(float)nbThreadsPerBlock - 1.0f) / (float)nbThreadsPerBlock);
	return nbBlocks;
}

TurboLibiGWN::TurboLibiGWN(std::vector<std::vector<int>>& O_PI, Eigen::MatrixXi& O_CH, Eigen::MatrixXd& O_CM, Eigen::VectorXd& O_R, Eigen::MatrixXd& O_EC, std::vector<geometrycentral::Vector3>& point_cloud)
{
	gpuErrchk(cudaPeekAtLastError());

	unsigned int node_num = O_R.rows();

	this->m_Node_Size = node_num;
	cudaMallocManaged((char**)&this->m_Octree_GPU, node_num * sizeof(*this->m_Octree_GPU));
	this->CopyToGPU(O_PI, O_CH, O_CM, O_R, O_EC, point_cloud);
}

TurboLibiGWN::TurboLibiGWN(std::vector<std::vector<int>>& O_PI, Eigen::MatrixXi& O_CH, Eigen::MatrixXd& O_CM, Eigen::VectorXd& O_R, Eigen::MatrixXd& O_EC)
{
	gpuErrchk(cudaPeekAtLastError());

	unsigned int node_num = O_R.rows();

	std::cout << "node_num: " << node_num << std::endl;

	this->m_Node_Size = node_num;
	cudaMallocManaged((char**)&this->m_Octree_GPU, node_num * sizeof(*this->m_Octree_GPU));

	int child_num = 8;

	uint32_t num_indices_to_copy = 0;
	std::vector<unsigned int> whose_indices_to_copy;

	for (int i = 0; i < O_R.rows(); i++)
	{
		this->m_Octree_GPU[i].Mass_Center.x = O_CM(i, 0);
		this->m_Octree_GPU[i].Mass_Center.y = O_CM(i, 1);
		this->m_Octree_GPU[i].Mass_Center.z = O_CM(i, 2);

		this->m_Octree_GPU[i].Mass_Normal.x = O_EC(i, 0);
		this->m_Octree_GPU[i].Mass_Normal.y = O_EC(i, 1);
		this->m_Octree_GPU[i].Mass_Normal.z = O_EC(i, 2);

		this->m_Octree_GPU[i].Max_R = O_R[i];
	}

	this->m_Octree_GPU[0].Father = -1;
	for (int i = 0; i < O_CH.rows(); i++)
	{
		for (int j = 0; j < child_num; j++)
		{
			int c_ci = O_CH(i, j);

			if (c_ci != -1)
			{
				this->m_Octree_GPU[c_ci].Father = i;
				this->m_Octree_GPU[c_ci].IndexInFather = j;
			}
		}

		this->m_Octree_GPU[i].Point_Cloud_Num = O_PI[i].size();

		if (this->m_Octree_GPU[i].Point_Cloud_Num != 0)
		{
			whose_indices_to_copy.push_back(i);
			num_indices_to_copy += this->m_Octree_GPU[i].Point_Cloud_Num;
		}
	}

	if (num_indices_to_copy > 0)
	{
		unsigned int* point_indices = nullptr;
		cudaMallocManaged((char**)&point_indices, num_indices_to_copy * sizeof(*point_indices));

		uint32_t offset = 0;
		for (unsigned int i : whose_indices_to_copy)
		{
			this->m_Octree_GPU[i].Point_Cloud_Indices = point_indices + offset;
			offset += this->m_Octree_GPU[i].Point_Cloud_Num;
		}

		std::unique_ptr<unsigned int[]> idxCPU = std::make_unique<unsigned int[]>(num_indices_to_copy * sizeof(unsigned int));
		unsigned int* idx = idxCPU.get();
		for (unsigned int i : whose_indices_to_copy)
		{
			for (unsigned int j = 0; j < this->m_Octree_GPU[i].Point_Cloud_Num; j++)
				*(idx++) = O_PI[i][j];
		}
		gpuErrchk(cudaMemcpy(point_indices, idxCPU.get(), num_indices_to_copy * sizeof(unsigned int), cudaMemcpyHostToDevice));
	}

	int* node_indices = nullptr;
	int total_node_num = ((int)O_CH.rows()) * child_num;
	cudaMallocManaged((char**)&node_indices, total_node_num * sizeof(*node_indices));

	uint32_t offset = 0;
	for (int i = 0; i < O_CH.rows(); i++)
	{
		this->m_Octree_GPU[i].Children = node_indices + offset;
		offset += child_num;
	}

	std::unique_ptr<int[]> idxCPU = std::make_unique<int[]>(total_node_num * sizeof(int));
	int* idx = idxCPU.get();
	for (int i = 0; i < O_CH.rows(); i++)
	{
		for (int j = 0; j < 8; j++)
			*(idx++) = O_CH(i, j);
	}
	gpuErrchk(cudaMemcpy(node_indices, idxCPU.get(), total_node_num * sizeof(int), cudaMemcpyHostToDevice));

	std::cout << "node_num: " << node_num << std::endl;
}

void TurboLibiGWN::PreLibiGWN(std::vector<std::vector<int>>& O_PI, Eigen::MatrixXi& O_CH, Eigen::MatrixXd& O_CM, Eigen::VectorXd& O_R, Eigen::MatrixXd& O_EC)
{
	gpuErrchk(cudaPeekAtLastError());

	unsigned int node_num = O_R.rows();

	std::cout << "node_num: " << node_num << std::endl;

	this->m_Node_Size = node_num;
	cudaMallocManaged((char**)&this->m_Octree_GPU, node_num * sizeof(*this->m_Octree_GPU));

	OctreeNodeLibiGPU* octree_cpu = new OctreeNodeLibiGPU[node_num];

	int child_num = 8;

	uint32_t num_indices_to_copy = 0;
	std::vector<unsigned int> whose_indices_to_copy;

	for (int i = 0; i < O_R.rows(); i++)
	{
		octree_cpu[i].Mass_Center.x = O_CM(i, 0);
		octree_cpu[i].Mass_Center.y = O_CM(i, 1);
		octree_cpu[i].Mass_Center.z = O_CM(i, 2);

		octree_cpu[i].Mass_Normal.x = O_EC(i, 0);
		octree_cpu[i].Mass_Normal.y = O_EC(i, 1);
		octree_cpu[i].Mass_Normal.z = O_EC(i, 2);

		octree_cpu[i].Max_R = O_R[i];
	}

	octree_cpu[0].Father = -1;
	for (int i = 0; i < O_CH.rows(); i++)
	{
		for (int j = 0; j < child_num; j++)
		{
			int c_ci = O_CH(i, j);

			if (c_ci != -1)
			{
				octree_cpu[c_ci].Father = i;
				octree_cpu[c_ci].IndexInFather = j;
			}
		}

		octree_cpu[i].Point_Cloud_Num = O_PI[i].size();

		if (octree_cpu[i].Point_Cloud_Num != 0)
		{
			whose_indices_to_copy.push_back(i);
			num_indices_to_copy += octree_cpu[i].Point_Cloud_Num;
		}
	}

	if (num_indices_to_copy > 0)
	{
		unsigned int* point_indices = nullptr;
		cudaMallocManaged((char**)&point_indices, num_indices_to_copy * sizeof(*point_indices));

		uint32_t offset = 0;
		for (unsigned int i : whose_indices_to_copy)
		{
			octree_cpu[i].Point_Cloud_Indices = point_indices + offset;
			offset += octree_cpu[i].Point_Cloud_Num;
		}

		std::unique_ptr<unsigned int[]> idxCPU = std::make_unique<unsigned int[]>(num_indices_to_copy * sizeof(unsigned int));
		unsigned int* idx = idxCPU.get();
		for (unsigned int i : whose_indices_to_copy)
		{
			for (unsigned int j = 0; j < octree_cpu[i].Point_Cloud_Num; j++)
				*(idx++) = O_PI[i][j];
		}
		gpuErrchk(cudaMemcpy(point_indices, idxCPU.get(), num_indices_to_copy * sizeof(unsigned int), cudaMemcpyHostToDevice));
	}

	int* node_indices = nullptr;
	int total_node_num = ((int)O_CH.rows()) * child_num;
	cudaMallocManaged((char**)&node_indices, total_node_num * sizeof(*node_indices));

	uint32_t offset = 0;
	for (int i = 0; i < O_CH.rows(); i++)
	{
		octree_cpu[i].Children = node_indices + offset;
		offset += child_num;
	}

	std::unique_ptr<int[]> idxCPU = std::make_unique<int[]>(total_node_num * sizeof(int));
	int* idx = idxCPU.get();
	for (int i = 0; i < O_CH.rows(); i++)
	{
		for (int j = 0; j < 8; j++)
			*(idx++) = O_CH(i, j);
	}
	gpuErrchk(cudaMemcpy(node_indices, idxCPU.get(), total_node_num * sizeof(int), cudaMemcpyHostToDevice));

	gpuErrchk(cudaMemcpy(this->m_Octree_GPU, octree_cpu, node_num * sizeof(OctreeNodeLibiGPU), cudaMemcpyHostToDevice));

	this->m_Octree_CPU = octree_cpu;
}

void TurboLibiGWN::CopyToGPU(std::vector<std::vector<int>>& O_PI, Eigen::MatrixXi& O_CH, Eigen::MatrixXd& O_CM, Eigen::VectorXd& O_R, Eigen::MatrixXd& O_EC, std::vector<geometrycentral::Vector3>& point_cloud)
{
	int child_num = 8;

	uint32_t num_indices_to_copy = 0;
	std::vector<unsigned int> whose_indices_to_copy;

	for (int i = 0; i < O_R.rows(); i++)
	{
		this->m_Octree_GPU[i].Mass_Center.x = O_CM(i, 0);
		this->m_Octree_GPU[i].Mass_Center.y = O_CM(i, 1);
		this->m_Octree_GPU[i].Mass_Center.z = O_CM(i, 2);

		this->m_Octree_GPU[i].Mass_Normal.x = O_EC(i, 0);
		this->m_Octree_GPU[i].Mass_Normal.y = O_EC(i, 1);
		this->m_Octree_GPU[i].Mass_Normal.z = O_EC(i, 2);

		this->m_Octree_GPU[i].Max_R = O_R[i];
	}

	this->m_Octree_GPU[0].Father = -1;
	for (int i = 0; i < O_CH.rows(); i++)
	{
		for (int j = 0; j < child_num; j++)
		{
			int c_ci = O_CH(i, j);

			if (c_ci != -1)
			{
				this->m_Octree_GPU[c_ci].Father = i;
				this->m_Octree_GPU[c_ci].IndexInFather = j;
			}
		}

		this->m_Octree_GPU[i].Point_Cloud_Num = O_PI[i].size();

		if (this->m_Octree_GPU[i].Point_Cloud_Num != 0)
		{
			whose_indices_to_copy.push_back(i);
			num_indices_to_copy += this->m_Octree_GPU[i].Point_Cloud_Num;
		}
	}

	if (num_indices_to_copy > 0)
	{
		unsigned int* point_indices = nullptr;
		cudaMallocManaged((char**)&point_indices, num_indices_to_copy * sizeof(*point_indices));

		uint32_t offset = 0;
		for (unsigned int i : whose_indices_to_copy)
		{
			this->m_Octree_GPU[i].Point_Cloud_Indices = point_indices + offset;
			offset += this->m_Octree_GPU[i].Point_Cloud_Num;
		}

		std::unique_ptr<unsigned int[]> idxCPU = std::make_unique<unsigned int[]>(num_indices_to_copy * sizeof(unsigned int));
		unsigned int* idx = idxCPU.get();
		for (unsigned int i : whose_indices_to_copy)
		{
			for (unsigned int j = 0; j < this->m_Octree_GPU[i].Point_Cloud_Num; j++)
				*(idx++) = O_PI[i][j];
		}
		gpuErrchk(cudaMemcpy(point_indices, idxCPU.get(), num_indices_to_copy * sizeof(unsigned int), cudaMemcpyHostToDevice));
	}

	int* node_indices = nullptr;
	int total_node_num = ((int)O_CH.rows()) * child_num;
	cudaMallocManaged((char**)&node_indices, total_node_num * sizeof(*node_indices));

	uint32_t offset = 0;
	for (int i = 0; i < O_CH.rows(); i++)
	{
		this->m_Octree_GPU[i].Children = node_indices + offset;
		offset += child_num;
	}

	std::unique_ptr<int[]> idxCPU = std::make_unique<int[]>(total_node_num * sizeof(int));
	int* idx = idxCPU.get();
	for (int i = 0; i < O_CH.rows(); i++)
	{
		for (int j = 0; j < 8; j++)
			*(idx++) = O_CH(i, j);
	}
	gpuErrchk(cudaMemcpy(node_indices, idxCPU.get(), total_node_num * sizeof(int), cudaMemcpyHostToDevice));

	int point_cloud_size = point_cloud.size();
	cudaMallocManaged((char**)&this->m_Point_Cloud_GPU, point_cloud_size * sizeof(*this->m_Point_Cloud_GPU));
	std::unique_ptr<float3[]> point_cloud_cpu = std::make_unique<float3[]>(size_t(point_cloud_size));
	for (int i = 0; i < point_cloud_size; i++)
	{
		point_cloud_cpu[i] = make_float3(point_cloud[i].x, point_cloud[i].y, point_cloud[i].z);
	}
	gpuErrchk(cudaMemcpy(this->m_Point_Cloud_GPU, point_cloud_cpu.get(), point_cloud_size * sizeof(float3), cudaMemcpyHostToDevice));
}

__device__
float GWNLibiPStoQ(OctreeNodeLibiGPU* root_node, float3& q, float3* i_points, float3* i_Normals, float* i_As, float beta)
{
	float gwn = 0.0;
	bool continue_traversal = true;

	int iter_num = 0;
	int points_num = 0;

	int traversal_index = 0;

	int current_node_index = 0;

	while (continue_traversal)
	{
		if (root_node[current_node_index].Children[0] == -1)
		{
			for (int i_point_in_cn = 0; i_point_in_cn < root_node[current_node_index].Point_Cloud_Num; i_point_in_cn++)
			{
				int p_idx = root_node[current_node_index].Point_Cloud_Indices[i_point_in_cn];
				float3& p = i_points[p_idx];
				float3& N = i_Normals[p_idx];
				float _A = i_As[p_idx];

				float3 _R;
				_R.x = p.x - q.x; _R.y = p.y - q.y; _R.z = p.z - q.z;
				float _R_norm = sqrtf(_R.x * _R.x + _R.y * _R.y + _R.z * _R.z);
				float dot_RN = _R.x * N.x + _R.y * N.y + _R.z * N.z;
				dot_RN *= _A;
				dot_RN /= (4.0 * 3.14159265358979323 * _R_norm * _R_norm * _R_norm + 1e-8);
				gwn += dot_RN;

				//out_gwn[points_num] = gwn;

				points_num++;
			}

			while (true)
			{
				bool is_last_child = true;
				//int next_node = -1;
				if (root_node[current_node_index].IndexInFather == 7)
					is_last_child = true;
				else if (root_node[current_node_index].Father == -1)
				{
					continue_traversal = false;
					break;
				}
				else
				{
					traversal_index = root_node[current_node_index].IndexInFather + 1;
					current_node_index = root_node[current_node_index].Father;
					break;
				}
				if (root_node[current_node_index].Father != -1 && is_last_child)
				{
					current_node_index = root_node[current_node_index].Father;

					if (root_node[current_node_index].Father == -1)
					{
						continue_traversal = false;
						break;
					}
				}
				else
				{
					break;
				}
			}

		}
		else
		{
			//if (traversal_index == 0)
				//out_indices[iter_num] = current_node->Breadth_First_Index;

			bool descend = false;
			for (int c = traversal_index; c < 8; c++)
			{
				int cn_child = root_node[current_node_index].Children[c];
				if (root_node[cn_child].Point_Cloud_Num > 0)
				{
					float3& _massCenter = root_node[cn_child].Mass_Center;
					float _max_R = root_node[cn_child].Max_R;
					float3 MCToq;
					MCToq.x = _massCenter.x - q.x; MCToq.y = _massCenter.y - q.y; MCToq.z = _massCenter.z - q.z;
					float mctop_norm = sqrtf(MCToq.x * MCToq.x + MCToq.y * MCToq.y + MCToq.z * MCToq.z);

					if (mctop_norm > beta * _max_R)
					{
						if (root_node[cn_child].Children[0] == -1)
						{
							for (int i_point_in_cnc = 0; i_point_in_cnc < root_node[cn_child].Point_Cloud_Num; i_point_in_cnc++)
							{
								int p_idx = root_node[cn_child].Point_Cloud_Indices[i_point_in_cnc];
								float3& p = i_points[p_idx];
								float3& N = i_Normals[p_idx];
								float _A = i_As[p_idx];

								float3 _R;
								_R.x = p.x - q.x; _R.y = p.y - q.y; _R.z = p.z - q.z;
								float _R_norm = sqrtf(_R.x * _R.x + _R.y * _R.y + _R.z * _R.z);
								float dot_RN = _R.x * N.x + _R.y * N.y + _R.z * N.z;
								dot_RN *= _A;
								dot_RN /= (4.0 * 3.14159265358979323 * _R_norm * _R_norm * _R_norm + 1e-8);
								gwn += dot_RN;

								//out_gwn[points_num] = gwn;

								points_num++;
							}
						}
						else
						{
							float3& _massNormal = root_node[cn_child].Mass_Normal;

							//float3 _R;
							//_R.x = _massCenter.x - q.x; _R.y = _massCenter.y - q.y; _R.z = _massCenter.z - q.z;
							//float _R_norm = sqrtf(_R.x * _R.x + _R.y * _R.y + _R.z * _R.z);
							//float dot_RN = _R.x * _massNormal.x + _R.y * _massNormal.y + _R.z * _massNormal.z;
							float dot_RN = MCToq.x * _massNormal.x + MCToq.y * _massNormal.y + MCToq.z * _massNormal.z;

							//dot_RN /= (4.0 * 3.14159265358979323 * _R_norm * _R_norm * _R_norm + 1e-8);
							dot_RN /= (4.0 * 3.14159265358979323 * mctop_norm * mctop_norm * mctop_norm + 1e-8);
							gwn += dot_RN;

							//out_gwn[points_num] = gwn;

							points_num++;
						}
					}
					else
					{
						current_node_index = cn_child;
						descend = true;
						break;
					}
				}
			}

			traversal_index = 0;

			if (!descend)
			{
				while (true)
				{
					bool is_last_child = true;
					//int next_node = -1;
					if (root_node[current_node_index].IndexInFather == 7)
						is_last_child = true;
					else if (root_node[current_node_index].Father == -1)
					{
						continue_traversal = false;
						break;
					}
					else
					{
						traversal_index = root_node[current_node_index].IndexInFather + 1;
						current_node_index = root_node[current_node_index].Father;
						break;
					}
					if (root_node[current_node_index].Father != -1 && is_last_child)
					{
						current_node_index = root_node[current_node_index].Father;

						if (root_node[current_node_index].Father == -1)
						{
							continue_traversal = false;
							break;
						}
					}
					else
					{
						break;
					}
				}
			}
		}

		iter_num++;
	}

	return gwn;
}

__device__
float ScreenedGWNLibiPStoQ(OctreeNodeLibiGPU* __restrict__ root_node, float3& q, float3* __restrict__ i_points, float3* __restrict__ i_Normals, float* __restrict__ i_As, float* __restrict__ i_radii, float beta, float sigma, bool use_radii)
{
	float gwn = 0.0;
	bool continue_traversal = true;

	int iter_num = 0;
	int points_num = 0;

	int traversal_index = 0;

	int current_node_index = 0;

	float PI_4 = 4.0f * 3.14159265358979323;

	while (continue_traversal)
	{
		if (root_node[current_node_index].Children[0] == -1)
		{
			//out_indices[iter_num] = current_node->Breadth_First_Index;
			for (int i_point_in_cn = 0; i_point_in_cn < root_node[current_node_index].Point_Cloud_Num; i_point_in_cn++)
			{
				int p_idx = root_node[current_node_index].Point_Cloud_Indices[i_point_in_cn];
				float3& p = i_points[p_idx];
				float3& N = i_Normals[p_idx];
				float _A = i_As[p_idx];

				float3 _R;
				_R.x = p.x - q.x; _R.y = p.y - q.y; _R.z = p.z - q.z;
				float _R_norm = sqrtf(_R.x * _R.x + _R.y * _R.y + _R.z * _R.z);

				if (_R_norm > 1e-10)
				{
					float dot_RN = _R.x * N.x + _R.y * N.y + _R.z * N.z;
					dot_RN *= _A;

					float screened = (use_radii) ? i_radii[p_idx] : 0.0015f;
					bool sign = (dot_RN > 0.0f) ? true : false;

					if (screened > _R_norm)
					{
						float up = (screened * sigma + 1.0f) * dot_RN;
						float down = expf(screened * sigma) * (PI_4 * screened * screened * screened) + 1e-7;
						float res = up / down;

						if (sign && res < 0)
							gwn += (-1.0f * res);
						else if (!sign && res > 0)
						{
							gwn += (-1.0f * res);
						}
						else
						{
							gwn += res;
						}
					}
					else
					{
						float up = (_R_norm * sigma + 1.0f) * dot_RN;
						float down = expf(_R_norm * sigma) * (PI_4 * _R_norm * _R_norm * _R_norm) + 1e-7;
						float res = up / down;

						if (sign && res < 0)
							gwn += (-1.0f * res);
						else if (!sign && res > 0)
						{
							gwn += (-1.0f * res);
						}
						else
						{
							gwn += res;
						}
					}
				}

				//out_gwn[points_num] = gwn;

				points_num++;
			}

			while (true)
			{
				bool is_last_child = true;
				//int next_node = -1;
				if (root_node[current_node_index].IndexInFather == 7)
					is_last_child = true;
				else if (root_node[current_node_index].Father == -1)
				{
					continue_traversal = false;
					break;
				}
				else
				{
					traversal_index = root_node[current_node_index].IndexInFather + 1;
					current_node_index = root_node[current_node_index].Father;
					break;
				}
				if (root_node[current_node_index].Father != -1 && is_last_child)
				{
					current_node_index = root_node[current_node_index].Father;

					if (root_node[current_node_index].Father == -1)
					{
						continue_traversal = false;
						break;
					}
				}
				else
				{
					break;
				}
			}

		}
		else
		{
			//if (traversal_index == 0)
				//out_indices[iter_num] = current_node->Breadth_First_Index;

			bool descend = false;
			for (int c = traversal_index; c < 8; c++)
			{
				int cn_child = root_node[current_node_index].Children[c];
				if (root_node[cn_child].Point_Cloud_Num > 0)
				{
					float3& _massCenter = root_node[cn_child].Mass_Center;
					float _max_R = root_node[cn_child].Max_R;
					float3 MCToq;
					MCToq.x = _massCenter.x - q.x; MCToq.y = _massCenter.y - q.y; MCToq.z = _massCenter.z - q.z;
					float mctop_norm = sqrtf(MCToq.x * MCToq.x + MCToq.y * MCToq.y + MCToq.z * MCToq.z);

					if (mctop_norm > beta * _max_R)
					{
						if (root_node[cn_child].Children[0] == -1)
						{
							for (int i_point_in_cnc = 0; i_point_in_cnc < root_node[cn_child].Point_Cloud_Num; i_point_in_cnc++)
							{
								int p_idx = root_node[cn_child].Point_Cloud_Indices[i_point_in_cnc];
								float3& p = i_points[p_idx];
								float3& N = i_Normals[p_idx];
								float _A = i_As[p_idx];

								float3 _R;
								_R.x = p.x - q.x; _R.y = p.y - q.y; _R.z = p.z - q.z;
								float _R_norm = sqrtf(_R.x * _R.x + _R.y * _R.y + _R.z * _R.z);

								if (_R_norm > 1e-10)
								{
									float dot_RN = _R.x * N.x + _R.y * N.y + _R.z * N.z;
									dot_RN *= _A;

									float screened = (use_radii) ? i_radii[p_idx] : 0.0015f;
									bool sign = (dot_RN > 0.0f) ? true : false;

									if (screened > _R_norm)
									{
										float up = (screened * sigma + 1.0f) * dot_RN;
										float down = expf(screened * sigma) * (PI_4 * screened * screened * screened) + 1e-7;
										float res = up / down;

										if (sign && res < 0)
											gwn += (-1.0f * res);
										else if (!sign && res > 0)
										{
											gwn += (-1.0f * res);
										}
										else
										{
											gwn += res;
										}
									}
									else
									{
										float up = (_R_norm * sigma + 1.0f) * dot_RN;
										float down = expf(_R_norm * sigma) * (PI_4 * _R_norm * _R_norm * _R_norm) + 1e-7;
										float res = up / down;

										if (sign && res < 0)
											gwn += (-1.0f * res);
										else if (!sign && res > 0)
										{
											gwn += (-1.0f * res);
										}
										else
										{
											gwn += res;
										}
									}
								}

								points_num++;
							}
						}
						else
						{
							float3& _massNormal = root_node[cn_child].Mass_Normal;

							float _R_norm = mctop_norm;
							if (_R_norm > 1e-10)
							{
								float dot_RN = MCToq.x * _massNormal.x + MCToq.y * _massNormal.y + MCToq.z * _massNormal.z;

								float screened = 0.0015f;
								bool sign = (dot_RN > 0.0f) ? true : false;

								if (screened > _R_norm)
								{
									float up = (screened * sigma + 1.0f) * dot_RN;
									float down = expf(screened * sigma) * (PI_4 * screened * screened * screened) + 1e-7;
									float res = up / down;

									if (sign && res < 0)
										gwn += (-1.0f * res);
									else if (!sign && res > 0)
									{
										gwn += (-1.0f * res);
									}
									else
									{
										gwn += res;
									}
								}
								else
								{
									float up = (_R_norm * sigma + 1.0f) * dot_RN;
									float down = expf(_R_norm * sigma) * (PI_4 * _R_norm * _R_norm * _R_norm) + 1e-7;
									float res = up / down;

									if (sign && res < 0)
										gwn += (-1.0f * res);
									else if (!sign && res > 0)
									{
										gwn += (-1.0f * res);
									}
									else
									{
										gwn += res;
									}
								}
							}
							//out_gwn[points_num] = gwn;

							points_num++;
						}
					}
					else
					{
						current_node_index = cn_child;
						descend = true;
						break;
					}
				}
			}

			traversal_index = 0;

			if (!descend)
			{
				while (true)
				{
					bool is_last_child = true;
					//int next_node = -1;
					if (root_node[current_node_index].IndexInFather == 7)
						is_last_child = true;
					else if (root_node[current_node_index].Father == -1)
					{
						continue_traversal = false;
						break;
					}
					else
					{
						traversal_index = root_node[current_node_index].IndexInFather + 1;
						current_node_index = root_node[current_node_index].Father;
						break;
					}
					if (root_node[current_node_index].Father != -1 && is_last_child)
					{
						current_node_index = root_node[current_node_index].Father;

						if (root_node[current_node_index].Father == -1)
						{
							continue_traversal = false;
							break;
						}
					}
					else
					{
						break;
					}
				}
			}
		}

		iter_num++;
	}

	//printf("points_num: %d\n", points_num);

	return gwn;
}

__global__
void TurboLibiGWNKernal(int query_size, OctreeNodeLibiGPU* root_node, float3* q, float3* i_points, float3* i_Normals, float* i_As, float beta, float* gwn_results)
{
	int tid = threadIdx.x + blockIdx.x * blockDim.x;
	int stride = blockDim.x * gridDim.x;

	for (int tid_i = tid; tid_i < query_size; tid_i += stride)
	{
		gwn_results[tid_i] = GWNLibiPStoQ(root_node, q[tid_i], i_points, i_Normals, i_As, beta);
	}
}

__global__
void TurboLibiScreenedGWNKernal(int query_size, OctreeNodeLibiGPU* root_node, float3* q, float3* i_points, float3* i_Normals, float* i_As, float* i_radii, float beta, float sigma, float* gwn_results, bool use_radii)
{
	int tid = threadIdx.x + blockIdx.x * blockDim.x;
	int stride = blockDim.x * gridDim.x;

	for (int tid_i = tid; tid_i < query_size; tid_i += stride)
	{
		gwn_results[tid_i] = ScreenedGWNLibiPStoQ(root_node, q[tid_i], i_points, i_Normals, i_As, i_radii, beta, sigma, use_radii);
	}
}

void TurboLibiGWN::TurboLibiGWNLaunch(std::vector<geometrycentral::Vector3>& q, std::vector<geometrycentral::Vector3>& i_Normals, std::vector<float>& i_As, float& beta, float* out_gwn)
{
	float* gwn_results = 0;
	int query_size = q.size();
	cudaMallocManaged((char**)&gwn_results, query_size * sizeof(*gwn_results));

	float3* q_gpu = 0;
	std::cout << "query_size: " << query_size << std::endl;
	cudaMallocManaged((char**)&q_gpu, query_size * sizeof(*q_gpu));
	std::unique_ptr<float3[]> q_cpu = std::make_unique<float3[]>(size_t(query_size));
	for (int i = 0; i < query_size; i++)
	{
		q_cpu[i] = make_float3(q[i].x, q[i].y, q[i].z);
	}
	gpuErrchk(cudaMemcpy(q_gpu, q_cpu.get(), query_size * sizeof(float3), cudaMemcpyHostToDevice));

	float3* i_Normals_gpu = 0;
	int i_Normals_size = i_Normals.size();
	cudaMallocManaged((char**)&i_Normals_gpu, i_Normals_size * sizeof(*i_Normals_gpu));
	std::unique_ptr<float3[]> i_Normals_cpu = std::make_unique<float3[]>(size_t(i_Normals_size));
	for (int i = 0; i < i_Normals_size; i++)
	{
		i_Normals_cpu[i] = make_float3(i_Normals[i].x, i_Normals[i].y, i_Normals[i].z);
	}
	gpuErrchk(cudaMemcpy(i_Normals_gpu, i_Normals_cpu.get(), i_Normals_size * sizeof(float3), cudaMemcpyHostToDevice));

	float* i_As_gpu = 0;
	int i_As_size = i_As.size();
	cudaMallocManaged((char**)&i_As_gpu, i_As_size * sizeof(*i_As_gpu));
	std::unique_ptr<float[]> i_As_cpu = std::make_unique<float[]>(size_t(i_As_size));
	for (int i = 0; i < i_As_size; i++)
	{
		i_As_cpu[i] = i_As[i];
	}
	gpuErrchk(cudaMemcpy(i_As_gpu, i_As_cpu.get(), i_As_size * sizeof(float), cudaMemcpyHostToDevice));

	int blockSize = 128;
	dim3 numBlocks = computeNbBlocks((unsigned int)query_size, blockSize);

	TurboLibiGWNKernal << <numBlocks, blockSize >> > ((int)query_size, this->m_Octree_GPU, q_gpu, this->m_Point_Cloud_GPU, i_Normals_gpu, i_As_gpu, beta, gwn_results);

	cudaDeviceSynchronize();

	gpuErrchk(cudaMemcpy(out_gwn, gwn_results, query_size * sizeof(float), cudaMemcpyDeviceToHost));

	gpuErrchk(cudaFree(gwn_results));
	gwn_results = nullptr;

	gpuErrchk(cudaFree(q_gpu));
	q_gpu = nullptr;

	gpuErrchk(cudaFree(i_Normals_gpu));
	i_Normals_gpu = nullptr;

	gpuErrchk(cudaFree(i_As_gpu));
	i_As_gpu = nullptr;

	std::cout << "GWN Done" << std::endl;
}

void TurboLibiGWN::TurboLibiGWNLaunch(float3* query_points, float3* point_cloud, int query_size, float3* i_Normals, float* i_As, float& beta, float* out_gwn)
{
	int blockSize = 128;
	dim3 numBlocks = computeNbBlocks((unsigned int)query_size, blockSize);

	TurboLibiGWNKernal << <numBlocks, blockSize >> > (query_size, this->m_Octree_GPU, query_points, point_cloud, i_Normals, i_As, beta, out_gwn);

	cudaDeviceSynchronize();

	std::cout << "out_gwn[666] " << out_gwn[666] << std::endl;

	std::cout << "GWN Done" << std::endl;
}

void TurboLibiGWN::TurboLibiScreenedGWNLaunch(float3* query_points, float3* point_cloud, int query_size, float3* i_Normals, float* i_As, float* i_radii, float& beta, float& sigma, float* out_gwn, bool use_radii)
{
	int blockSize = 128;
	dim3 numBlocks = computeNbBlocks((unsigned int)query_size, blockSize);

	TurboLibiScreenedGWNKernal << <numBlocks, blockSize >> > (query_size, this->m_Octree_GPU, query_points, point_cloud, i_Normals, i_As, i_radii, beta, sigma, out_gwn, use_radii);

	cudaDeviceSynchronize();

	std::cout << "GWN Done" << std::endl;
}

__device__
float3 CalMassNormal(OctreeNodeLibiGPU* node, float3* i_Normals, float* i_As)
{
	float3 new_mass_normal;
	new_mass_normal.x = 0.0;
	new_mass_normal.y = 0.0;
	new_mass_normal.z = 0.0;
	for (int i = 0; i < node->Point_Cloud_Num; i++)
	{
		int curr_point_index = node->Point_Cloud_Indices[i];
		new_mass_normal.x += i_As[curr_point_index] * i_Normals[curr_point_index].x;
		new_mass_normal.y += i_As[curr_point_index] * i_Normals[curr_point_index].y;
		new_mass_normal.z += i_As[curr_point_index] * i_Normals[curr_point_index].z;
	}

	return new_mass_normal;
}

__global__
void UpdateMassNormalsKernal(int nodes_size, OctreeNodeLibiGPU* root_node, float3* i_Normals, float* i_As)
{
	int tid = threadIdx.x + blockIdx.x * blockDim.x;
	int stride = blockDim.x * gridDim.x;

	for (int tid_i = tid; tid_i < nodes_size; tid_i += stride)
	{
		root_node[tid_i].Mass_Normal = CalMassNormal(root_node + tid_i, i_Normals, i_As);
	}
}

__global__
void UpdateGWNInfoKernal(int nodes_size, OctreeNodeLibiGPU* root_node, float3* i_Normals, float3* point_cloud, float* i_As)
{
	int tid = threadIdx.x + blockIdx.x * blockDim.x;
	int stride = blockDim.x * gridDim.x;

	for (int tid_i = tid; tid_i < nodes_size; tid_i += stride)
	{
		OctreeNodeLibiGPU* node = root_node + tid_i;

		float3 new_mass_normal;
		new_mass_normal.x = 0.0;
		new_mass_normal.y = 0.0;
		new_mass_normal.z = 0.0;

		float3 new_mass_center;
		new_mass_center.x = 0.0;
		new_mass_center.y = 0.0;
		new_mass_center.z = 0.0;

		float areatotal = 0.0;

		for (int i = 0; i < node->Point_Cloud_Num; i++)
		{
			int curr_point_index = node->Point_Cloud_Indices[i];
			new_mass_normal.x += i_As[curr_point_index] * i_Normals[curr_point_index].x;
			new_mass_normal.y += i_As[curr_point_index] * i_Normals[curr_point_index].y;
			new_mass_normal.z += i_As[curr_point_index] * i_Normals[curr_point_index].z;

			areatotal += i_As[curr_point_index];
			new_mass_center.x += i_As[curr_point_index] * point_cloud[curr_point_index].x;
			new_mass_center.y += i_As[curr_point_index] * point_cloud[curr_point_index].y;
			new_mass_center.z += i_As[curr_point_index] * point_cloud[curr_point_index].z;
		}

		new_mass_center.x = new_mass_center.x / areatotal;
		new_mass_center.y = new_mass_center.y / areatotal;
		new_mass_center.z = new_mass_center.z / areatotal;

		float c_norm = 0;
		float max_norm = 0;
		for (int i = 0; i < node->Point_Cloud_Num; i++)
		{
			int curr_point_index = node->Point_Cloud_Indices[i];
			float3 point_diff;
			point_diff.x = point_cloud[curr_point_index].x - new_mass_center.x;
			point_diff.y = point_cloud[curr_point_index].y - new_mass_center.y;
			point_diff.z = point_cloud[curr_point_index].z - new_mass_center.z;

			c_norm = __fsqrt_rn(point_diff.x * point_diff.x +
				point_diff.y * point_diff.y +
				point_diff.z * point_diff.z);

			if (c_norm > max_norm)
			{
				max_norm = c_norm;
			}
		}

		root_node[tid_i].Mass_Normal = new_mass_normal;
		root_node[tid_i].Mass_Center = new_mass_center;
		root_node[tid_i].Max_R = max_norm;
	}
}

void TurboLibiGWN::UpdateMassNormals(std::vector<geometrycentral::Vector3>& i_Normals, std::vector<float>& i_As)
{
	float3* i_Normals_gpu = 0;
	int i_Normals_size = i_Normals.size();
	//gpuErrchk(cudaMalloc(&i_Normals_gpu, i_Normals_size * sizeof(float3)));
	cudaMallocManaged((char**)&i_Normals_gpu, i_Normals_size * sizeof(*i_Normals_gpu));
	std::unique_ptr<float3[]> i_Normals_cpu = std::make_unique<float3[]>(size_t(i_Normals_size));
	for (int i = 0; i < i_Normals_size; i++)
	{
		i_Normals_cpu[i] = make_float3(i_Normals[i].x, i_Normals[i].y, i_Normals[i].z);
	}
	gpuErrchk(cudaMemcpy(i_Normals_gpu, i_Normals_cpu.get(), i_Normals_size * sizeof(float3), cudaMemcpyHostToDevice));

	float* i_As_gpu = 0;
	int i_As_size = i_As.size();
	//gpuErrchk(cudaMalloc(&i_As_gpu, i_As_size * sizeof(float)));
	cudaMallocManaged((char**)&i_As_gpu, i_As_size * sizeof(*i_As_gpu));
	std::unique_ptr<float[]> i_As_cpu = std::make_unique<float[]>(size_t(i_As_size));
	for (int i = 0; i < i_As_size; i++)
	{
		i_As_cpu[i] = i_As[i];
	}
	gpuErrchk(cudaMemcpy(i_As_gpu, i_As_cpu.get(), i_As_size * sizeof(float), cudaMemcpyHostToDevice));

	int blockSize = 128;
	dim3 numBlocks = computeNbBlocks((unsigned int)this->m_Node_Size, blockSize);

	UpdateMassNormalsKernal << <numBlocks, blockSize >> > (this->m_Node_Size, this->m_Octree_GPU, i_Normals_gpu, i_As_gpu);

	cudaDeviceSynchronize();
}

void TurboLibiGWN::UpdateMassNormals(float3* i_Normals, float* i_As)
{
	int blockSize = 128;
	dim3 numBlocks = computeNbBlocks((unsigned int)this->m_Node_Size, blockSize);

	UpdateMassNormalsKernal << <numBlocks, blockSize >> > (this->m_Node_Size, this->m_Octree_GPU, i_Normals, i_As);

	cudaDeviceSynchronize();
}

void TurboLibiGWN::UpdateGWNInfo(float3* i_Normals, float3* point_cloud, float* i_As)
{
	int blockSize = 128;
	dim3 numBlocks = computeNbBlocks((unsigned int)this->m_Node_Size, blockSize);

	UpdateGWNInfoKernal << <numBlocks, blockSize >> > (this->m_Node_Size, this->m_Octree_GPU, i_Normals, point_cloud, i_As);

	cudaDeviceSynchronize();
}

void TurboLibiGWN::EarseMemory()
{
	for (int i = 0; i < m_Node_Size; i++)
	{
		if (m_Octree_CPU[i].Point_Cloud_Num > 0)
		{
			gpuErrchk(cudaFree(m_Octree_CPU[i].Point_Cloud_Indices));
			break;
		}
	}

	gpuErrchk(cudaFree(m_Octree_CPU[0].Children));

	delete[] m_Octree_CPU;
	m_Octree_CPU = nullptr;
	gpuErrchk(cudaFree(m_Octree_GPU));
	m_Octree_GPU = nullptr;
}