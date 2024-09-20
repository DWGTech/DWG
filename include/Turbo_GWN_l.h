#pragma once
#include <cuda.h>
#include <cuda_runtime.h>

#include <device_launch_parameters.h>

#include <Eigen/Core>

#include "geometrycentral/vector3.h"

struct OctreeNodeLibiGPU
{
	int* Children = nullptr;
	int Father = -1;
	int IndexInFather = -1;
	unsigned int* Point_Cloud_Indices = nullptr;
	unsigned int Point_Cloud_Num = 0;

	float3 Mass_Center;
	float3 Mass_Normal;
	float Max_R;
};

class TurboLibiGWN
{
public:
	TurboLibiGWN(std::vector<std::vector<int>>& O_PI, Eigen::MatrixXi& O_CH, Eigen::MatrixXd& O_CM, Eigen::VectorXd& O_R, Eigen::MatrixXd& O_EC, std::vector<geometrycentral::Vector3>& point_cloud);
	TurboLibiGWN(std::vector<std::vector<int>>& O_PI, Eigen::MatrixXi& O_CH, Eigen::MatrixXd& O_CM, Eigen::VectorXd& O_R, Eigen::MatrixXd& O_EC);
	TurboLibiGWN() = default;
	void PreLibiGWN(std::vector<std::vector<int>>& O_PI, Eigen::MatrixXi& O_CH, Eigen::MatrixXd& O_CM, Eigen::VectorXd& O_R, Eigen::MatrixXd& O_EC);

	void TurboLibiGWNLaunch(std::vector<geometrycentral::Vector3>& q,
		std::vector<geometrycentral::Vector3>& i_Normals,
		std::vector<float>& i_As,
		float& beta,
		float* out_gwn);

	void TurboLibiGWNLaunch(float3* query_points,
		float3* point_cloud,
		int query_size,
		float3* i_Normals,
		float* i_As,
		float& beta,
		float* out_gwn);

	void TurboLibiScreenedGWNLaunch(float3* query_points,
		float3* point_cloud,
		int query_size,
		float3* i_Normals,
		float* i_As,
		float* i_radii,
		float& beta,
		float& sigma,
		float* out_gwn,
		bool use_radii);

	void UpdateMassNormals(std::vector<geometrycentral::Vector3>& i_Normals, std::vector<float>& i_As);
	void UpdateMassNormals(float3* i_Normals, float* i_As);
	void UpdateGWNInfo(float3* i_Normals, float3* point_cloud, float* i_As);
	void EarseMemory();
private:
	OctreeNodeLibiGPU* m_Octree_GPU = 0;
	float3* m_Point_Cloud_GPU = 0;
	OctreeNodeLibiGPU* m_Octree_CPU = 0;
	int m_Node_Size = 0;


	void CopyToGPU(std::vector<std::vector<int>>& O_PI, Eigen::MatrixXi& O_CH, Eigen::MatrixXd& O_CM, Eigen::VectorXd& O_R, Eigen::MatrixXd& O_EC, std::vector<geometrycentral::Vector3>& point_cloud);
};
