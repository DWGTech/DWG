#include <thrust/reduce.h>
#include <thrust/device_ptr.h>
#include <thrust/extrema.h>
#include <thrust/device_vector.h>
#include <unordered_map>
#include <set>
#include <string>

#include "Turbo_GWN_l.h"
#include "MarchingCubeCuda.h"
#include "TurboNormalInit.h"
//#include "TurboKNN.h"

void GenerateKDTreeForDWG(
	float3* d_points,
	float3* d_normals,
	int points_size,
	float3* bound_upper, float3* bound_lower);

void CudaKNNWithKDTreeforDWG(
	float3* d_points,
	int numPoints,
	float3* d_queries,
	int numQueries,
	float3* bound_upper, float3* bound_lower,
	int* d_results,
	int k);

class TurboDWG
{
public:
	TurboDWG(Eigen::MatrixXf& octree_leaves, Eigen::MatrixXf& leaves_Ns, Eigen::VectorXf& leaves_As, Eigen::VectorXf radii, Eigen::MatrixXf& grid_points, int res_x, int res_y, int res_z, std::vector<float> bound_upper_kdtree, std::vector<float> bound_lower_kdtree);
	void PreTurboDWG(int point_cloud_size, Eigen::VectorXf& leaves_As, Eigen::MatrixXf& grid_points, int res_x, int res_y, int res_z);
	clock_t TurboScreenedDWGLaunch(std::vector<std::vector<int>>& O_PI, Eigen::MatrixXi& O_CH, Eigen::MatrixXd& O_CM, Eigen::VectorXd& O_R, Eigen::MatrixXd& O_EC, float beta, float sigma, int total_iters, bool use_radii, float As_threshold, int input_normal, clock_t start_o, std::string mesh_path, int test_iter);
	void UpdateRadii(int k_size);
	void UpdateAs(Eigen::VectorXd& leaves_As);
	void PreGeneration(Eigen::MatrixXf& cloud_points, Eigen::VectorXf& points_As, Eigen::MatrixXf& grid_points,
		int res_x, int res_y, int res_z,
		std::vector<float> bound_upper_kdtree, std::vector<float> bound_lower_kdtree,
		int k_normal_size,
		Eigen::MatrixXd& out_normal);
	void GenerateMesh(Eigen::MatrixXf& grid_points, int res_x, int res_y, int res_z, float beta, float sigma, int k_size, bool use_radii, std::string mesh_full_path, clock_t strat_time, clock_t strat);
	void GenerateMesh(Eigen::MatrixXf& cloud_points, Eigen::VectorXf& leaves_As, Eigen::MatrixXf& grid_points,
		std::vector<std::vector<int>>& O_PI, Eigen::MatrixXi& O_CH, Eigen::MatrixXd& O_CM, Eigen::VectorXd& O_R, Eigen::MatrixXd& O_EC,
		int res_x, int res_y, int res_z,
		std::vector<float> bound_upper_kdtree, std::vector<float> bound_lower_kdtree,
		float beta, float sigma, int k_size, int k_normal_size, bool use_radii, float As_threshold, std::string mesh_full_path, clock_t strat_time, clock_t strat, int output_mode);
	void MeshGeneration(std::vector<std::vector<int>>& O_PI, Eigen::MatrixXi& O_CH, Eigen::MatrixXd& O_CM, Eigen::VectorXd& O_R, Eigen::MatrixXd& O_EC,
		int res_x, int res_y, int res_z,
		float beta, float sigma, int k_size, bool use_radii, float As_threshold, std::string mesh_full_path, clock_t start, std::chrono::high_resolution_clock::time_point start_time_s, int output_mode);
protected:
	float3* m_Point_Cloud_GPU = 0;
	float3* m_Grid_Points_GPU = 0;

	float* m_PC_WN_GPU = 0;
	float* m_GP_WN_GPU = 0;

	float3* m_Normals_GPU = 0;
	float* m_As_GPU = 0;
	float* m_Radii_GPU = 0;

	int m_Point_Cloud_size = 0;
	int m_Grid_Points_size = 0;

	int m_Grid_Res_X = 0;
	int m_Grid_Res_Y = 0;
	int m_Grid_Res_Z = 0;

	float3* m_Bound_Upper_KDT_GPU = 0;
	float3* m_Bound_Lower_KDT_GPU = 0;

	TurboLibiGWN* m_GWN_GPU = 0;

	void OutlierFilter(float* As, float threshold, int point_num);
};