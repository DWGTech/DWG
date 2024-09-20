#include <vector>
#include <iomanip>
#include <random>
#include <queue>
#include <iostream>
#include <unordered_map>

#include "cukd/fcp.h"
#include "cukd/knn.h"
#include "cukd/builder.h"

#include "geometrycentral/utilities/vector3.h"

using namespace cukd;

float3* generatePoints(int N)
{
    //static int g_seed = 100000;
    //std::seed_seq seq{ g_seed++ };
    // std::random_device rd(seq());  // Will be used to obtain a seed for the random number engine
    //std::default_random_engine rd(seq);
    //std::mt19937 gen(rd()); // Standard mersenne_twister_engine seeded with rd()
    //std::uniform_int_distribution<> dist(0, N);
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0.0, 100.0);

    std::cout << "generating " << N << " uniform random points" << std::endl;
    float3* d_points = 0;
    cudaMallocManaged((char**)&d_points, N * sizeof(*d_points));
    if (!d_points)
        throw std::runtime_error("could not allocate points mem...");

    enum { num_dims = num_dims_of<float3>::value };
    for (int i = 0; i < N; i++)
        for (int d = 0; d < num_dims; d++) {
            ((float*)&d_points[i])[d] = (float)dis(gen);
        }
    return d_points;
}

template<typename CandidateList>
__global__
void d_knn(int* d_results,
    float3* d_queries,
    int      numQueries,
    const cukd::box_t<float3>* d_bounds,
    float3* d_nodes,
    int      numNodes,
    int      k,
    float    cutOffRadius)
{
    int tid = threadIdx.x + blockIdx.x * blockDim.x;
    if (tid >= numQueries) return;

    CandidateList result(cutOffRadius);

    cct::knn(result,
        d_queries[tid], *d_bounds, d_nodes, numNodes);

    for (int i = 0; i < k; i++)
    {
        int c_index = tid * k + i;
        d_results[c_index] = result.get_pointID(i);
    }
}

extern "C" void GenerateKDTreeForGWN(std::vector<float>& points_pos_x, std::vector<float>& points_pos_y, std::vector<float>& points_pos_z,
    std::vector<float>& new_points_pos_x, std::vector<float>& new_points_pos_y, std::vector<float>& new_points_pos_z,
    std::vector<float>& bound_upper, std::vector<float>& bound_lower,
    bool out_index_map,
    std::unordered_map<geometrycentral::Vector3, int>& points_indices_map)
{
    using namespace cukd::common;

    /*=======================================================================================*/
    int numPoints = points_pos_x.size();
    float3* d_points = 0;
    cudaMallocManaged((char**)&d_points, numPoints * sizeof(*d_points));
    if (!d_points)
        throw std::runtime_error("could not allocate points mem...");

    for (int i = 0; i < numPoints; i++) {
        d_points[i].x = points_pos_x[i];
        d_points[i].y = points_pos_y[i];
        d_points[i].z = points_pos_z[i];
    }
    /*=======================================================================================*/

    if (out_index_map)
    {
        for (int i = 0; i < numPoints; i++) {
            geometrycentral::Vector3 c_pos;
            c_pos.x = (float)d_points[i].x;
            c_pos.y = (float)d_points[i].y;
            c_pos.z = (float)d_points[i].z;

            points_indices_map.insert(std::pair<geometrycentral::Vector3, int>(c_pos, i));
        }
    }
    else
    {
        points_indices_map.clear();
    }

    /*=======================================================================================*/
    cukd::box_t<float3>* d_bounds;
    cudaMallocManaged((void**)&d_bounds, sizeof(cukd::box_t<float3>));
    std::cout << "allocated memory for the world space bounding box ..." << std::endl;
    /*=======================================================================================*/

    /*=======================================================================================*/
    std::cout << "calling builder..." << std::endl;
    double t0 = getCurrentTime();
    cukd::buildTree(d_points, numPoints, d_bounds);
    CUKD_CUDA_SYNC_CHECK();
    double t1 = getCurrentTime();
    std::cout << "done building tree, took "
        << prettyDouble(t1 - t0) << "s" << std::endl;
    /*=======================================================================================*/

    bound_upper[0] = d_bounds->upper.x; bound_upper[1] = d_bounds->upper.y; bound_upper[2] = d_bounds->upper.z;
    bound_lower[0] = d_bounds->lower.x; bound_lower[1] = d_bounds->lower.y; bound_lower[2] = d_bounds->lower.z;

    std::cout << "KDT Upper: (" << bound_upper[0] << " " << bound_upper[1] << " " << bound_upper[2] << ")" << std::endl;
    std::cout << "KDT Lower: (" << bound_lower[0] << " " << bound_lower[1] << " " << bound_lower[2] << ")" << std::endl;

    for (int i = 0; i < numPoints; i++)
    {
        new_points_pos_x[i] = (float)d_points[i].x;
        new_points_pos_y[i] = (float)d_points[i].y;
        new_points_pos_z[i] = (float)d_points[i].z;
    }

    cudaFree(d_points);
    d_points = nullptr;

    cudaFree(d_bounds);
    d_bounds = nullptr;
}

//void GenerateKDTreeForDWG(float3* d_points,
//    int points_size,
//    float3* bound_upper, float3* bound_lower,
//    std::unordered_map<geometrycentral::Vector3, int>& points_indices_map)
//{
//    using namespace cukd::common;
//
//    /*=======================================================================================*/
//    for (int i = 0; i < points_size; i++) {
//        geometrycentral::Vector3 c_pos;
//        c_pos.x = (double)d_points[i].x;
//        c_pos.y = (double)d_points[i].y;
//        c_pos.z = (double)d_points[i].z;
//
//        points_indices_map.insert(std::pair<geometrycentral::Vector3, int>(c_pos, i));
//    }
//    /*=======================================================================================*/
//
//    /*=======================================================================================*/
//    cukd::box_t<float3>* d_bounds;
//    cudaMallocManaged((void**)&d_bounds, sizeof(cukd::box_t<float3>));
//    std::cout << "allocated memory for the world space bounding box ..." << std::endl;
//    /*=======================================================================================*/
//
//    /*=======================================================================================*/
//    std::cout << "calling builder..." << std::endl;
//    double t0 = getCurrentTime();
//    cukd::buildTree(d_points, points_size, d_bounds);
//    CUKD_CUDA_SYNC_CHECK();
//    double t1 = getCurrentTime();
//    std::cout << "done building tree, took "
//        << prettyDouble(t1 - t0) << "s" << std::endl;
//    /*=======================================================================================*/
//
//    bound_upper[0].x = d_bounds->upper.x; bound_upper[0].y = d_bounds->upper.y; bound_upper[0].z = d_bounds->upper.z;
//    bound_lower[0].x = d_bounds->lower.x; bound_lower[0].y = d_bounds->lower.y; bound_lower[0].z = d_bounds->lower.z;
//
//    std::cout << "KDT Upper: (" << bound_upper[0].x << " " << bound_upper[0].y << " " << bound_upper[0].z << ")" << std::endl;
//    std::cout << "KDT Lower: (" << bound_lower[0].x << " " << bound_lower[0].y << " " << bound_lower[0].z << ")" << std::endl;
//}

void GenerateKDTreeForDWG(
    float3* d_points,
    float3* d_normals,
    int points_size,
    float3* bound_upper, float3* bound_lower)
{
    using namespace cukd::common;

    /*=======================================================================================*/
    cukd::box_t<float3>* d_bounds;
    cudaMallocManaged((void**)&d_bounds, sizeof(cukd::box_t<float3>));
    std::cout << "allocated memory for the world space bounding box ..." << std::endl;
    /*=======================================================================================*/

    /*=======================================================================================*/
    std::cout << "calling builder..." << std::endl;
    double t0 = getCurrentTime();
    cukd::buildDWGTree(d_points, d_normals, points_size, d_bounds);
    CUKD_CUDA_SYNC_CHECK();
    double t1 = getCurrentTime();
    std::cout << "done building tree, took "
        << prettyDouble(t1 - t0) << "s" << std::endl;
    /*=======================================================================================*/

    bound_upper[0].x = d_bounds->upper.x; bound_upper[0].y = d_bounds->upper.y; bound_upper[0].z = d_bounds->upper.z;
    bound_lower[0].x = d_bounds->lower.x; bound_lower[0].y = d_bounds->lower.y; bound_lower[0].z = d_bounds->lower.z;

    std::cout << "KDT Upper: (" << bound_upper[0].x << " " << bound_upper[0].y << " " << bound_upper[0].z << ")" << std::endl;
    std::cout << "KDT Lower: (" << bound_lower[0].x << " " << bound_lower[0].y << " " << bound_lower[0].z << ")" << std::endl;
}

extern "C" void CudaKNNWithKDTreeforGWN(
    std::vector<float>& tree_pos_x, std::vector<float>& tree_pos_y, std::vector<float>& tree_pos_z,
    std::vector<float>& queries_pos_x, std::vector<float>& queries_pos_y, std::vector<float>& queries_pos_z,
    std::vector<float>& bound_upper, std::vector<float>& bound_lower,
    std::vector<std::vector<int>>& results_pos_index, 
    int k)
{
    using namespace cukd::common;

    float  cutOffRadius = std::numeric_limits<float>::infinity();

    /*=======================================================================================*/
    int numPoints = tree_pos_x.size();
    float3* d_points = 0;
    cudaMallocManaged((char**)&d_points, numPoints * sizeof(*d_points));
    if (!d_points)
        throw std::runtime_error("could not allocate points mem...");

    for (int i = 0; i < numPoints; i++) {
        d_points[i].x = tree_pos_x[i];
        d_points[i].y = tree_pos_y[i];
        d_points[i].z = tree_pos_z[i];
    }
    /*=======================================================================================*/

    /*=======================================================================================*/
    cukd::box_t<float3>* d_bounds;
    cudaMallocManaged((void**)&d_bounds, sizeof(cukd::box_t<float3>));
    d_bounds->lower.x = bound_lower[0]; d_bounds->lower.y = bound_lower[1]; d_bounds->lower.z = bound_lower[2];
    d_bounds->upper.x = bound_upper[0]; d_bounds->upper.y = bound_upper[1]; d_bounds->upper.z = bound_upper[2];
    /*=======================================================================================*/

    /*=======================================================================================*/
    double t0 = getCurrentTime();

    int numQueries = queries_pos_x.size();
    float3* d_queries = 0;
    cudaMallocManaged((char**)&d_queries, numQueries * sizeof(*d_queries));
    if (!d_queries)
        throw std::runtime_error("could not allocate points mem...");

    for (int i = 0; i < numQueries; i++) {
        d_queries[i].x = queries_pos_x[i];
        d_queries[i].y = queries_pos_y[i];
        d_queries[i].z = queries_pos_z[i];
    }
    /*=======================================================================================*/

    int* d_results;
    CUKD_CUDA_CALL(MallocManaged((void**)&d_results, numQueries * k * sizeof(*d_results)));
    CUKD_CUDA_SYNC_CHECK();

    int bs = 128;
    int nb = divRoundUp(numQueries, bs);
    
    if (k == 108)
    {
        d_knn<HeapCandidateList<108>> << <nb, bs >> >
            (d_results,
                d_queries,
                numQueries,
                d_bounds,
                d_points,
                numPoints,
                k,
                cutOffRadius);
    }
    else if (k == 64)
    {
        d_knn<HeapCandidateList<64>> << <nb, bs >> >
            (d_results,
                d_queries,
                numQueries,
                d_bounds,
                d_points,
                numPoints,
                k,
                cutOffRadius);
    }
    else if (k == 36)
    {
        d_knn<HeapCandidateList<36>> << <nb, bs >> >
            (d_results,
                d_queries,
                numQueries,
                d_bounds,
                d_points,
                numPoints,
                k,
                cutOffRadius);
    }
    else if (k == 30)
    {
        d_knn<HeapCandidateList<30>> << <nb, bs >> >
            (d_results,
                d_queries,
                numQueries,
                d_bounds,
                d_points,
                numPoints,
                k,
                cutOffRadius);
    }
    else if (k == 20)
    {
        d_knn<FixedCandidateList<20>> << <nb, bs >> >
            (d_results,
                d_queries,
                numQueries,
                d_bounds,
                d_points,
                numPoints,
                k,
                cutOffRadius);
    }
    else if (k == 15)
    {
        d_knn<FixedCandidateList<15>> << <nb, bs >> >
            (d_results,
                d_queries,
                numQueries,
                d_bounds,
                d_points,
                numPoints,
                k,
                cutOffRadius);
    }
    else if (k == 10)
    {
        d_knn<FixedCandidateList<10>> << <nb, bs >> >
            (d_results,
                d_queries,
                numQueries,
                d_bounds,
                d_points,
                numPoints,
                k,
                cutOffRadius);
    }
    else if (k == 9)
    {
        d_knn<FixedCandidateList<9>> << <nb, bs >> >
            (d_results,
                d_queries,
                numQueries,
                d_bounds,
                d_points,
                numPoints,
                k,
                cutOffRadius);
    }
    else if (k == 8)
    {
        d_knn<FixedCandidateList<8>> << <nb, bs >> >
            (d_results,
                d_queries,
                numQueries,
                d_bounds,
                d_points,
                numPoints,
                k,
                cutOffRadius);
    }
    else if (k == 7)
    {
        d_knn<FixedCandidateList<7>> << <nb, bs >> >
            (d_results,
                d_queries,
                numQueries,
                d_bounds,
                d_points,
                numPoints,
                k,
                cutOffRadius);
    }
    else if (k == 6)
    {
        d_knn<FixedCandidateList<6>> << <nb, bs >> >
            (d_results,
                d_queries,
                numQueries,
                d_bounds,
                d_points,
                numPoints,
                k,
                cutOffRadius);
    }
    else if (k == 5)
    {
        d_knn<FixedCandidateList<5>> << <nb, bs >> >
            (d_results,
                d_queries,
                numQueries,
                d_bounds,
                d_points,
                numPoints,
                k,
                cutOffRadius);
    }
    else if (k == 4)
    {
        d_knn<FixedCandidateList<4>> << <nb, bs >> >
            (d_results,
                d_queries,
                numQueries,
                d_bounds,
                d_points,
                numPoints,
                k,
                cutOffRadius);
    }
    else if (k == 3)
    {
        d_knn<FixedCandidateList<3>> << <nb, bs >> >
            (d_results,
                d_queries,
                numQueries,
                d_bounds,
                d_points,
                numPoints,
                k,
                cutOffRadius);
    }
    else if (k == 2)
    {
        d_knn<FixedCandidateList<2>> << <nb, bs >> >
            (d_results,
                d_queries,
                numQueries,
                d_bounds,
                d_points,
                numPoints,
                k,
                cutOffRadius);
    }
    else if (k == 1)
    {
        d_knn<FixedCandidateList<1>> << <nb, bs >> >
            (d_results,
                d_queries,
                numQueries,
                d_bounds,
                d_points,
                numPoints,
                k,
                cutOffRadius);
    }


    CUKD_CUDA_SYNC_CHECK();

    double t1 = getCurrentTime();
    std::cout << "done searching, took "
        << prettyDouble(t1 - t0) << "s" << std::endl;

    for (int i = 0; i < numQueries; i++)
    {
        for (int u = 0; u < k; u++)
        {
            results_pos_index[i][u] = d_results[i * k + u];
        }
    }

    cudaFree(d_points);
    d_points = nullptr;

    cudaFree(d_bounds);
    d_bounds = nullptr;

    cudaFree(d_queries);
    d_queries = nullptr;

    cudaFree(d_results);
    d_results = nullptr;
}

void CudaKNNWithKDTreeforDWG(
    float3* d_points,
    int numPoints,
    float3* d_queries,
    int numQueries,
    float3* bound_upper, float3* bound_lower,
    int* d_results,
    int k)
{
    using namespace cukd::common;

    float  cutOffRadius = std::numeric_limits<float>::infinity();

    /*=======================================================================================*/
    cukd::box_t<float3>* d_bounds;
    cudaMallocManaged((void**)&d_bounds, sizeof(cukd::box_t<float3>));
    d_bounds->lower.x = bound_lower[0].x; d_bounds->lower.y = bound_lower[0].y; d_bounds->lower.z = bound_lower[0].z;
    d_bounds->upper.x = bound_upper[0].x; d_bounds->upper.y = bound_upper[0].y; d_bounds->upper.z = bound_upper[0].z;
    /*=======================================================================================*/

    double t0 = getCurrentTime();

    int bs = 128;
    int nb = divRoundUp(numQueries, bs);

    //CUKD_CUDA_SYNC_CHECK();

    if (k == 108)
    {
        d_knn<HeapCandidateList<108>> << <nb, bs >> >
            (d_results,
                d_queries,
                numQueries,
                d_bounds,
                d_points,
                numPoints,
                k,
                cutOffRadius);
    }
    else if (k == 64)
    {
        d_knn<HeapCandidateList<64>> << <nb, bs >> >
            (d_results,
                d_queries,
                numQueries,
                d_bounds,
                d_points,
                numPoints,
                k,
                cutOffRadius);
    }
    else if (k == 36)
    {
        d_knn<HeapCandidateList<36>> << <nb, bs >> >
            (d_results,
                d_queries,
                numQueries,
                d_bounds,
                d_points,
                numPoints,
                k,
                cutOffRadius);
    }
    else if (k == 30)
    {
        d_knn<HeapCandidateList<30>> << <nb, bs >> >
            (d_results,
                d_queries,
                numQueries,
                d_bounds,
                d_points,
                numPoints,
                k,
                cutOffRadius);
    }
    else if (k == 20)
    {
        d_knn<HeapCandidateList<20>> << <nb, bs >> >
            (d_results,
                d_queries,
                numQueries,
                d_bounds,
                d_points,
                numPoints,
                k,
                cutOffRadius);
    }
    else if (k == 15)
    {
        d_knn<FixedCandidateList<15>> << <nb, bs >> >
            (d_results,
                d_queries,
                numQueries,
                d_bounds,
                d_points,
                numPoints,
                k,
                cutOffRadius);
    }
    else if (k == 10)
    {
        d_knn<FixedCandidateList<10>> << <nb, bs >> >
            (d_results,
                d_queries,
                numQueries,
                d_bounds,
                d_points,
                numPoints,
                k,
                cutOffRadius);
    }
    else if (k == 9)
    {
        d_knn<FixedCandidateList<9>> << <nb, bs >> >
            (d_results,
                d_queries,
                numQueries,
                d_bounds,
                d_points,
                numPoints,
                k,
                cutOffRadius);
    }
    else if (k == 8)
    {
        d_knn<FixedCandidateList<8>> << <nb, bs >> >
            (d_results,
                d_queries,
                numQueries,
                d_bounds,
                d_points,
                numPoints,
                k,
                cutOffRadius);
    }
    else if (k == 7)
    {
        d_knn<FixedCandidateList<7>> << <nb, bs >> >
            (d_results,
                d_queries,
                numQueries,
                d_bounds,
                d_points,
                numPoints,
                k,
                cutOffRadius);
    }
     else if (k == 6)
    {
        d_knn<FixedCandidateList<6>> << <nb, bs >> >
            (d_results,
                d_queries,
                numQueries,
                d_bounds,
                d_points,
                numPoints,
                k,
                cutOffRadius);
    }
    else if (k == 5)
    {
        d_knn<FixedCandidateList<5>> << <nb, bs >> >
            (d_results,
                d_queries,
                numQueries,
                d_bounds,
                d_points,
                numPoints,
                k,
                cutOffRadius);
    }
    else if (k == 4)
    {
        d_knn<FixedCandidateList<4>> << <nb, bs >> >
            (d_results,
                d_queries,
                numQueries,
                d_bounds,
                d_points,
                numPoints,
                k,
                cutOffRadius);
    }
    else if (k == 3)
    {
        d_knn<FixedCandidateList<3>> << <nb, bs >> >
            (d_results,
                d_queries,
                numQueries,
                d_bounds,
                d_points,
                numPoints,
                k,
                cutOffRadius);
    }
    else if (k == 2)
    {
        d_knn<FixedCandidateList<2>> << <nb, bs >> >
            (d_results,
                d_queries,
                numQueries,
                d_bounds,
                d_points,
                numPoints,
                k,
                cutOffRadius);
    }
    else if (k == 1)
    {
        d_knn<FixedCandidateList<1>> << <nb, bs >> >
            (d_results,
                d_queries,
                numQueries,
                d_bounds,
                d_points,
                numPoints,
                k,
                cutOffRadius);
    }

    CUKD_CUDA_SYNC_CHECK();

    double t1 = getCurrentTime();
    std::cout << "done searching, took "
        << prettyDouble(t1 - t0) << "s" << std::endl;
}

extern "C" void CudaKNNforGWN(std::vector<float>& points_pos_x, std::vector<float>& points_pos_y, std::vector<float>& points_pos_z,
                   std::vector<float>& queries_pos_x, std::vector<float>& queries_pos_y, std::vector<float>& queries_pos_z,
                   std::vector<std::vector<float>>& results_pos_x, std::vector<std::vector<float>>& results_pos_y, std::vector<std::vector<float>>& results_pos_z,
                   int k)
{
    using namespace cukd::common;

    float  cutOffRadius = std::numeric_limits<float>::infinity();

    /*=======================================================================================*/
    int numPoints = points_pos_x.size();
    float3* d_points = 0;
    cudaMallocManaged((char**)&d_points, numPoints * sizeof(*d_points));
    if (!d_points)
        throw std::runtime_error("could not allocate points mem...");

    for (int i = 0; i < numPoints; i++) {
        d_points[i].x = points_pos_x[i];
        d_points[i].y = points_pos_y[i];
        d_points[i].z = points_pos_z[i];
    }
    /*=======================================================================================*/

    /*=======================================================================================*/
    cukd::box_t<float3>* d_bounds;
    cudaMallocManaged((void**)&d_bounds, sizeof(cukd::box_t<float3>));
    std::cout << "allocated memory for the world space bounding box ..." << std::endl;
    /*=======================================================================================*/

    /*=======================================================================================*/
    std::cout << "calling builder..." << std::endl;
    double t0 = getCurrentTime();
    cukd::buildTree(d_points, numPoints, d_bounds);
    CUKD_CUDA_SYNC_CHECK();
    double t1 = getCurrentTime();
    std::cout << "done building tree, took "
        << prettyDouble(t1 - t0) << "s" << std::endl;
    /*=======================================================================================*/

    /*=======================================================================================*/
    int numQueries = queries_pos_x.size();
    float3* d_queries = 0;
    cudaMallocManaged((char**)&d_queries, numQueries * sizeof(*d_queries));
    if (!d_queries)
        throw std::runtime_error("could not allocate points mem...");

    for (int i = 0; i < numQueries; i++) {
        d_queries[i].x = queries_pos_x[i];
        d_queries[i].y = queries_pos_y[i];
        d_queries[i].z = queries_pos_z[i];
    }
    /*=======================================================================================*/

    int* d_results;
    CUKD_CUDA_CALL(MallocManaged((void**)&d_results, numQueries * k * sizeof(*d_results)));
    CUKD_CUDA_SYNC_CHECK();

    int bs = 128;
    int nb = divRoundUp(numQueries, bs);

    if (k == 30)
    {
        d_knn<HeapCandidateList<30>> << <nb, bs >> >
            (d_results,
                d_queries,
                numQueries,
                d_bounds,
                d_points,
                numPoints,
                k,
                cutOffRadius);
    }
    else
    {
        d_knn<FixedCandidateList<15>> << <nb, bs >> >
            (d_results,
                d_queries,
                numQueries,
                d_bounds,
                d_points,
                numPoints,
                k,
                cutOffRadius);
    }

    CUKD_CUDA_SYNC_CHECK();

    for (int i = 0; i < numQueries; i++)
    {
        for (int u = 0; u < k; u++)
        {
            results_pos_x[i][u] = (float)d_points[d_results[i * k + u]].x;
            results_pos_y[i][u] = (float)d_points[d_results[i * k + u]].y;
            results_pos_z[i][u] = (float)d_points[d_results[i * k + u]].z;
        }
    }
}

/*int main()
{
    using namespace cukd::common;
    int k = 30;
    float  cutOffRadius = std::numeric_limits<float>::infinity();

    int numPoints = 200000;
    float3* d_points = generatePoints(numPoints);

    int numQueries = 100000;
    float3* d_queries = generatePoints(numQueries);

    std::vector<float> points_pos_x; points_pos_x.resize(numPoints);
    std::vector<float> points_pos_y; points_pos_y.resize(numPoints);
    std::vector<float> points_pos_z; points_pos_z.resize(numPoints);

    for (int i = 0; i < numPoints; i++)
    {
        points_pos_x[i] = d_points[i].x;
        points_pos_y[i] = d_points[i].y;
        points_pos_z[i] = d_points[i].z;
    }

    std::vector<float> queries_pos_x; queries_pos_x.resize(numQueries);
    std::vector<float> queries_pos_y; queries_pos_y.resize(numQueries);
    std::vector<float> queries_pos_z; queries_pos_z.resize(numQueries);

    for (int i = 0; i < numQueries; i++)
    {
        queries_pos_x[i] = d_queries[i].x;
        queries_pos_y[i] = d_queries[i].y;
        queries_pos_z[i] = d_queries[i].z;
    }

    std::vector<float> new_points_pos_x; new_points_pos_x.resize(numPoints);
    std::vector<float> new_points_pos_y; new_points_pos_y.resize(numPoints);
    std::vector<float> new_points_pos_z; new_points_pos_z.resize(numPoints);

    std::vector<float> bound_upper;
    bound_upper.resize(3);
    std::vector<float> bound_lower;
    bound_lower.resize(3);

    GenerateKDTreeForGWN(points_pos_x, points_pos_y, points_pos_z,
             new_points_pos_x, new_points_pos_y, new_points_pos_z,
             bound_upper, bound_lower);

    std::vector<std::vector<int>> results_pos_index; results_pos_index.resize(numQueries);
    //std::vector<std::vector<float>> results_pos_y; results_pos_y.resize(numQueries);
    //std::vector<std::vector<float>> results_pos_z; results_pos_z.resize(numQueries);

    for (int i = 0; i < numQueries; i++)
    {
        std::vector<int> c_result_pos_index; c_result_pos_index.resize(k);
        //std::vector<float> c_result_pos_y; c_result_pos_y.resize(k);
        //std::vector<float> c_result_pos_z; c_result_pos_z.resize(k);

        results_pos_index[i] = c_result_pos_index;
        //results_pos_y[i] = c_result_pos_y;
        //results_pos_z[i] = c_result_pos_z;
    }

    CudaKNNWithKDTreeforGWN(new_points_pos_x, new_points_pos_y, new_points_pos_z,
        queries_pos_x, queries_pos_y, queries_pos_z,
        bound_upper, bound_lower,
        results_pos_index,
        k
    );

    
    /*for (int i = 0; i < numQueries; i++)
    {
        std::cout << i << " th:" << std::endl;
        std::cout << (float)d_queries[i].x << " " << (float)d_queries[i].y << " " << (float)d_queries[i].z << std::endl;
        for (int u = 0; u < k; u++)
        {
            int index = d_results[i * k + u];
            std::cout << (float)d_points[index].x << " " << (float)d_points[index].y << " " << (float)d_points[index].z << std::endl;
            std::cout << (float)d_points_c[index].x << " " << (float)d_points_c[index].y << " " << (float)d_points_c[index].z << std::endl;
        }

        std::cout << std::endl;
    }
}*/