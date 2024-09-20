#include <chrono>

#include "DWGProcess.h"

void DWGLaunch(std::string point_cloud_full_path, std::string out_mesh_path, std::string out_mesh_full_path, int input_normal, int grid_reso, int final_reso, int kw, int final_kw, int normal_kw, int max_depth, float sigma, int min_num, bool use_radii, int com_As, float As_threshold, float offset_1, float offset_2, int use_leaves_generate, int total_iter, int test_iter, int output_mode)
{
    clock_t start, finish;
    start = clock();

    std::chrono::high_resolution_clock::time_point start_time_s = std::chrono::high_resolution_clock::now();

    test3D::DataprePointCloud(point_cloud_full_path, max_depth, input_normal, com_As, start, start_time_s);
   
    test3D::MainIterative(start, start_time_s, out_mesh_path, out_mesh_full_path, input_normal, grid_reso, final_reso, kw, final_kw, normal_kw, max_depth, sigma, min_num, use_radii, com_As, As_threshold, offset_1, offset_2, use_leaves_generate, total_iter, test_iter, output_mode);
}

int main(int argc, char* argv[])
{
    std::string point_cloud_path = " ";
    std::string file_name = " ";
    std::string out_mesh_path = " ";
    int input_normal = 3;
    int grid_reso = 400;
    int final_reso = 500;
    int kw = 10;
    int final_kw = 10;
    int normal_kw = 10;
    int max_depth = 10;
    int total_iter = 40;
    float sigma = 10;
    int output_mode = 1;
    int min_num = 0;
    int use_leaves_generate = 0;
    bool use_radii = 0;
    int com_As = 0;
    float As_threshold = 0.0;
    float offset_1 = 0.05;
    float offset_2 = 0.01;
    int test_iter = 999999;

    if (std::strcmp(argv[1], "--help") == 0)
    {
        std::cout << "--in_path   " << "The absolute path of the input point cloud" << std::endl;
        std::cout << "--in_name   " << "The file name of the input point cloud (only supports .xyz currently)" << std::endl;
        std::cout << "--out_path   " << "The absolute path of the output mesh/point cloud with normals" << std::endl;
        std::cout << "--output_mode   " << "Ouput mode: 1 Mesh(.stl) (default)   2 Point cloud with normals(.xyz)   3 Both 1 and 2" << std::endl;
        std::cout << "--in_normal   " << "Initialization mode: 0 Random   1 Input   3 Gauss map (default)" << std::endl;
        std::cout << "--lambda   " << "Screening Coefficient" << std::endl;
        std::cout << "--max_depth   " << "The max depth of octree" << std::endl;
        std::cout << "--use_radii   " << "Turn on screening mode or not (0 turn off   1 turn on)" << std::endl;
        std::cout << "--kw   " << "The number of nearest neighbors for the iteration (default 10)" << std::endl;
        std::cout << "--final_kw   " << "The number of nearest neighbors for the final output, usually be same to kw (default 10)" << std::endl;
        std::cout << "--grid_reso   " << "The resolution of Marching Cubes for the iteration (e.g. 200 for <10w size; 450 for >1000w size)" << std::endl;
        std::cout << "--final_reso   " << "The resolution of Marching Cubes for the final output (e.g. 260 for <10w size; 700 for >1000w size)" << std::endl;
        std::cout << "--total_iter   " << "The number of iteration" << std::endl;
        std::cout << "--test_iter   " << "Output the result each [test_iter] iterations (it is turned off by default)" << std::endl;
        std::cout << "--com_As   " << "Computing area weight or not (0 turn off   1 turn on)" << std::endl;
        std::cout << "--use_leaves_generate   " << "Recommend to open it (0 turn off   1 turn on), when the point cloud noise is high" << std::endl;

        return 0;
    }

    for (int i = 1; i < argc; i += 2)
    {
        if (std::strcmp(argv[i], "--in_path") == 0)
        {
            point_cloud_path = argv[i + 1];
        }
        else if (std::strcmp(argv[i], "--in_name") == 0)
        {
            file_name = argv[i + 1];
        }
        else if (std::strcmp(argv[i], "--out_path") == 0)
        {
            out_mesh_path = argv[i + 1];
        }
        else if (std::strcmp(argv[i], "--in_normal") == 0)
        {
            input_normal = std::stoi(argv[i + 1]);
        }
        else if (std::strcmp(argv[i], "--grid_reso") == 0)
        {
            grid_reso = std::stoi(argv[i + 1]);
        }
        else if (std::strcmp(argv[i], "--final_reso") == 0)
        {
            final_reso = std::stoi(argv[i + 1]);
        }
        else if (std::strcmp(argv[i], "--max_depth") == 0)
        {
            max_depth = std::stoi(argv[i + 1]);
        }
        else if (std::strcmp(argv[i], "--kw") == 0)
        {
            kw = std::stoi(argv[i + 1]);
        }
        else if (std::strcmp(argv[i], "--final_kw") == 0)
        {
            final_kw = std::stoi(argv[i + 1]);
        }
        else if (std::strcmp(argv[i], "--normal_kw") == 0)
        {
            normal_kw = std::stoi(argv[i + 1]);
        }
        else if (std::strcmp(argv[i], "--lambda") == 0)
        {
            sigma = std::stof(argv[i + 1]);
        }
        else if (std::strcmp(argv[i], "--min_num") == 0)
        {
            min_num = std::stoi(argv[i + 1]);
        }
        else if (std::strcmp(argv[i], "--total_iter") == 0)
        {
            total_iter = std::stoi(argv[i + 1]);
        }
        else if (std::strcmp(argv[i], "--output_mode") == 0)
        {
            output_mode = std::stoi(argv[i + 1]);
        }
        else if (std::strcmp(argv[i], "--use_leaves_generate") == 0)
        {
            use_leaves_generate = std::stoi(argv[i + 1]);
        }
        else if (std::strcmp(argv[i], "--use_radii") == 0)
        {
            use_radii = std::stoi(argv[i + 1]);
        }
        else if (std::strcmp(argv[i], "--com_As") == 0)
        {
            com_As = std::stoi(argv[i + 1]);
        }
        else if (std::strcmp(argv[i], "--As_thres") == 0)
        {
            As_threshold = std::stof(argv[i + 1]);
        }
        else if (std::strcmp(argv[i], "--offset_1") == 0)
        {
            offset_1 = std::stof(argv[i + 1]);
        }
        else if (std::strcmp(argv[i], "--offset_2") == 0)
        {
            offset_2 = std::stof(argv[i + 1]);
        }
        else if (std::strcmp(argv[i], "--test_iter") == 0)
        {
            test_iter = std::stoi(argv[i + 1]);
        }
    }

    std::string point_cloud_full_path = point_cloud_path + "/" + file_name;
    std::string pure_name = file_name.substr(0, file_name.length() - 4);
    std::string mesh_full_name = pure_name + "_md_" + std::to_string(max_depth) + "_gr_" + std::to_string(grid_reso) + "_iter_" + std::to_string(total_iter);

    std::string out_mesh_full_path = out_mesh_path + "/" + mesh_full_name;
    std::string out_mesh_path_dwg = out_mesh_path + "/";

    if (com_As == 1 && input_normal == 0)
    {
        input_normal = 2;
    }

    DWGLaunch(point_cloud_full_path, out_mesh_path_dwg, out_mesh_full_path, input_normal, grid_reso, final_reso, kw, final_kw, normal_kw, max_depth, sigma, min_num, use_radii, com_As, As_threshold, offset_1, offset_2, use_leaves_generate, total_iter, test_iter, output_mode);

    return 0;
}