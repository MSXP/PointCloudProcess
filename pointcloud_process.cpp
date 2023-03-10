#include <iostream>
#include <fstream>
#include <sstream>
#include <sys/types.h>
#include <dirent.h>
#include <vector>
#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include "include/keypoints_extraction.h"
#include "include/region_growing.h"

using namespace std;

// 数据集目录
string DATASET_PATH = "/home/xp/local_feature/nkxp/benchmark_datasets/oxford/";
string csv_file_name = "pointcloud_locations_20m_10overlap.csv";
string point_cloud_folder = "pointcloud_20m_10overlap/";
string regions_cloud_folder = "regionscloud_20m_10overlap/";
// string local_features_folder = "localfeature_20m_10overlap/";
// string global_features_folder = "globalfeature_20m_overlap/";

// 关键点滤波半径
double FILTER_RADIUS = 0.05;
// 半径内需要满足的点数量
int MIN_NEIGHBORS = 10;
// 采样的关键点数量
int SAMPLE_NUM = 128;
// 每个关键点生长区域的大小
int ONE_REGION_SIZE = 32;
// 法线估计需要的近邻点个数
int NORMAL_NEBOR_SIZE = 5;
// 区域生长的近邻点搜索个数
int K_NEBOR_SIZE = 8;
// 区域生长的法线阈值和曲率阈值
float CURVATURE_THRESHOLD = 0.03;
float NORMAL_THRESHOLD = 15.0;

struct Double_point
{
    double x;
    double y;
    double z;
};

// 获取文件夹下的文件目录
void getFiles(string path, vector<string> &filenames)
{
    DIR *pDir;
    struct dirent *ptr;
    if (!(pDir = opendir(path.c_str())))
    {
        cout << "Folder doesn't Exist!" << endl;
        return;
    }
    while ((ptr = readdir(pDir)) != 0)
    {
        if (strcmp(ptr->d_name, ".") != 0 && strcmp(ptr->d_name, "..") != 0)
        {
            filenames.push_back(path + ptr->d_name + "/");
        }
    }
    closedir(pDir);
}

// 读取二进制文件,转换为pcd点云文件
pcl::PointCloud<pcl::PointXYZ>::Ptr bin2pcd(string &in_file)
{
    fstream input(in_file.c_str(), ios::in | ios::binary);
    if (!input.good())
    {
        cerr << "Couldn't read in_file: " << in_file << endl;
        exit(1);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr points(new pcl::PointCloud<pcl::PointXYZ>);

    while (1)
    {
        Double_point dp;
        input.read((char *)&dp.x, sizeof(double));
        input.read((char *)&dp.y, sizeof(double));
        input.read((char *)&dp.z, sizeof(double));
        
        if (input.eof())
            break;
        pcl::PointXYZ point;
        point.x = dp.x;
        point.y = dp.y;
        point.z = dp.z;
        
        points->push_back(point);
    }

    input.close();
    return points;
}

// 区域生长后的点云转换为二进制文件并保存
void regions2bin(string &out_file, pcl::PointCloud<pcl::PointXYZ>::Ptr regions)
{
    fstream output(out_file.c_str(), ios::out | ios::binary);
    if (!output.good())
    {
        cerr << "Couldn't read out_file: " << out_file << endl;
        exit(1);
    }

    for (int i = 0; i < regions->points.size(); ++i)
    {
        output.write((char *)&regions->points[i], 3 * sizeof(float));
    }

    output.close();
}

void getIndices(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudin, pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints, pcl::PointIndices::Ptr indices)
{
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloudin); //原始点云
    std::vector<int> Idx;
    std::vector<float> Distance; //近邻点集的距离
    for (size_t i = 0; i < keypoints->points.size(); ++i)
    {
        kdtree.nearestKSearch(keypoints->points[i], 1, Idx, Distance);
        // cout<<"the indieces is:"<<Idx[0]<<endl;
        // cout<<"the distance is:"<<Distance[0]<<endl;
        indices->indices.push_back(Idx[0]);
    }
}


int main()
{
    string BASE_DIR = getcwd(NULL, 0);
    vector<string> runs_files;
    // 获取路径下所有文件夹的路径
    getFiles(BASE_DIR + DATASET_PATH, runs_files);

    for (int i = 0; i < runs_files.size(); ++i)
    {
        // 获取csv文件
        ifstream point_clouds(runs_files[i] + csv_file_name, ios::in);
        if (!point_clouds)
        {
            cout << "failed to open the csv file!" << endl;
            exit(1);
        }

        string line;
        // 跳过列名，不作处理
        getline(point_clouds, line);

        int pc_num = 0;
        while(getline(point_clouds, line))
        {
            // 获取第一列的文件名
            string field;
            istringstream readline(line);
            getline(readline, field, ',');

            // 将bin文件转换为点云
            string bin_file = runs_files[i] + point_cloud_folder + field + ".bin";
            pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud = bin2pcd(bin_file);

            // 提取关键点
            pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_ptr = get_narfKeypoints(point_cloud);

            // 关键点筛选第一步：去除周围点较少和与其他关键点离得太近的关键点
            search::KdTree<PointXYZ>::Ptr point_cloud_tree(new search::KdTree<PointXYZ>);
            point_cloud_tree->setInputCloud(point_cloud);
            search::KdTree<PointXYZ>::Ptr keypoints_tree(new search::KdTree<PointXYZ>);
            keypoints_tree->setInputCloud(keypoints_ptr);
            vector<int> points_nebor_idx;
            vector<float> points_nebor_distance;
            vector<int> keypoints_nebor_idx;
            vector<float> keypoints_nebor_distance;
            // 该数组记录关键点是否需要剔除，默认全部保留(1)
            vector<int> point_label;
            point_label.resize(keypoints_ptr->points.size(), 1);
            for(int j = 0; j < keypoints_ptr->points.size(); ++j){
                if(point_label[j] == 0){
                    continue;
                }
                int nearest_points = point_cloud_tree->radiusSearch(keypoints_ptr->points[j], FILTER_RADIUS, points_nebor_idx, points_nebor_distance);
                int nearest_keypoints = keypoints_tree->radiusSearch(keypoints_ptr->points[j], FILTER_RADIUS, keypoints_nebor_idx, keypoints_nebor_distance);
                // 关键点周围点太少，剔除该关键点
                if(nearest_points < MIN_NEIGHBORS){
                    point_label[j] = 0;
                    continue;
                }
                // 剔除该关键点周围的其他关键点
                while(nearest_keypoints > 1){
                    point_label[keypoints_nebor_idx[nearest_keypoints-1]] = 0;
                    nearest_keypoints--;
                }
            }
            pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_keypoints(new pcl::PointCloud<pcl::PointXYZ>);
            for(int j = 0; j < point_label.size(); ++j){
                if(point_label[j] == 1){
                    filtered_keypoints->push_back(keypoints_ptr->points[j]);
                }
            }

            // 1.如何限制关键点数量? 采用最远点采样(modified)
            pcl::PointCloud<pcl::PointXYZ>::Ptr sample_keypoints(new pcl::PointCloud<pcl::PointXYZ>);
            // 先随机选取一个点
            srand((unsigned)time(NULL));
            int keypoints_num = filtered_keypoints->points.size();
            int index = rand() % keypoints_num;
            sample_keypoints->push_back(filtered_keypoints->points[index]);
            // 原点集到采样点集的最小距离，初始化为最大值
            float min_dist[keypoints_num];
            float *ptr = min_dist;
            for (int j = 0; j < keypoints_num; ++j)
            {
                *ptr = FLT_MAX;
                ptr++;
            }
            // 利用上一个采样点更新最小距离数组，并选择距离最大值作为采样点
            for (int j = 0; j < SAMPLE_NUM - 1; ++j)
            {
                float max_dist = -FLT_MAX;
                int sample_idx = -1;
                for (int k = 0; k < keypoints_num; ++k)
                {
                    pcl::Vector3fMap pt = filtered_keypoints->points[k].getVector3fMap();
                    pcl::Vector3fMap pt_last = sample_keypoints->points[j].getVector3fMap();
                    float dist = (pt_last - pt).norm();
                    if (dist < min_dist[k])
                    {
                        min_dist[k] = dist;
                    }
                    if (min_dist[k] > max_dist)
                    {
                        max_dist = min_dist[k];
                        sample_idx = k;
                    }
                }
                sample_keypoints->push_back(filtered_keypoints->points[sample_idx]);
            }

            // 获取关键点在点云中的索引
            pcl::PointIndices::Ptr keypoints_indices(new pcl::PointIndices); 
            getIndices(point_cloud, sample_keypoints, keypoints_indices);

            // 区域生长
            region_growing reg_grow(CURVATURE_THRESHOLD, NORMAL_THRESHOLD, K_NEBOR_SIZE, ONE_REGION_SIZE);
            reg_grow.setinput_point(point_cloud);
            reg_grow.normal_estimation(NORMAL_NEBOR_SIZE);
            for (int k = 0; k < keypoints_indices->indices.size(); ++k)
            {
                // 2.如何限制每个区域的大小?
                reg_grow.region_growing_one(keypoints_indices->indices[k]);
            }
            // 3.分割结果如何保存下来?
            vector<int> regions_idx = reg_grow.get_all_index();
            pcl::PointCloud<pcl::PointXYZ>::Ptr regions(new pcl::PointCloud<pcl::PointXYZ>);
            copyPointCloud<pcl::PointXYZ, pcl::PointXYZ>(*point_cloud, regions_idx, *regions);
            // 保存区域生长后的点云到文件
            string out_file = runs_files[i] + regions_cloud_folder;
            if (access(out_file.c_str(), 0) == -1)
                mkdir(out_file.c_str(), S_IRWXU | S_IRWXG | S_IROTH);
            out_file = out_file + field + ".bin";
            regions2bin(out_file, regions);

            pc_num++;
            // 内存释放
            point_cloud->~PointCloud();
            keypoints_ptr->~PointCloud();
            filtered_keypoints->~PointCloud();
            sample_keypoints->~PointCloud();
            regions->~PointCloud();
            point_cloud_tree->~KdTree();
            keypoints_tree->~KdTree();
        }

        point_clouds.close();
        cout << runs_files[i] << " with " << pc_num << " point clouds"
             << "-----> finished!" << endl;
    }

    return 0;
}
