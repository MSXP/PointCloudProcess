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

#include "include/handcrafted_local_features.h"
#include "include/pc_segmentation.h"
#include "include/keypoints_extraction.h"

using namespace std;

// typedef pcl::PointCloud<pcl::FPFHSignature33> handcraftedLocalFeature;
// int LOCAL_FEATURE_SIZE = 33;
// typedef pcl::PointCloud<pcl::SHOT352> handcraftedLocalFeature;
// int LOCAL_FEATURE_SIZE = 352;
typedef pcl::PointCloud<pcl::Narf36> handcraftedLocalFeature;
int LOCAL_FEATURE_SIZE = 36;

//string DATASET_PATH = "/home/xp/local_feature/nkxp/benchmark_datasets/oxford/";
string DATASET_PATH = "/../../../benchmark_datasets/oxford/";
string csv_file_name = "pointcloud_locations_20m_10overlap.csv";
string point_cloud_folder = "pointcloud_20m_10overlap/";
string local_features_folder = "localfeature_20m_10overlap/";
//string global_features_folder = "globalfeature_20m_overlap/";

struct Double_point
{
    double x;
    double y;
    double z;
};

// 获取文件夹下的文件目录
void getFiles(string path, vector<string>& filenames)
{
	DIR *pDir;
    struct dirent* ptr;
    if(!(pDir = opendir(path.c_str()))){
        cout<<"Folder doesn't Exist!"<<endl;
        return;
    }
    while((ptr = readdir(pDir))!=0) {
        if (strcmp(ptr->d_name, ".") != 0 && strcmp(ptr->d_name, "..") != 0){
            filenames.push_back(path + ptr->d_name + "/");
    	}
    }
    closedir(pDir);
}

// 读取二进制文件,转换为pcd点云文件
pcl::PointCloud<pcl::PointXYZ>::Ptr bin2pcd(string& in_file)
{
    fstream input(in_file.c_str(), ios::in | ios::binary);
    if(!input.good()){
        cerr << "Couldn't read in_file: " << in_file << endl;
        exit(1);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr points(new pcl::PointCloud<pcl::PointXYZ>);

    while(1) 
    {
        Double_point dp;
        input.read((char *) &dp.x, sizeof(double));
        input.read((char *) &dp.y, sizeof(double));
        input.read((char *) &dp.z, sizeof(double));
        // 转换KITTI
        // pcl::PointXYZI point;
        // input.read((char *) &point.x, 3*sizeof(float));
        // input.read((char *) &point.intensity, sizeof(float));
        if(input.eof())
            break;
        pcl::PointXYZ point;
        point.x = dp.x;
        point.y = dp.y;
        point.z = dp.z;
        // pcl::PointXYZ point_t;
        // point_t.x = point.x;
        // point_t.y = point.y;
        // point_t.z = point.z;
        points->push_back(point);
    }

    input.close();

    //cout << "Read point cloud with " << points->points.size() << endl;
    //pcl::io::savePCDFileASCII(out_file, *points);
    return points;
}

// 提取的局部特征转换为二进制文件并保存
void feature2bin(string& out_file, handcraftedLocalFeature::Ptr local_features)
{
    fstream output(out_file.c_str(), ios::out | ios::binary);
    if(!output.good()){
        cerr << "Couldn't read out_file: " << out_file << endl;
        exit(1);
    }

    for(int i=0; i < local_features->points.size(); i++)
    {
        output.write((char *) &local_features->points[i], LOCAL_FEATURE_SIZE*sizeof(float));
    }

    output.close();
}


int main()
{
    string BASE_DIR = getcwd(NULL, 0);
    //cout << BASE_DIR << endl;
    vector<string> runs_files;
    // 获取路径下所有文件夹的路径
    getFiles(BASE_DIR + DATASET_PATH, runs_files);
    
    for(int i = 0; i < runs_files.size(); i++)
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
        getline(point_clouds, line);
        // while(getline(point_clouds, line))
        {
            // 获取第一列的文件名
            string field;
            istringstream readline(line);
            getline(readline, field, ',');

            // 将bin文件转换为点云
            string bin_file = runs_files[i] + point_cloud_folder + field + ".bin";
            pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud = bin2pcd(bin_file);
            // pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            // pcl::io::loadPCDFile("/home/xp/Downloads/00001.pcd", *point_cloud);

            // 提取点云特征
            // handcraftedLocalFeature::Ptr handcrafted_local_features = get_narf(point_cloud);

            // 点云分割
            // euclidean_clustering(point_cloud);
            region_growing(point_cloud);

            // 提取关键点
            // get_harrisKeypoints(point_cloud);
            // get_siftKeypoints(point_cloud);
            // get_narfKeypoints(point_cloud);
            
            // 将特征保存到文件
            // string out_file = runs_files[i] + local_features_folder;
            // if(access(out_file.c_str(),0)==-1)
            //     mkdir(out_file.c_str(), S_IRWXU | S_IRWXG | S_IROTH);
            // out_file = out_file + field + ".bin";
            // feature2bin(out_file, handcrafted_local_features);
            pc_num++;
        }

        point_clouds.close();
        cout << runs_files[i] << " with " << pc_num << " point clouds" << "-----> finished!" << endl;
    }

    return 0;
}
