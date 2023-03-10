// #include <iostream>
// #include <fstream>
// #include <sstream>
// #include <sys/types.h>
// #include <dirent.h>
// #include <vector>
// #include <string.h>
// #include <stdio.h>
// #include <unistd.h>
// #include <pcl/io/pcd_io.h>
// #include <pcl/point_types.h>
// #include <pcl/filters/radius_outlier_removal.h>

// #include "include/handcrafted_local_features.h"
// #include "include/pc_segmentation.h"
// #include "include/keypoints_extraction.h"
// #include "include/region_growing.h"

// using namespace std;

// // 数据集目录
// string DATASET_PATH = "../test_data/";
// string ROUTE = "2014-05-19-13-20-57/";
// string point_cloud_folder = "benchmark_datasets/oxford/2014-05-19-13-20-57/pointcloud_20m_10overlap/";

// // 局部特征
// // typedef pcl::PointCloud<pcl::FPFHSignature33> handcraftedLocalFeature;
// // int LOCAL_FEATURE_SIZE = 33;
// // typedef pcl::PointCloud<pcl::SHOT352> handcraftedLocalFeature;
// // int LOCAL_FEATURE_SIZE = 352;
// typedef pcl::PointCloud<pcl::Narf36> handcraftedLocalFeature;
// int LOCAL_FEATURE_SIZE = 36;

// // 关键点滤波半径
// double FILTER_RADIUS = 0.05;
// // 半径内需要满足的点数量
// int MIN_NEIGHBORS = 10;
// // 采样的关键点数量
// int SAMPLE_NUM = 128;
// // 每个关键点生长区域的大小
// int ONE_REGION_SIZE = 32;
// // 法线估计需要的近邻点个数
// int NORMAL_NEBOR_SIZE = 5;
// // 区域生长的近邻点搜索个数
// int K_NEBOR_SIZE = 8;
// // 区域生长的法线阈值和曲率阈值
// float CURVATURE_THRESHOLD = 0.03;
// float NORMAL_THRESHOLD = 15.0;

// struct Double_point
// {
//     double x;
//     double y;
//     double z;
// };

// // 读取二进制文件,转换为pcd点云文件
// pcl::PointCloud<pcl::PointXYZ>::Ptr bin2pcd(string &in_file)
// {
//     fstream input(in_file.c_str(), ios::in | ios::binary);
//     if (!input.good())
//     {
//         cerr << "Couldn't read in_file: " << in_file << endl;
//         exit(1);
//     }

//     pcl::PointCloud<pcl::PointXYZ>::Ptr points(new pcl::PointCloud<pcl::PointXYZ>);

//     while (1)
//     {
//         Double_point dp;
//         input.read((char *)&dp.x, sizeof(double));
//         input.read((char *)&dp.y, sizeof(double));
//         input.read((char *)&dp.z, sizeof(double));
//         // 转换KITTI
//         // pcl::PointXYZI point;
//         // input.read((char *) &point.x, 3*sizeof(float));
//         // input.read((char *) &point.intensity, sizeof(float));
//         if (input.eof())
//             break;
//         pcl::PointXYZ point;
//         point.x = dp.x;
//         point.y = dp.y;
//         point.z = dp.z;
//         // pcl::PointXYZ point_t;
//         // point_t.x = point.x;
//         // point_t.y = point.y;
//         // point_t.z = point.z;
//         points->push_back(point);
//     }

//     input.close();

//     // cout << "Read point cloud with " << points->points.size() << endl;
//     // pcl::io::savePCDFileASCII(out_file, *points);
//     return points;
// }

// // 区域生长后的点云转换为二进制文件并保存
// void regions2bin(string &out_file, pcl::PointCloud<pcl::PointXYZ>::Ptr regions)
// {
//     fstream output(out_file.c_str(), ios::out | ios::binary);
//     if (!output.good())
//     {
//         cerr << "Couldn't read out_file: " << out_file << endl;
//         exit(1);
//     }

//     for (int i = 0; i < regions->points.size(); ++i)
//     {
//         output.write((char *)&regions->points[i], 3 * sizeof(float));
//     }

//     output.close();
// }

// // 提取的局部特征转换为二进制文件并保存
// void feature2bin(string &out_file, handcraftedLocalFeature::Ptr local_features)
// {
//     fstream output(out_file.c_str(), ios::out | ios::binary);
//     if (!output.good())
//     {
//         cerr << "Couldn't read out_file: " << out_file << endl;
//         exit(1);
//     }

//     for (int i = 0; i < local_features->points.size(); ++i)
//     {
//         output.write((char *)&local_features->points[i], LOCAL_FEATURE_SIZE * sizeof(float));
//     }

//     output.close();
// }

// void getIndices(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudin, pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints, pcl::PointIndices::Ptr indices)
// {
//     pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
//     kdtree.setInputCloud(cloudin); // 原始点云
//     std::vector<int> Idx;
//     std::vector<float> Distance; // 近邻点集的距离
//     for (size_t i = 0; i < keypoints->points.size(); ++i)
//     {
//         kdtree.nearestKSearch(keypoints->points[i], 1, Idx, Distance);
//         // cout<<"the indieces is:"<<Idx[0]<<endl;
//         // cout<<"the distance is:"<<Distance[0]<<endl;
//         indices->indices.push_back(Idx[0]);
//     }
// }

// int main()
// {
//     // 将bin文件转换为点云
//     // 1.经PointNetVLAD处理后的数据
//     string bin_file = DATASET_PATH + point_cloud_folder + "1400505893170765.bin";
//     // 2.原数据集的二维激光数据生成的点云
//     // string bin_file = DATASET_PATH + ROUTE + "origin/1400505893170765.bin";
//     // 3.原数据集的三维激光数据
//     // string bin_file = DATASET_PATH + ROUTE + "ldmrs/1/1400505893437719.bin";
//     pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud = bin2pcd(bin_file);
//     // pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZ>);
//     // pcl::io::loadPCDFile("/home/xp/Downloads/00001.pcd", *point_cloud);
//     cout << "point number: " << point_cloud->points.size() << endl;
//     // 可视化点云
//     pcl::visualization::PCLVisualizer viewer("region_growing");
//     viewer.setBackgroundColor(0, 0, 0);
//     pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> point_cloud_color_handler(point_cloud, 255, 255, 255);
//     viewer.addPointCloud(point_cloud, point_cloud_color_handler, "input_cloud");
//     viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "input_cloud");

//     // 提取点云特征
//     // handcraftedLocalFeature::Ptr handcrafted_local_features = get_narf(point_cloud);

//     // 将特征保存到文件
//     // string out_file = runs_files[i] + local_features_folder;
//     // if(access(out_file.c_str(),0)==-1)
//     //     mkdir(out_file.c_str(), S_IRWXU | S_IRWXG | S_IROTH);
//     // out_file = out_file + field + ".bin";
//     // feature2bin(out_file, handcrafted_local_features);

//     // 点云分割
//     // euclidean_clustering(point_cloud);
//     // region_growing(point_cloud);
    
//     // 提取关键点
//     // get_harrisKeypoints(point_cloud);
//     // get_siftKeypoints(point_cloud);
//     pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_ptr = get_narfKeypoints(point_cloud);
//     cout << "keypoint number: " << keypoints_ptr->points.size() << endl;
//     // 可视化关键点1
//     // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> keypoints_color_handler1(keypoints_ptr, 0, 255, 0);
//     // viewer.addPointCloud<pcl::PointXYZ>(keypoints_ptr, keypoints_color_handler1, "narf_keypoints");
//     // viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 8, "narf_keypoints");

//     // 关键点筛选第一步：去除周围点较少和与其他关键点离得太近的关键点
//     search::KdTree<PointXYZ>::Ptr point_cloud_tree(new search::KdTree<PointXYZ>);
// 	point_cloud_tree->setInputCloud(point_cloud);
//     search::KdTree<PointXYZ>::Ptr keypoints_tree(new search::KdTree<PointXYZ>);
// 	keypoints_tree->setInputCloud(keypoints_ptr);
//     vector<int> points_nebor_idx;
// 	vector<float> points_nebor_distance;
//     vector<int> keypoints_nebor_idx;
// 	vector<float> keypoints_nebor_distance;
//     // 该数组记录关键点是否需要剔除，默认全部保留(1)
//     vector<int> point_label;
//     point_label.resize(keypoints_ptr->points.size(), 1);
//     for(int j = 0; j < keypoints_ptr->points.size(); ++j){
//         if(point_label[j] == 0){
//             continue;
//         }
//         int nearest_points = point_cloud_tree->radiusSearch(keypoints_ptr->points[j], FILTER_RADIUS, points_nebor_idx, points_nebor_distance);
//         int nearest_keypoints = keypoints_tree->radiusSearch(keypoints_ptr->points[j], FILTER_RADIUS, keypoints_nebor_idx, keypoints_nebor_distance);
//         // 关键点周围点太少，剔除该关键点
//         if(nearest_points < MIN_NEIGHBORS){
//             point_label[j] = 0;
//             continue;
//         }
//         // 剔除该关键点周围的其他关键点
//         while(nearest_keypoints > 1){
//             point_label[keypoints_nebor_idx[nearest_keypoints-1]] = 0;
//             nearest_keypoints--;
//         }
//     }
//     pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_keypoints(new pcl::PointCloud<pcl::PointXYZ>);
//     for(int j = 0; j < point_label.size(); ++j){
//         if(point_label[j] == 1){
//             filtered_keypoints->push_back(keypoints_ptr->points[j]);
//         }
//     }
//     // 半径滤波
//     // pcl::RadiusOutlierRemoval<pcl::PointXYZ> pcFilter;
//     // pcFilter.setInputCloud(keypoints_ptr);
//     // pcFilter.setRadiusSearch(FILTER_RADIUS);
//     // pcFilter.setMinNeighborsInRadius(MIN_NEIGHBORS);
//     // // pcFilter.setNegative(true);
//     // pcFilter.filter(*filtered_keypoints);
//     cout << "keypoint number after filter: " << filtered_keypoints->points.size() << endl;

//     // 1.如何限制关键点数量? 采用最远点采样(modified)
//     pcl::PointCloud<pcl::PointXYZ>::Ptr sample_keypoints(new pcl::PointCloud<pcl::PointXYZ>);
//     // 先随机选取一个点
//     srand((unsigned)time(NULL));
//     int keypoints_num = filtered_keypoints->points.size();
//     int index = rand() % keypoints_num;
//     sample_keypoints->push_back(filtered_keypoints->points[index]);
//     // 原点集到采样点集的最小距离，初始化为最大值
//     float min_dist[keypoints_num];
//     float *ptr = min_dist;
//     for (int j = 0; j < keypoints_num; ++j)
//     {
//         *ptr = FLT_MAX;
//         ptr++;
//     }
//     // 记录上一个采样点
//     // int last_sample_idx = index;
//     // 利用上一个采样点更新最小距离数组，并选择距离最大值作为采样点
//     for (int j = 0; j < SAMPLE_NUM - 1; ++j)
//     {
//         float max_dist = -FLT_MAX;
//         int sample_idx = -1;
//         for (int k = 0; k < keypoints_num; ++k)
//         {
//             pcl::Vector3fMap pt = filtered_keypoints->points[k].getVector3fMap();
//             pcl::Vector3fMap pt_last = sample_keypoints->points[j].getVector3fMap();
//             // pcl::Vector3fMap pt_last = filtered_keypoints->points[last_sample_idx].getVector3fMap();
//             float dist = (pt_last - pt).norm();
//             if (dist < min_dist[k])
//             {
//                 min_dist[k] = dist;
//             }
//             if (min_dist[k] > max_dist)
//             {
//                 max_dist = min_dist[k];
//                 sample_idx = k;
//             }
//         }
//         sample_keypoints->push_back(filtered_keypoints->points[sample_idx]);
//         // last_sample_idx = sample_idx;
//         // cout << sample_idx << endl;
//     }
//     cout << "sample keypoint number: " << sample_keypoints->points.size() << endl;
//     // 可视化关键点2
//     pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> keypoints_color_handler2(sample_keypoints, 255, 0, 0);
//     viewer.addPointCloud<pcl::PointXYZ>(sample_keypoints, keypoints_color_handler2, "sample_keypoints");
//     viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 8, "sample_keypoints");

//     // 获取关键点在点云中的索引
//     pcl::PointIndices::Ptr keypoints_indices(new pcl::PointIndices);
//     getIndices(point_cloud, sample_keypoints, keypoints_indices);

//     // 区域生长
//     region_growing reg_grow(CURVATURE_THRESHOLD, NORMAL_THRESHOLD, K_NEBOR_SIZE, ONE_REGION_SIZE);
//     reg_grow.setinput_point(point_cloud);
//     reg_grow.normal_estimation(NORMAL_NEBOR_SIZE);
//     // vector<int> index_region;
//     // pcl::PointCloud<pcl::PointXYZ>::Ptr oneregion(new pcl::PointCloud<pcl::PointXYZ>);
//     // stringstream ss;
//     for (int k = 0; k < keypoints_indices->indices.size(); ++k)
//     {
//         // 2.如何限制每个区域的大小?
//         reg_grow.region_growing_one(keypoints_indices->indices[k]);
//         // index_region = reg_grow.region_growing_one(keypoints_indices->indices[k]);
//         // cout << "keypoint " << k << ": " << index_region.size() << endl;
//         // copyPointCloud(*point_cloud, index_region, *oneregion);
//         // 可视化各区域
//         // ss << "region_growing" << keypoints_indices->indices[k];
//         // visualization::PointCloudColorHandlerRandom<PointT> region_growing_point_color(oneregion);
//         // viewer.addPointCloud(oneregion, region_growing_point_color, ss.str());
//         // viewer.setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 4, ss.str());
//     }
//     // 3.分割结果如何保存下来?
//     vector<int> regions_idx = reg_grow.get_all_index();
//     pcl::PointCloud<pcl::PointXYZ>::Ptr regions(new pcl::PointCloud<pcl::PointXYZ>);
//     copyPointCloud<pcl::PointXYZ, pcl::PointXYZ>(*point_cloud, regions_idx, *regions);

//     // 可视化区域生长后的点云
//     // pcl::visualization::PCLVisualizer viewer2("regions");
//     // viewer2.setBackgroundColor(0, 0, 0);
//     // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> regions_color_handler(regions, 255, 255, 255);
//     // viewer2.addPointCloud(regions, regions_color_handler, "regions");
//     // viewer2.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "regions");
    
//     // 保存区域生长后的点云到文件
//     string out_file = DATASET_PATH + ROUTE;
//     if (access(out_file.c_str(), 0) == -1)
//         mkdir(out_file.c_str(), S_IRWXU | S_IRWXG | S_IROTH);
//     out_file = out_file + "1400505893170765_regions.bin";
//     regions2bin(out_file, regions);

//     viewer.spin();
//     // viewer2.spin();
//     return 0;
// }
