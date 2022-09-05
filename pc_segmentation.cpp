#include "include/pc_segmentation.h"

/// 定义颜色数组，用于可视化
float colors[] = {
    255, 0, 0,     // red 		1
    0, 255, 0,     // green		2
    0, 0, 255,     // blue		3
    255, 255, 0,   // yellow	4
    0, 255, 255,   // light blue5
    255, 0, 255,   // magenta   6
    255, 255, 255, // white		7
    255, 128, 0,   // orange	8
    255, 153, 255, // pink		9
    51, 153, 255,  //			10
    153, 102, 51,  //			11
    128, 51, 153,  //			12
    153, 153, 51,  //			13
    163, 38, 51,   //			14
    204, 153, 102, //			15
    204, 224, 255, //			16
    128, 179, 255, //			17
    206, 255, 0,   //			18
    255, 204, 204, //			19
    204, 255, 153, //			20
};

void euclidean_clustering(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr)
{
    /// 创建kd树
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud_ptr);

    /// 设置分割参数
    vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.1); //设置近邻搜索的半径
    ec.setMinClusterSize(128);   //设置最小聚类点数
    ec.setMaxClusterSize(4096);  //设置最大聚类点数
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_ptr);
    ec.extract(cluster_indices);

    //----- 可视化1/3-----↓
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D viewer"));

    int v1(0);
    viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1); //设置第一个视口在X轴、Y轴的最小值、最大值，取值在0-1之间
    viewer->setBackgroundColor(0.1, 0.1, 0.1, v1);
    viewer->addText("cloud_ptr", 10, 10, "v1 text", v1);
    viewer->addPointCloud<pcl::PointXYZ>(cloud_ptr, "cloud_ptr", v1);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud_ptr", v1);

    int v2(0);
    viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
    viewer->setBackgroundColor(0.3, 0.3, 0.3, v2);
    viewer->addText("cloud_cluster", 10, 10, "v2 text", v2);
    //-----可视化1/3-----↑

    /// 执行欧式聚类分割，并保存分割结果
    pcl::PCDWriter writer;
    int j = 0;
    for (vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);

        for (vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++)
            cloud_cluster->points.push_back(cloud_ptr->points[*pit]);
        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        cout << "PointCloud representing the Cluster: " << cloud_cluster->size() << " data points." << endl;
        stringstream ss;
        ss << "cloud_cluster_" << j + 1 << ".pcd";
        writer.write<pcl::PointXYZ>(ss.str(), *cloud_cluster, true);
        cout << "-----" << ss.str() << "详情-----" << endl;
        cout << *cloud_cluster << endl;

        //-----可视化2/3-----↓
        viewer->addPointCloud<pcl::PointXYZ>(cloud_cluster, ss.str(), v2);
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, ss.str(), v2);
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, colors[j * 3] / 255, colors[j * 3 + 1] / 255, colors[j * 3 + 2] / 255, ss.str(), v2);
        //-----可视化2/3-----↑

        j++;
    }

    //-----可视化3/3-----↓
    while (!viewer->wasStopped())
    {
        // viewer->spinOnce(100);
        // boost::this_thread::sleep(boost::posix_time::microseconds(100000));
        viewer->spinOnce();
        pcl_sleep(0.01);
    }
    //-----可视化3/3-----↑
}

void region_growing_pcl(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr)
{
    pcl::search::Search<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
    normal_estimator.setSearchMethod(tree);
    normal_estimator.setInputCloud(cloud_ptr);
    normal_estimator.setKSearch(50);
    normal_estimator.compute(*normals);

    // pcl::IndicesPtr indices(new std::vector<int>);
    // pcl::PassThrough<pcl::PointXYZ> pass;
    // pass.setInputCloud(cloud_ptr);
    // pass.setFilterFieldName("z");
    // pass.setFilterLimits(0.0, 1.0);
    // pass.filter(*indices);

    pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
    reg.setMinClusterSize(30);
    reg.setMaxClusterSize(4000);
    reg.setSearchMethod(tree);
    reg.setNumberOfNeighbours(25);
    reg.setInputCloud(cloud_ptr);
    // reg.setIndices (indices);
    reg.setInputNormals(normals);
    reg.setSmoothnessThreshold(3.0 / 180.0 * M_PI);
    reg.setCurvatureThreshold(1.0);

    std::vector<pcl::PointIndices> clusters;
    reg.extract(clusters);

    std::cout << "Number of clusters is equal to " << clusters.size() << std::endl;
    std::cout << "First cluster has " << clusters[0].indices.size() << " points." << std::endl;
    std::cout << "These are the indices of the points of the initial" << std::endl
              << "cloud that belong to the first cluster:" << std::endl;
    int counter = 0;
    while (counter < clusters[0].indices.size())
    {
        std::cout << clusters[0].indices[counter] << ", ";
        counter++;
        if (counter % 10 == 0)
            std::cout << std::endl;
    }
    std::cout << std::endl;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud();
    pcl::visualization::CloudViewer viewer("Cluster viewer");
    viewer.showCloud(colored_cloud);
    while (!viewer.wasStopped())
    {
    }
}