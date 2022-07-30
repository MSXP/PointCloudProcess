#include "include/handcrafted_local_features.h"

pcl::PointCloud<pcl::FPFHSignature33>::Ptr get_fpfh(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr)
{
    if (cloud_ptr == NULL)
    {
        cout << "pcd file get err" << endl;
        return NULL;
    }
    // cout << "PointCLoud size() " << cloud_ptr->width * cloud_ptr->height
    //      << " data points ( " << pcl::getFieldsList(*cloud_ptr) << ")." << endl;

    // =====【1】计算法线========创建法线估计类====================================
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud_ptr);
    // 添加搜索算法 kdtree search  最近的几个点 估计平面 协方差矩阵PCA分解 求解法线
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod(tree); //设置近邻搜索算法
    // 输出点云 带有法线描述
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_ptr(new pcl::PointCloud<pcl::Normal>);
    // Use all neighbors in a sphere of radius 3cm
    // ne.setRadiusSearch(0.03);
    ne.setKSearch(25);
    // 计算表面法线特征
    ne.compute(*cloud_normals_ptr);

    //=======【2】创建FPFH估计对象fpfh, 并将输入点云数据集cloud和法线normals传递给它=================
    pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
    // pcl::FPFHEstimationOMP<pcl::PointXYZ,pcl::Normal,pcl::FPFHSignature33> fpfh;//多核加速
    fpfh.setInputCloud(cloud_ptr);
    // fpfh.setIndices(indicesptr);
    // fpfh.setSearchSurface(cloud_ptr);
    fpfh.setInputNormals(cloud_normals_ptr);
    //创建一个空的kd树表示法，并把它传递给PFH估计对象。
    //基于已给的输入数据集，建立kdtree
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree2(new pcl::search::KdTree<pcl::PointXYZ>());
    fpfh.setSearchMethod(tree2); //设置近邻搜索算法
    //输出数据集
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_fe_ptr(new pcl::PointCloud<pcl::FPFHSignature33>());
    //注意：此处使用的半径必须要大于估计表面法线时使用的半径!!!
    // fpfh.setRadiusSearch(0.05);
    fpfh.setKSearch(30);
    //计算fpfh特征值
    fpfh.compute(*fpfh_fe_ptr);

    // cout << "fphf feature size : " << fpfh_fe_ptr->points.size() << endl;
    // 应该与input cloud->points.size ()有相同的大小，即每个点都有一个fpfh特征向量

    // ========直方图可视化=============================
    pcl::visualization::PCLHistogramVisualizer view;                         //直方图可视化
    view.setBackgroundColor(255, 0, 0);                                      //背景红色
    view.addFeatureHistogram<pcl::FPFHSignature33>(*fpfh_fe_ptr, "fpfh", 1); //对下标为1的点的直方图特征可视化
    // view.spinOnce();  //循环的次数
    view.spin(); //无限循环

    return fpfh_fe_ptr;
}

pcl::PointCloud<pcl::SHOT352>::Ptr get_shot(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr)
{
    if (cloud_ptr == NULL)
    {
        cout << "pcd file get err" << endl;
        return NULL;
    }
    cout << "PointCLoud size() " << cloud_ptr->width * cloud_ptr->height
         << " data points (" << pcl::getFieldsList(*cloud_ptr) << ")." << endl;

    // =====【1】计算法线========创建法线估计类====================================
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud_ptr);

    // 添加搜索算法 kdtree search  最近的几个点 估计平面 协方差矩阵PCA分解 求解法线
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod(tree); //设置近邻搜索算法
    // 输出点云 带有法线描述
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_ptr(new pcl::PointCloud<pcl::Normal>);
    // Use all neighbors in a sphere of radius 3cm
    // ne.setRadiusSearch(0.03); //半径内搜索临近点 3cm
    ne.setKSearch(25);
    // 计算表面法线特征
    ne.compute(*cloud_normals_ptr);

    // pcl::SHOTLocalReferenceFrameEstimation<pcl::PointXYZ, pcl::ReferenceFrame> shot_lrf;
    // pcl::search::KdTree<pcl::PointXYZ>::Ptr tree2(new pcl::search::KdTree<pcl::PointXYZ>());
    // shot_lrf.setInputCloud(cloud_ptr);
    // shot_lrf.setSearchMethod(tree2);
    // // shot_lrf.setKSearch(30);
    // shot_lrf.setRadiusSearch(0.3);
    // pcl::PointCloud<pcl::ReferenceFrame>::Ptr shot_RF(new pcl::PointCloud<pcl::ReferenceFrame>);
    // shot_lrf.compute(*shot_RF);

    //=======【2】创建SHOT估计对象shot, 并将输入点云数据集cloud和法线normals传递给它=================
    pcl::SHOTEstimation<pcl::PointXYZ, pcl::Normal, pcl::SHOT352, pcl::ReferenceFrame> shot;
    pcl::PointCloud<pcl::SHOT352>::Ptr shot_descriptors(new pcl::PointCloud<pcl::SHOT352>);
    // pcl::search::KdTree<pcl::PointXYZ>::Ptr tree3(new pcl::search::KdTree<pcl::PointXYZ>());

    shot.setInputCloud(cloud_ptr);
    shot.setLRFRadius(0.3);
    // shot.setInputReferenceFrames(shot_RF);
    // shot.setIndices (indicesptr);
    shot.setInputNormals(cloud_normals_ptr);
    // shot.setSearchMethod(tree3); //设置近邻搜索算法
    //注意：此处使用的半径必须要大于估计表面法线时使用的半径!!!
    shot.setRadiusSearch(0.4);
    // shot.setKSearch(35);
    //计算关键点在其原始点云处的邻域特征
    // shot.setSearchSurface(cloud_ptr);
    shot.compute(*shot_descriptors);

    cout << "shot feature size : " << shot_descriptors->points.size() << endl;

    return shot_descriptors;
}

pcl::PointCloud<pcl::Narf36>::Ptr get_narf(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr)
{
    // --------------------
    // -----Parameters-----
    // --------------------
    float angular_resolution = 0.5f;
    float support_size = 0.2f;
    /*
        定义坐标轴正方向：
        CAMERA_FRAME：X轴向右，Y轴向下，Z轴向前；
        LASER_FRAME：X轴向前，Y轴向左，Z轴向上；
    */
    pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
    bool setUnseenToMaxRange = true;
    bool rotation_invariant = true;

    // -----------------------------------------------
    // -----Create RangeImage from the PointCloud-----
    // -----------------------------------------------
    float noise_level = 0.0;
    float min_range = 0.0f;
    int border_size = 1;

    pcl::PointCloud<pcl::PointWithViewpoint> far_ranges;
    Eigen::Affine3f scene_sensor_pose(Eigen::Affine3f::Identity());

    scene_sensor_pose = Eigen::Affine3f(Eigen::Translation3f(cloud_ptr->sensor_origin_[0],
                                                             cloud_ptr->sensor_origin_[1],
                                                             cloud_ptr->sensor_origin_[2])) *
                        Eigen::Affine3f(cloud_ptr->sensor_orientation_);

    pcl::RangeImage::Ptr range_image_ptr(new pcl::RangeImage);
    pcl::RangeImage &range_image = *range_image_ptr;
    range_image.createFromPointCloud(*cloud_ptr, angular_resolution, pcl::deg2rad(360.0f), pcl::deg2rad(180.0f),
                                     scene_sensor_pose, coordinate_frame, noise_level, min_range, border_size);
    // range_image.integrateFarRanges(far_ranges);
    if (setUnseenToMaxRange)
        range_image.setUnseenToMaxRange();

    // --------------------------------
    // -----Extract NARF keypoints-----
    // --------------------------------
    pcl::RangeImageBorderExtractor range_image_border_extractor;
    pcl::NarfKeypoint narf_keypoint_detector(&range_image_border_extractor);
    narf_keypoint_detector.setRangeImage(&range_image);
    narf_keypoint_detector.getParameters().support_size = support_size;
    // narf_keypoint_detector.getParameters ().add_points_on_straight_edges = true;
    // narf_keypoint_detector.getParameters ().distance_for_additional_points = 0.5;

    pcl::PointCloud<int> keypoint_indices;
    narf_keypoint_detector.compute(keypoint_indices);
    std::cout << "Found " << keypoint_indices.size() << " key points.\n";

    // ------------------------------------------------------
    // -----Extract NARF descriptors for interest points-----
    // ------------------------------------------------------
    std::vector<int> keypoint_indices2;
    keypoint_indices2.resize(keypoint_indices.size());
    for (unsigned int i = 0; i < keypoint_indices.size(); ++i) // This step is necessary to get the right vector type
        keypoint_indices2[i] = keypoint_indices[i];
    pcl::NarfDescriptor narf_descriptor(&range_image, &keypoint_indices2);
    narf_descriptor.getParameters().support_size = support_size;
    narf_descriptor.getParameters().rotation_invariant = rotation_invariant;
    pcl::PointCloud<pcl::Narf36>::Ptr narf_descriptors(new pcl::PointCloud<pcl::Narf36>);
    narf_descriptor.compute(*narf_descriptors);
    std::cout << "Extracted " << narf_descriptors->size() << " descriptors for "
              << keypoint_indices.size() << " keypoints.\n";
    return narf_descriptors;
}