#include "include/keypoints_extraction.h"

void get_harrisKeypoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr)
{
    float r_normal = 0.1f;   //法向量估计的半径
    float r_keypoint = 0.2f; //关键点估计的近邻搜索半径

    typedef pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> ColorHandlerT3;

    pcl::PointCloud<pcl::PointXYZI>::Ptr Harris_keypoints(new pcl::PointCloud<pcl::PointXYZI>()); //存放最后的特征点提取结果
    //实例化一个Harris特征检测对象harris_detector
    pcl::HarrisKeypoint3D<pcl::PointXYZ, pcl::PointXYZI, pcl::Normal> *harris_detector = new pcl::HarrisKeypoint3D<pcl::PointXYZ, pcl::PointXYZI, pcl::Normal>;

    // harris_detector->setNonMaxSupression(true);
    harris_detector->setRadius(r_normal);         //设置法向量估计的半径
    harris_detector->setRadiusSearch(r_keypoint); //设置关键点估计的近邻搜索半径
    harris_detector->setInputCloud(cloud_ptr);    //设置输入点云
    // harris_detector->setNormals(normal_source);
    // harris_detector->setMethod(pcl::HarrisKeypoint3D<pcl::PointXYZRGB,pcl::PointXYZI>::LOWE);
    harris_detector->compute(*Harris_keypoints); //结果存放在Harris_keypoints
    cout << "Harris_keypoints number: " << Harris_keypoints->size() << endl;
    cout << "Points number: " << cloud_ptr->size() << endl;
    // 结果保存
    // writer.write<pcl::PointXYZI> ("Harris_keypoints.pcd",*Harris_keypoints,false);
    // pcl::io::savePCDFileASCII("Harris keypoints.pcd", *Harris_keypoints);

    //可视化点云
    pcl::visualization::PCLVisualizer viewer("Harris");
    viewer.setBackgroundColor(255, 255, 255);
    //原始点云可视化
    viewer.addPointCloud(cloud_ptr, "input_cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 0, "input_cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "input_cloud");
    // Harris_keypoints关键点可视化
    viewer.addPointCloud(Harris_keypoints, ColorHandlerT3(Harris_keypoints, 255.0, 0.0, 0.0), "harris_keypoints");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 8, "harris_keypoints");
    viewer.spin();
}

void get_siftKeypoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr)
{
    const float min_scale = 0.01;      //尺度空间中最小尺度的标准偏差
    const int n_octaves = 6;           //高斯金字塔中组的数目
    const int n_scales_per_octave = 4; //每组计算的尺度数目
    const float min_contrast = 0.01;   //设置关键点检测的阈值

    // SIFT关键点检测
    pcl::SIFTKeypoint<pcl::PointXYZ, pcl::PointWithScale> sift; //创建sift关键点检测对象
    pcl::PointCloud<pcl::PointWithScale> result;                // SIFT关键点提取结果
    sift.setInputCloud(cloud_ptr);                              //设置输入点云
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    sift.setSearchMethod(tree);                                //创建一个空的kd树对象tree，并把它传递给sift检测对象
    sift.setScales(min_scale, n_octaves, n_scales_per_octave); //指定搜索关键点的尺度范围
    sift.setMinimumContrast(min_contrast);                     //设置限制关键点检测的阈值
    sift.compute(result);                                      //执行sift关键点检测，保存结果在result
    cout << "sift_keypoints number: " << result.size() << endl;
    cout << "Points number: " << cloud_ptr->size() << endl;

    //类型转换
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZ>);
    copyPointCloud(result, *cloud_temp); //将点类型pcl::PointWithScale的数据转换为点类型pcl::PointXYZ的数据

    //可视化输入点云和关键点
    pcl::visualization::PCLVisualizer viewer("Sift");
    viewer.setBackgroundColor(255, 255, 255);
    viewer.addPointCloud(cloud_ptr, "input_cloud"); //在视窗中添加原始点云数据
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 0, "input_cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "input_cloud");
    viewer.addPointCloud(cloud_temp, "sift_keypoints"); //将SIFT关键点添加至视窗
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 8, "sift_keypoints");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 255, 0, 0, "sift_keypoints");
    viewer.spin();
}

void get_narfKeypoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr)
{
    // --------------------
    // -----Parameters-----
    // --------------------
    float angular_resolution = 0.5f;
    angular_resolution = pcl::deg2rad(angular_resolution);
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
    range_image.integrateFarRanges(far_ranges);
    if (setUnseenToMaxRange)
        range_image.setUnseenToMaxRange();

    // --------------------------------
    // -----Extract NARF keypoints-----
    // --------------------------------
    pcl::RangeImageBorderExtractor range_image_border_extractor;
    pcl::NarfKeypoint narf_keypoint_detector(&range_image_border_extractor);
    narf_keypoint_detector.setRangeImage(&range_image);
    narf_keypoint_detector.getParameters().support_size = support_size;
    narf_keypoint_detector.getParameters().add_points_on_straight_edges = true;
    // narf_keypoint_detector.getParameters ().distance_for_additional_points = 0.5;

    pcl::PointCloud<int> keypoint_indices;
    narf_keypoint_detector.compute(keypoint_indices);
    std::cout << "Found " << keypoint_indices.size() << " key points.\n";

    // 将深度图像的索引转换到点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_ptr(new pcl::PointCloud<pcl::PointXYZ>);//存放关键点指针
    pcl::PointCloud<pcl::PointXYZ>& keypoints = *keypoints_ptr;//存放关键点指针 的引用
    keypoints.resize(keypoint_indices.size());//设置点云数量
    for(size_t i=0;i<keypoint_indices.size();++i)//循环 将深度图像的对应关键点 提取的  关键点云 里面
        keypoints[i].getVector3fMap()=range_image[keypoint_indices[i]].getVector3fMap();

    // --------------------------------------------
    // -----Open 3D viewer and add point cloud-----
    // --------------------------------------------
    pcl::visualization::PCLVisualizer viewer("NARF");
    viewer.setBackgroundColor(255,255,255);
    // 实测:以下两种可视化结果相同
    // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> range_image_color_handler(range_image_ptr,0,0,0);
    // viewer.addPointCloud(range_image_ptr,range_image_color_handler,"range_image");
    // viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "range_image");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> point_cloud_color_handler(cloud_ptr, 0, 0, 0);
    viewer.addPointCloud(cloud_ptr, point_cloud_color_handler, "input_cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "input_cloud");
    // 添加坐标轴并初始化位置
    // viewer.addCoordinateSystem(1.0f);
    // viewer.initCameraParameters();
    // setViewerPose (viewer, range_image.getTransformationToWorldSystem ());

    // --------------------------
    // -----Show range image-----
    // --------------------------
    pcl::visualization::RangeImageVisualizer range_image_widget("Range_image");
    range_image_widget.showRangeImage(range_image);

    // -------------------------------------
    // -----Show keypoints in 3D viewer-----
    // -------------------------------------
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> keypoints_color_handler(keypoints_ptr, 255, 0, 0);
    viewer.addPointCloud<pcl::PointXYZ>(keypoints_ptr,keypoints_color_handler,"narf_keypoints");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,8,"narf_keypoints");
    // range_image_widget.spin();
    viewer.spin();
}