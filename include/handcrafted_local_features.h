#include <pcl/point_types.h>
//#include <pcl/features/pfh.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/shot.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/shot_lrf.h>
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/features/narf_descriptor.h>
#include <pcl/range_image/range_image.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/features/normal_3d.h>                 //法线特征
#include <pcl/visualization/histogram_visualizer.h> //直方图的可视化
#include <pcl/visualization/pcl_plotter.h>          // 直方图的可视化 方法2

using namespace std;

pcl::PointCloud<pcl::FPFHSignature33>::Ptr get_fpfh(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr);
pcl::PointCloud<pcl::SHOT352>::Ptr get_shot(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr);
pcl::PointCloud<pcl::Narf36>::Ptr get_narf(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr);