#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>

using namespace std;

namespace pcl
{
	template <>
	struct SIFTKeypointFieldSelector<PointXYZ>
	{
		inline float
		operator()(const PointXYZ &p) const
		{
			return p.z;
		}
	};
}

pcl::PointCloud<pcl::PointXYZ>::Ptr get_harrisKeypoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr);
pcl::PointCloud<pcl::PointXYZ>::Ptr get_siftKeypoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr);
pcl::PointCloud<pcl::PointXYZ>::Ptr get_narfKeypoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr);