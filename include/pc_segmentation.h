#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/extract_clusters.h>

using namespace std;

void euclidean_clustering(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr);
void region_growing(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr);