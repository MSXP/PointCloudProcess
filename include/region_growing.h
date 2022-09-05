//头文件
#include <pcl/io/pcd_io.h>
#include <pcl/search/kdtree.h>
#include <vector>
#include <queue>
#include <pcl/features/normal_3d.h>
#include <Eigen/Dense>
#include <cmath>
#include <ctime>
#include <pcl/visualization/pcl_visualizer.h>

using namespace pcl;
using namespace Eigen;
using namespace std;

typedef PointXYZ PointT;

class region_growing
{
public:
	region_growing();
	void setinput_point(PointCloud<PointT>::Ptr &point_cloud);
	void set_normal_curvature(float, float);
	void normal_estimation(int);
	vector<int> region_growing_one(int index_point);

private:
	PointCloud<PointT>::Ptr cloud;
	float curvature_threshold;
	float normal_threshold;
	int K_nebor_size;
	PointCloud<Normal>::Ptr normal_;
	vector<int> all_index;
};

region_growing::region_growing()
{
	curvature_threshold = 0.2;
	normal_threshold = 10.0;
	K_nebor_size = 10;
}

inline void region_growing::set_normal_curvature(float curvature_threshold1, float normal_threshold1)
{
	curvature_threshold = curvature_threshold1;
	normal_threshold = normal_threshold1;
}

void region_growing::setinput_point(PointCloud<PointT>::Ptr &point_cloud)
{
	cloud = point_cloud;
}

void region_growing::normal_estimation(int K_nebor_size)
{
	PointCloud<Normal>::Ptr normal(new PointCloud<Normal>);
	search::KdTree<PointT>::Ptr tree(new search::KdTree<PointT>);
	tree->setInputCloud(cloud);
	NormalEstimation<PointT, Normal> ne;
	ne.setInputCloud(cloud);
	ne.setSearchMethod(tree);
	ne.setKSearch(K_nebor_size);
	ne.compute(*normal);
	normal_ = normal;
}

vector<int> region_growing::region_growing_one(int index_point)
{
	search::KdTree<PointXYZ>::Ptr tree(new search::KdTree<PointXYZ>);
	tree->setInputCloud(cloud);
	float normal_threshold_real = cosf(normal_threshold / 180.0 * M_PI);
	queue<int> seed;
	seed.push(index_point);
	vector<int> point_label;
	vector<int> nebor_idx;
	vector<float> nebor_distance;
	point_label.resize(cloud->points.size(), -1);
	point_label[index_point] = 0;
	vector<int>().swap(all_index);
	while (!seed.empty())
	{
		int curr_seed = seed.front();
		seed.pop();
		int K_nebor(0);
		tree->nearestKSearch(cloud->points[curr_seed], K_nebor_size, nebor_idx, nebor_distance);
		while (K_nebor < nebor_idx.size())
		{
			int index_nebor = nebor_idx[K_nebor];
			if (point_label[index_nebor] != -1)
			{
				K_nebor++;
				continue;
			}
			bool is_a_seed = false;
			Map<Vector3f> vec_curr_seed(static_cast<float *>(normal_->points[curr_seed].normal));
			Map<Vector3f> vec_seed_nebor(static_cast<float *>(normal_->points[index_nebor].normal));
			float dot_normal = fabsf(vec_curr_seed.dot(vec_seed_nebor));
			if (dot_normal < normal_threshold_real)
			{
				is_a_seed = false;
			}
			else if (normal_->points[index_nebor].curvature > curvature_threshold)
			{
				is_a_seed = false;
			}
			else
			{
				is_a_seed = true;
			}
			if (!is_a_seed)
			{
				K_nebor++;
				continue;
			}
			all_index.push_back(index_nebor);
			point_label[index_nebor] = 0;
			if (is_a_seed)
			{
				seed.push(index_nebor);
			}
			K_nebor++;
		}
	}
	return all_index;
}