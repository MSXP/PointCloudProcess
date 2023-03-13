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
	region_growing(float, float, int, int);
	void setinput_point(PointCloud<PointT>::Ptr &point_cloud);
	void normal_estimation(int);
	vector<int> region_growing_one(int index_point);
	vector<int> get_all_index();

private:
	PointCloud<PointT>::Ptr cloud;
	float curvature_threshold;
	float normal_threshold;
	int K_nebor_size;
	int one_region_size;
	PointCloud<Normal>::Ptr normal_;
	vector<int> all_index;
};

region_growing::region_growing()
{
	curvature_threshold = 0.2;
	normal_threshold = 10.0; //角度值
	K_nebor_size = 10;
	one_region_size = 32;
}

region_growing::region_growing(float curvature_threshold1, float normal_threshold1, int K_nebor_size1, int one_region_size1)
{
	curvature_threshold = curvature_threshold1;
	normal_threshold = normal_threshold1; //角度值
	K_nebor_size = K_nebor_size1;
	one_region_size = one_region_size1;
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
	all_index.push_back(index_point);
	vector<int> point_label;
	vector<int> nebor_idx;
	vector<float> nebor_distance;
	point_label.resize(cloud->points.size(), -1);
	point_label[index_point] = 0;
	int point_num(1);
	// 每次清空是为了可视化每个区域
	// vector<int>().swap(all_index);
	// cout << normal_->points[index_point].curvature << "   ";
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
			Map<Vector3f> vec_curr_seed(static_cast<float *>(normal_->points[curr_seed].normal));
			Map<Vector3f> vec_seed_nebor(static_cast<float *>(normal_->points[index_nebor].normal));
			float dot_normal = fabsf(vec_curr_seed.dot(vec_seed_nebor));
			// 夹角为0度，对应阈值1；夹角为90度，对应阈值为0
			if (dot_normal > normal_threshold_real)
			{
				all_index.push_back(index_nebor);
				point_label[index_nebor] = 0;
				point_num++;
				if (normal_->points[index_nebor].curvature < curvature_threshold)
				{
					seed.push(index_nebor);
				}
			}
			if(point_num == one_region_size){
				queue<int>().swap(seed);
				break;
			}
			K_nebor++;
		}
	}
	// 通过法线和曲率筛选的点数量不够时，将还未选择的近邻点添加进去
	tree->nearestKSearch(cloud->points[index_point], one_region_size, nebor_idx, nebor_distance);
	int K_nebor(0);
	while(point_num < one_region_size){
		if(point_label[nebor_idx[K_nebor]] == -1){
			all_index.push_back(nebor_idx[K_nebor]);
			point_num++;
		}
		K_nebor++;
	}
	return all_index;
}

vector<int> region_growing::get_all_index(){
	return all_index;
}