#include<pcl/io/pcd_io.h>
#include<pcl/point_types.h>
#include"k_means_segmentation.h"
#include"h_file/name_def.h"
#include"h_file/delete_floor.h"
#include"h_file/radius_filter.h"
#include<vector>
#include<pcl/filters/voxel_grid.h>
#include"h_file/interact_show.h"
#include"DBSCAN_segmentation.h"
using namespace std;
int main() {
	PointCloud::Ptr cloud_in(new PointCloud);
	pcl::io::loadPCDFile("1.pcd", *cloud_in);
	vector<int>indices;
	pcl::removeNaNFromPointCloud(*cloud_in, *cloud_in, indices);
	pcl::VoxelGrid<PointT> voxel;
	//voxel.setLeafSize(10.0, 10.0, 500.0);
	voxel.setLeafSize(2.0, 2.0, 50.0);
	voxel.setInputCloud(cloud_in);
	voxel.filter(*cloud_in);
	delete_floor(cloud_in,cloud_in);
	radius_filter(cloud_in,cloud_in,4.5,13);
	interact_show(cloud_in);
	cout << "size of cloud_in after filter: " << cloud_in->points.size()<<endl;
	PointCloud::Ptr cloud_k0(new PointCloud);
	PointCloud::Ptr cloud_k1(new PointCloud);
	//k_means_segmentation(cloud_in, cloud_k0, cloud_k1);
	DBSCAN_segmentation(cloud_in, cloud_k0, cloud_k1);
	interact_show(cloud_k0);
	interact_show(cloud_k1);
	system("pause");
	return 0;



}