#pragma once
#include<pcl/keypoints/harris_3d.h>
#include<pcl/features/normal_3d.h>
void harris_search(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out) {
	pcl::HarrisKeypoint3D<pcl::PointXYZ, pcl::PointXYZI, pcl::Normal> harris;
	cout << "cloud_in:" << cloud_in->size() << endl;
	harris.setInputCloud(cloud_in);

	pcl::PointCloud<pcl::Normal>::Ptr harris_normal(new pcl::PointCloud<pcl::Normal>);
	pcl::NormalEstimation<pcl::PointXYZ,pcl::Normal> ne;
	ne.setInputCloud(cloud_in);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	ne.setSearchMethod(tree);
	ne.setKSearch(9);
	//ne.setRadiuSearch(2.0);
	ne.compute(*harris_normal);
	
	harris.setNormals(harris_normal);
	harris.setRadius(0.02);
	harris.setThreshold(0.01);
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out_i(new pcl::PointCloud<pcl::PointXYZI>);
	harris.compute(*cloud_out_i);
	cout << "cloud_out_i: " << cloud_out_i->size() << endl;
	cloud_out->resize(cloud_out_i->height*cloud_out_i->width);
	cloud_out->clear();
	for (int i = 0; i < cloud_out->size(); ++i) {
		cloud_out->points[i].x = cloud_out_i->points[i].x;
		cloud_out->points[i].y = cloud_out_i->points[i].y;
		cloud_out->points[i].y = cloud_out_i->points[i].y;
	}
	cout << "harris_out: " << cloud_out->size() << endl;
}
