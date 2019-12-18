#pragma once
#include"name_def.h"
#include<pcl/segmentation/sac_segmentation.h>
#include<pcl/sample_consensus/method_types.h>
#include<pcl/sample_consensus/model_types.h>
#include<pcl/filters/extract_indices.h>
#include<pcl/ModelCoefficients.h>
#include<iostream>


void extract_plane(PointCloud::Ptr &cloud_in, PointCloud::Ptr &cloud_out) {
	clock_t t_s, t_e;
	t_s = clock();
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);//提取的是平面点的索引
	pcl::SACSegmentation<PointT> seg;
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(5.0);
	seg.setInputCloud(cloud_in);
	seg.segment(*inliers, *coefficients);
	//从索引提取出具体的点
	pcl::ExtractIndices<PointT> extract;
	extract.setInputCloud(cloud_in);
	extract.setIndices(inliers);
	extract.setNegative(false);
	extract.filter(*cloud_out);

	t_e = clock();
	std::cout << "time of extract plane: " << t_e - t_s << std::endl;
}
