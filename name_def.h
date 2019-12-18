#pragma once
#include<pcl/point_cloud.h>
#include<pcl/point_types.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::Normal NormalT;
typedef pcl::PointCloud<NormalT> PointCloudNormal;
