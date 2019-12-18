#pragma once
//#include<pcl/filters/statistical_outlier_removal.h>
#include<pcl/filters/radius_outlier_removal.h>
void radius_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filter,float radius,int minpoint){
pcl::RadiusOutlierRemoval<pcl::PointXYZ> ror;
ror.setInputCloud(cloud_in);
ror.setRadiusSearch(radius);
ror.setMinNeighborsInRadius(minpoint);
ror.filter(*cloud_filter);
}
