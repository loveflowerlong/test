#pragma once
#include<pcl/filters/passthrough.h>
#include"name_def.h"
void delete_floor(PointCloud::Ptr &cloud_in, PointCloud::Ptr &cloud_out) {
	pcl::PassThrough<PointT> pass;
	pass.setInputCloud(cloud_in);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(500.0, 800.0);
	pass.setFilterLimitsNegative(false);
	pass.filter(*cloud_out);
}

