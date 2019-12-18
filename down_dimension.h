#pragma once
#include"name_def.h"
void down_dimension(PointCloud::Ptr &cloud_in,PointCloud::Ptr &cloud_out) {
	*cloud_out = *cloud_in;
	for (int i = 0; i < cloud_in->size(); i++) {
		cloud_out->points[i].z = 0;

}
}
