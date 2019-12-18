#pragma once
#include"h_file/name_def.h"
#include<float.h>
#include<array>
#include<vector>
#include<array>
#include<pcl/common/centroid.h>

//输入点云为滤波/降维后的较为干净的点云
//返回值rotation_angle为点云需要逆时针旋转的角度，选装轴当前点云的x=0,y=0处的z轴

inline void get_gesture_through_test_max_point(PointCloud::Ptr &cloud_in, float &rotation_angle) {

	//输入点云没有的离群点的点云
	//如果噪声较多，可以尝试使用统计滤波去除离群点，也可以使用k-means聚类获得相对干净的点

	float x_max = cloud_in->points[0].x;
	float x_min = cloud_in->points[0].x;
	Eigen::Vector4f pcaCentroid = Eigen::Vector4f::Identity();
	pcl::compute3DCentroid(*cloud_in, pcaCentroid);




	for (int i = 0; i < cloud_in->points.size(); i++) {
		if (cloud_in->points[i].x > x_max)x_max = cloud_in->points[i].x;
		if (cloud_in->points[i].x < x_min) x_min = cloud_in->points[i].x;
	}

	std::array<float, 450>edge_point_temp;
	std::array<float, 450>edge_point_temp_tox;

	edge_point_temp.fill(-10000);
	edge_point_temp_tox.fill(0.0);


	std::array<float, 90>edge_point;
	std::array<float, 89> adjacent_interval;
	float x_cell = (x_max - x_min) / 450;
	std::cout << "x_cell: " << x_max << "   " << x_min << "   " << x_cell << std::endl;
	int temp_edge;
	for (int j = 0; j < cloud_in->points.size(); j++) {
		temp_edge = floor((cloud_in->points[j].x - x_min) / x_cell);
		if (temp_edge == 450)temp_edge = 449;
		if (cloud_in->points[j].y > edge_point_temp[temp_edge])
			edge_point_temp[temp_edge] = cloud_in->points[j].y;
		edge_point_temp_tox[temp_edge] = cloud_in->points[j].x;

	}
	/*for(int nn=0;nn<450;nn++)
	std::cout << edge_point_temp[nn] << std::endl;*/


	float edge_point_max = -10000;
	int edge_point_max_indices;
	std::vector<float> sort_temp;
	std::vector<float> sort_temp_tox;

	float mean_interval = 0.0;
	for (int p = 0; p < 90; p++) {
		//sort_temp.push_back(temp2);
		for (int q = 0; q < 5; q++) {
			sort_temp.push_back(edge_point_temp[5 * p + q]);
			sort_temp_tox.push_back(edge_point_temp_tox[5 * p + q]);
		}

		for (int m = 5; m > 0; m--) {
			for (int o = 0; o < m - 1; o++) {
				if (sort_temp[o] > sort_temp[o + 1])
					std::swap(sort_temp[o], sort_temp[o + 1]);
				std::swap(sort_temp_tox[o], sort_temp_tox[o + 1]);

			}
		}
		float temp = 0.0;
		if (p > 0) {
			temp = fabs(edge_point[p-1] - sort_temp[2]) ;
			adjacent_interval[p - 1] = temp;
			mean_interval +=temp / 89;
		}
		edge_point[p] = sort_temp[2];
		
		std::cout << "90个点 " << edge_point[p] << std::endl;

		if (edge_point[p] > edge_point_max) {
			edge_point_max = edge_point[p];
			edge_point_max_indices = p;
		}
		if (edge_point[p] == edge_point_max) {
			if (abs(edge_point_max_indices - p) > 2)
				std::cerr << "出现多个间隔较大的最大值点" << std::endl;
		}
		sort_temp.clear();

	}

	std::cout << "mean_interval: "<<mean_interval << std::endl;
	int longline_k=0;
	int sum = 0;
	//vector<float> longline_interval;
	float mean_longline_interval=0;
	for (int i = 0; i < 89; i++) {
		if (adjacent_interval[i] < mean_interval) {
			longline_k++;
			sum += i;
			mean_longline_interval += adjacent_interval[i];	
			std::cout << "i 大约平均距离的点索引" << i << std::endl;
		}

	}
	mean_longline_interval /= (longline_k+1);
	float alfa;
	alfa = (atanf(mean_longline_interval / (x_cell * 5)))*0.5;//0.5根据实际效果进行的人为参数调整
	if ((sum/longline_k)>44)rotation_angle = 1 * alfa;
	else rotation_angle = -1* alfa;
	std::cout << "mean_longline_interval: " << mean_longline_interval << endl <<
		"alfa: " << rotation_angle << endl;

	//
	////需要绕z轴旋转-1*alfa
	////alfa = acosf((edge_point_max - pcaCentroid[1])/sqrt(powf((average_y - pcaCentroid[1]), 2) + powf((floor((pcaCentroid[0] - x_min) / x_cell) - edge_point_max_indices/2), 2)));
	////zai外部写一个判断函数，若旋转后误差较大，可以将原电云旋转30度后再次调用此函数

}