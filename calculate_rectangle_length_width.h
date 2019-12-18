#pragma once
#include"name_def.h"
#include<vector>
#include<iostream>

void calculate_rectangle_length_width(PointCloud::Ptr &cloud_in, float &length, float &width) {
	//遍历获取最大最小点（4个点），还需改进，鲁棒性较差


	int p_size = cloud_in->points.size();
	PointT x_max, x_min, y_max, y_min;
	x_max = cloud_in->points[0];
	x_min = cloud_in->points[0];
	y_max = cloud_in->points[0];
	y_min = cloud_in->points[0];
	for (int i = 0; i < p_size; i++) {

		if (cloud_in->points[i].x >= x_max.x) x_max = cloud_in->points[i];
		if (cloud_in->points[i].x <= x_min.x)x_min = cloud_in->points[i];
		if (cloud_in->points[i].y >= y_max.y)  y_max = cloud_in->points[i];
		if (cloud_in->points[i].y <= y_min.y) y_min = cloud_in->points[i];

	}
	std::cout << "x_max:" << x_max << std::endl << "x_min:" << x_min << std::endl << "y_min:" << y_min << std::endl << "y_max:" << y_max << std::endl;
	std::vector<float> v_k;
	std::vector<float> v_b;
	//float k, b;
	v_k.push_back((x_max.y - y_max.y) / (x_max.x - y_max.x));
	v_b.push_back(x_max.y - v_k[0] * x_max.x);

	v_k.push_back((x_max.y - y_min.y) / (x_max.x - y_min.x));
	v_b.push_back(x_max.y - v_k[1] * x_max.x);

	v_k.push_back((x_min.y - y_min.y) / (x_min.x - y_min.x));
	v_b.push_back(x_min.y - v_k[3] * x_min.x);

	v_k.push_back((x_min.y - y_max.y) / (x_min.x - y_max.x));
	v_b.push_back(x_min.y - v_k[2] * x_min.x);



	std::vector<float> search_k;
		
	for (int j = 0; j < (v_k.size()); j++) {
		if (j == v_k.size()) {
			search_k.push_back(v_k[0] * v_k[j]);
		}
		search_k.push_back(v_k[j] * v_k[j + 1]);
	}





	int indices;
	float temp = fabs(search_k[0] + 1.0);
	for (int q = 0; q < search_k.size(); q++) {
		if (fabs(search_k[q] + 1.0) <= temp) {
			temp = fabs(search_k[q] + 1.0);
			indices = q;
		}
		std::cout << "q" << indices << std::endl;
	}
	//float length, width;
	PointT corner1, corner2, corner3;
	int dir_a, dir_b;
	if (indices = 0) {
		dir_a = 0; dir_b = 1; corner1 = x_max; corner2 = y_max; corner3 = y_min;
		length = sqrt(pow((corner1.x - corner2.x), 2) + pow((corner1.y - corner2.y), 2));
		width = sqrt(pow((corner1.x - corner3.x), 2) + pow((corner1.y - corner3.y), 2));
	}
	if (indices = 1) {
		dir_a = 1; dir_b = 2; corner1 = x_max; corner2 = x_min; corner3 = y_min;
		length = sqrt(pow((corner1.x - corner3.x), 2) + pow((corner1.y - corner3.y), 2));
		width = sqrt(pow((corner2.x - corner3.x), 2) + pow((corner2.y - corner3.y), 2));
	}
	if (indices = 2) {
		dir_a = 2; dir_b = 3; corner1 = x_min; corner2 = y_max; corner3 = y_min;
		length = sqrt(pow((corner1.x - corner2.x), 2) + pow((corner1.y - corner2.y), 2));
		width = sqrt(pow((corner1.x - corner3.x), 2) + pow((corner1.y - corner3.y), 2));
	}
	if (indices = 3) {
		dir_a = 3; dir_b = 0; corner1 = x_max; corner2 = x_min; corner3 = y_max;
		length = sqrt(pow((corner1.x - corner3.x), 2) + pow((corner1.y - corner3.y), 2));
		width = sqrt(pow((corner2.x - corner3.x), 2) + pow((corner2.y - corner3.y), 2));
	}

	float k0 = v_k[dir_b];
	float b0 = v_b[dir_b];
	float k1 = v_k[dir_a];
	float b1 = v_b[dir_a];
	float temp2;

	if (length < width) { temp2 = width; width = length; length = temp2; }
	std::cout << "length:" << length << std::endl << "width: " << width << std::endl;






}




inline void calculate_rectangle_length_width(PointCloud::Ptr &cloud_in, 
	float &length, float &width, PointT &corner1,PointT &corner2,PointT &corner3,
	float &k0,float &b0, float &k1, float &b1) {
	//遍历获取最大最小点（4个点），还需改进，鲁棒性较差


	int p_size = cloud_in->points.size();
	PointT x_max, x_min, y_max, y_min;
	x_max = cloud_in->points[0];
	x_min = cloud_in->points[0];
	y_max = cloud_in->points[0];
	y_min = cloud_in->points[0];
	for (int i = 0; i < p_size; i++) {

		if (cloud_in->points[i].x >= x_max.x) x_max = cloud_in->points[i];
		if (cloud_in->points[i].x <= x_min.x)x_min = cloud_in->points[i];
		if (cloud_in->points[i].y >= y_max.y)  y_max = cloud_in->points[i];
		if (cloud_in->points[i].y <= y_min.y) y_min = cloud_in->points[i];

	}
	std::cout << "x_max:" << x_max << std::endl << "x_min:" << x_min << std::endl << "y_min:" << y_min << std::endl << "y_max:" << y_max << std::endl;
	std::vector<float> v_k;
	std::vector<float> v_b;
	//float k, b;
	v_k.push_back((x_max.y - y_max.y) / (x_max.x - y_max.x));
	v_b.push_back(x_max.y - v_k[0] * x_max.x);

	v_k.push_back((x_max.y - y_min.y) / (x_max.x - y_min.x));
	v_b.push_back(x_max.y - v_k[1] * x_max.x);

	v_k.push_back((x_min.y - y_min.y) / (x_min.x - y_min.x));
	v_b.push_back(x_min.y - v_k[3] * x_min.x);

	v_k.push_back((x_min.y - y_max.y) / (x_min.x - y_max.x));
	v_b.push_back(x_min.y - v_k[2] * x_min.x);



	std::vector<float> search_k;

	for (int j = 0; j < (v_k.size()); j++) {
		if (j == v_k.size()) {
			search_k.push_back(v_k[0] * v_k[j]);
		}
		search_k.push_back(v_k[j] * v_k[j + 1]);
	}





	int indices;
	float temp = fabs(search_k[0] + 1.0);
	for (int q = 0; q < search_k.size(); q++) {
		if (fabs(search_k[q] + 1.0) <= temp) {
			temp = fabs(search_k[q] + 1.0);
			indices = q;
		}
		std::cout << "q" << indices << std::endl;
	}
	//float length, width;
	//PointT corner1, corner2, corner3;
	int dir_a, dir_b;
	if (indices = 0) {
		dir_a = 0; dir_b = 1; corner1 = x_max; corner2 = y_max; corner3 = y_min;
		length = sqrt(pow((corner1.x - corner2.x), 2) + pow((corner1.y - corner2.y), 2));
		width = sqrt(pow((corner1.x - corner3.x), 2) + pow((corner1.y - corner3.y), 2));
	}
	if (indices = 1) {
		dir_a = 1; dir_b = 2; corner1 = x_max; corner2 = x_min; corner3 = y_min;
		length = sqrt(pow((corner1.x - corner3.x), 2) + pow((corner1.y - corner3.y), 2));
		width = sqrt(pow((corner2.x - corner3.x), 2) + pow((corner2.y - corner3.y), 2));
	}
	if (indices = 2) {
		dir_a = 2; dir_b = 3; corner1 = x_min; corner2 = y_max; corner3 = y_min;
		length = sqrt(pow((corner1.x - corner2.x), 2) + pow((corner1.y - corner2.y), 2));
		width = sqrt(pow((corner1.x - corner3.x), 2) + pow((corner1.y - corner3.y), 2));
	}
	if (indices = 3) {
		dir_a = 3; dir_b = 0; corner1 = x_max; corner2 = x_min; corner3 = y_max;
		length = sqrt(pow((corner1.x - corner3.x), 2) + pow((corner1.y - corner3.y), 2));
		width = sqrt(pow((corner2.x - corner3.x), 2) + pow((corner2.y - corner3.y), 2));
	}

	//float k0 = v_k[dir_b];
	//float b0 = v_b[dir_b];
	//float k1 = v_k[dir_a];
	//float b1 = v_b[dir_a];
	 k0 = v_k[dir_b];
	 b0 = v_b[dir_b];
	 k1 = v_k[dir_a];
	 b1 = v_b[dir_a];
	float temp2;

	if (length < width) { temp2 = width; width = length; length = temp2; }
	std::cout << "length:" << length << std::endl << "width: " << width << std::endl;






}
