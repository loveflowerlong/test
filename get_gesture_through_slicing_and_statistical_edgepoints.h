#pragma once
#include"h_file/name_def.h"
#include<float.h>
#include<array>
#include<vector>
#include<array>
//�������Ϊ�˲�/��ά��Ľ�Ϊ�ɾ��ĵ���
//����ֵrotation_angleΪ������Ҫ��ʱ����ת�ĽǶȣ�ѡװ�ᵱǰ���Ƶ�x=0,y=0����z��
void get_gesture_through_slicing_and_statistical_edgepoints(PointCloud::Ptr &cloud_in,float &rotation_angle) {
	 
	//�������û�е���Ⱥ��ĵ���
	//��������϶࣬���Գ���ʹ��ͳ���˲�ȥ����Ⱥ�㣬Ҳ����ʹ��k-means��������Ըɾ��ĵ�

	float x_max = cloud_in->points[0].x;
	float x_min=cloud_in->points[0].x;

	for (int i = 0; i < cloud_in->points.size(); i++) {
		if (cloud_in->points[i].x > x_max)x_max = cloud_in->points[i].x;
		if (cloud_in->points[i].x < x_min) x_min = cloud_in->points[i].x;
	}

	std::array<float, 450>edge_point_temp;
	edge_point_temp.fill( -10000 );

	std::array<float, 90>edge_point;
	float x_cell=(x_max - x_min) / 450;
	std::cout << "x_cell: " << x_max<<"   "<<x_min <<"   "<<x_cell << std::endl;
	int temp_edge;
	for (int j = 0; j < cloud_in->points.size(); j++) {
		temp_edge = floor((cloud_in->points[j].x - x_min) / x_cell);
		if (temp_edge == 450)temp_edge = 449;
		if(cloud_in->points[j].y>edge_point_temp[temp_edge])
		edge_point_temp[temp_edge] = cloud_in->points[j].y;	
	}
	/*for(int nn=0;nn<450;nn++)
	std::cout << edge_point_temp[nn] << std::endl;*/


	float edge_point_max=-10000;
	int edge_point_max_indices;
	std::vector<float> sort_temp;

	for (int p = 0; p < 90; p++) {
		//sort_temp.push_back(temp2);
		for (int q = 0; q < 5; q++) {
			sort_temp.push_back(edge_point_temp[5 * p + q]);		
		}

		for (int m = 5; m > 0; m--) {
			for (int o = 0; o < m-1; o++) {
				if(sort_temp[o] > sort_temp[o + 1])
				std::swap(sort_temp[o], sort_temp[o + 1]);
			}
		}
		edge_point[p] = sort_temp[2];
		std::cout << "90���� " << edge_point[p] << std::endl;

		if (edge_point[p] > edge_point_max) {
			edge_point_max = edge_point[p];
			edge_point_max_indices = p;
		}
		sort_temp.clear();
	}

	float gesture;
	gesture = (float)(edge_point_max_indices)/ 180 * M_PI;
	//gesture = (float)61 / 180;
	std::cout << "gesture of the plane is " << edge_point_max_indices<<"    "<< gesture << " degree��" << std::endl;
	rotation_angle = gesture;

}