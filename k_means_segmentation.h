#pragma once
#include"h_file/name_def.h"
#include<vector>
#include"h_file/interact_show.h"
//该算法，受摆放位置的影响很大，
//对正方形的鲁棒性稍好，对长方形，长宽比越大，效果越差



using namespace std;



void update_centroid(PointCloud::Ptr & cloud_in,vector<int> &k_indices,PointT &k_centroid) {
	int k_point_size = k_indices.size();
	k_centroid.x = 0;
	k_centroid.y = 0;
	k_centroid.z = 0;
	for (int i = 0; i <k_point_size; i++) {
		//cout << "k_indices[i]: " << k_indices[i] << endl;
		k_centroid.x += (float)cloud_in->points[k_indices[i]].x / k_point_size;
		k_centroid.y+= (float)cloud_in->points[k_indices[i]].y / k_point_size;
		k_centroid.z += (float)cloud_in->points[k_indices[i]].z / k_point_size;
	}
}

void get_two_point_dis(PointT &p0, PointT &p1,float &p_dis) {
	p_dis = sqrt(powf((p1.x - p0.x), 2.0) + powf((p1.y - p0.y), 2.0) + powf((p1.z - p0.z), 2.0));
	cout << "p1   p0" << p1.x << "  " << p1.y << "  " << p1.z << "  " << p0.x << "  " << p0.y << "  " << p0.z << endl;
	cout << "p_dis: " << p_dis << endl;
}




void k_means_segmentation(PointCloud::Ptr &cloud_in, PointCloud::Ptr &k0_cloud, PointCloud::Ptr &k1_cloud)
{
	//标准化
	PointT centroid0;
	PointT centroid1;
	float x_dis = 0.0;
	float y_dis = 0.0;
	float z_dis = 0.0;
	PointT temp_maxp=cloud_in->points[0];
	PointT temp_minp = cloud_in->points[0];
	PointT init_k0_point= cloud_in->points[0];
	PointT init_k1_point = cloud_in->points[0];
	PointCloud::Ptr nl_cloud(new PointCloud);
	//PointCloud::Ptr k0_cloud(new PointCloud);
	//PointCloud::Ptr k1_cloud(new PointCloud);

	vector<int> k0_indices, k1_indices;
	float temp_p_dis = 0.0; 
	float p_dis = 0.0;
	float k0_dis = 0.0;
	float k1_dis = 0.0;
	for(int i = 0; i < cloud_in->points.size(); i++) {
		if (cloud_in->points[i].x > temp_maxp.x) temp_maxp.x = cloud_in->points[i].x;
		if (cloud_in->points[i].y > temp_maxp.y) temp_maxp.y = cloud_in->points[i].y;
		if (cloud_in->points[i].z > temp_maxp.z) temp_maxp.z = cloud_in->points[i].z;
		if (cloud_in->points[i].x < temp_minp.x) temp_minp.x = cloud_in->points[i].x;
		if (cloud_in->points[i].y < temp_minp.y) temp_minp.y = cloud_in->points[i].y;
		if (cloud_in->points[i].z < temp_minp.z) temp_minp.z = cloud_in->points[i].z;
		temp_p_dis = powf((cloud_in->points[i].x - init_k0_point.x), 2.0) + powf((cloud_in->points[i].y - init_k0_point.y), 2.0) + powf((cloud_in->points[i].z - init_k0_point.z), 2.0);
		if (temp_p_dis > p_dis) {
			p_dis = temp_p_dis;
			init_k1_point = cloud_in->points[i];
		}
	}
	x_dis = temp_maxp.x - temp_minp.x;
	y_dis = temp_maxp.y - temp_minp.y;
	z_dis = temp_maxp.z - temp_minp.z;
	cout << "x,y,z-dis: " << x_dis << "  " << y_dis << "  " << z_dis << endl;




	nl_cloud->points.resize(cloud_in->width * cloud_in->height);
	cout << "nl_point: "<<nl_cloud->points.size() << endl;
	cout << "cloud_in: " << cloud_in->points.size() << endl;
	//标准化
	for (int i = 0; i<cloud_in->points.size(); i++) {
		//cout << "i: " << i << endl;
		nl_cloud->points[i].x = (cloud_in->points[i].x - temp_minp.x) / x_dis;
		nl_cloud->points[i].y = (cloud_in->points[i].y - temp_minp.y) / y_dis;
		nl_cloud->points[i].z = (cloud_in->points[i].z - temp_minp.z) / z_dis;
	}
	init_k1_point.x = (init_k1_point.x-temp_minp.x)/x_dis;
	init_k1_point.y = (init_k1_point.y - temp_minp.y) / y_dis;
	init_k1_point.z = (init_k1_point.z - temp_minp.z) / z_dis;
	init_k0_point = nl_cloud->points[0];
	centroid0 = init_k0_point;
	centroid1 = init_k1_point;

	PointT temp_ctd0;
	temp_ctd0.x = FLT_MAX;
	temp_ctd0.y = FLT_MAX;
	temp_ctd0.z = FLT_MAX;
	PointT temp_ctd1;
	temp_ctd1.x = FLT_MAX;
	temp_ctd1.y = FLT_MAX;
	temp_ctd1.z = FLT_MAX;

	float temp_dis_k0 = FLT_MAX;
	float temp_dis_k1 = FLT_MAX;


	int m = 0;
	//int k = 1000;
	while (temp_dis_k0 + temp_dis_k1) {
//	while (k--) {
		k0_indices.clear();
		k1_indices.clear();
		m++;
		//cout << m << ":  " << temp_dis_k0 + temp_dis_k1 << endl;	

		for (int i = 0; i < nl_cloud->points.size(); i++) {
			k0_dis = powf((nl_cloud->points[i].x - centroid0.x), 2.0) + powf((nl_cloud->points[i].y - centroid0.y), 2.0) + powf((nl_cloud->points[i].z - centroid0.z), 2.0);
			k1_dis = powf((nl_cloud->points[i].x - centroid1.x), 2.0) + powf((nl_cloud->points[i].y - centroid1.y), 2.0) + powf((nl_cloud->points[i].z - centroid1.z), 2.0);
			//k0_dis = powf((nl_cloud->points[i].x - centroid0.x), 2.0) + powf((nl_cloud->points[i].y - centroid0.y), 2.0);
			//k1_dis = powf((nl_cloud->points[i].x - centroid1.x), 2.0) + powf((nl_cloud->points[i].y - centroid1.y), 2.0);
			if (k0_dis <= k1_dis)k0_indices.push_back(i);
			else k1_indices.push_back(i);
		}
		update_centroid(nl_cloud, k0_indices, centroid0);
		update_centroid(nl_cloud, k1_indices, centroid1);

		get_two_point_dis(temp_ctd0, centroid0, temp_dis_k0);
		get_two_point_dis(temp_ctd1, centroid1, temp_dis_k1);
		cout <<endl<< m << ":  " << temp_dis_k0 <<"  "<< temp_dis_k1 << endl;

		temp_ctd0 = centroid0;
		temp_ctd1 = centroid1;
	}
	cout << "m: " << m << endl;

	k0_cloud->points.resize(k0_indices.size());
	k1_cloud->points.resize(k1_indices.size());
	for (int i = 0; i < k0_indices.size(); i++) k0_cloud->points[i] = cloud_in->points[k0_indices[i]];
	for (int i = 0; i < k1_indices.size(); i++) k1_cloud->points[i] = cloud_in->points[k1_indices[i]];

	interact_show(k0_cloud);
	interact_show(k1_cloud);




}
