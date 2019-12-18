#pragma once
#include<iostream>
#include<vector>
#include<string>
#include<fstream>
#include<cmath>
#include<map>
#include"h_file/name_def.h"
#include<pcl/kdtree/kdtree_flann.h>
using namespace std;

const int noise = -2;
const int not_classified = -1;

class Point {
public:
	PointT pts;
	int cluster;
	int nn_pts;
	bool change_cluster=0;
	float get_dis(const Point &other_pts){
		return sqrt(powf((pts.x - other_pts.pts.x), 2.0) + powf((pts.y - other_pts.pts.y), 2.0) + powf((pts.z - other_pts.pts.z), 2.0));
	}
};

class DBSCAN {
public:
	//声明对象
	vector<vector<int>> cluster;
	vector<vector<int>> adjpoints;
	vector<Point> points;
	int n;
	int min_pts;
	int size;
	float eps;
	int clusterIdx;
	PointCloud::Ptr k0;
	PointCloud::Ptr k1;
	PointCloud::Ptr cloud_in;

	//给对象分配相应的空间
	//所有要用的对象都必须分配
	DBSCAN(float eps, int min_pts, vector<Point>points, PointCloud::Ptr k0,PointCloud::Ptr k1,PointCloud::Ptr cloud_in) {
		//this->n = n;
		//->表示对对象的调用
		//类似外部空间中"."的使用
		this->eps = eps;
		this->min_pts = min_pts;
		this->points = points;
		this->size = (int)points.size();
		this->clusterIdx = -1;
		this->k0 = k0;
		this->k1 = k1;
		this->cloud_in = cloud_in;
		adjpoints.resize(size);
		cluster.resize(size);//必须初始化，不然运行cluster.push_back()会出现this是nullptr的中断错误
	}
	////检查近邻点，并将近邻点压入向量
	////n*n的复杂度，耗时较长可以用KDtree改进
	////实验后暴力搜索法耗时50s,kdtree耗时0.1s
	// void checkNearPoints() {
	//	clock_t cnp_0, cnp_1;
	//	cnp_0 = clock();
	//	cout << "checkNearPoints: true" << endl;
	//	for (int i = 0; i < size; i++) {
	//		points[i].cluster = i;
	//		for (int j = 0; j < size; j++) {
	//			if (i == j)continue;
	//			if (points[i].get_dis(points[j]) <= eps) {
	//				points[i].nn_pts++;
	//				adjpoints[i].push_back(j);
	//			}
	//		}
	//	}
	//	cnp_1 = clock();
	//	cout << "time of check near points: " << (cnp_1 - cnp_0) / 1000 << "s\n";
	//}
	
	
	//利用kdtree-flann的方法寻找最近点
	//36000点用时0.1s,若用暴力搜索方法用时50s
	 void checkNearPoints() {
		clock_t cnp_0, cnp_1;
		cnp_0 = clock();
		vector<float> pp_distance;
		vector<int>::iterator itr;
		pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
		kdtree.setInputCloud(cloud_in);
		for (int i = 0; i < size; i++) {
			points[i].cluster = i;//先将各个点的类值区分开，便于后面聚类
			//cout << "eps: " << eps << endl;
			adjpoints[i].clear();		
			kdtree.radiusSearch(cloud_in->points[i], eps, adjpoints[i], pp_distance);
			points[i].nn_pts = adjpoints[i].size()-1;//剔除点本身
			itr = adjpoints[i].begin();
			adjpoints[i].erase(itr);
		//	cout << i<<"nn_pts:  " << points[i].nn_pts << endl;
			//cout << i << "nm of adjpoints:  " << adjpoints[i].size() << endl;
		//	cout << i << "adjpoints:  " << adjpoints[i][0]<<"  "<<adjpoints[i][1]<<"  "<<adjpoints[i][2] << "  " << adjpoints[i][3] << endl;
			//cout << "pp_distance size: " << pp_distance.size()<<"  pp:  "<<pp_distance[0] <<"  "<<pp_distance[1]<<"   "<<pp_distance[pp_distance.size()]<< endl;
		}
		cnp_1 = clock();
		cout << "time of check near points: " << (float)(cnp_1 - cnp_0) / 1000 << "s\n";
	}





	bool iscore_pts(int idx) {
		return points[idx].nn_pts >= min_pts;
	}

	void def(int idx) {
		int temp_idx;
		int temp_k;
		bool change = 0;
		for (int i = 0; i < adjpoints[idx].size(); i++) {
			if (points[adjpoints[idx][i]].change_cluster) {
				temp_idx = i;
				temp_k = points[adjpoints[idx][i]].cluster;
				change = 1;
				break;
			}
		}
		if (change) {
			for (int i = 0; i < adjpoints[idx].size(); i++) {
				points[adjpoints[idx][i]].cluster = temp_k;
				points[idx].cluster = temp_k;
			}
		}
		else {
			for (int i = 0; i < adjpoints[idx].size(); i++) {
				points[adjpoints[idx][i]].cluster = points[idx].cluster;
				points[adjpoints[idx][i]].change_cluster = 1;
			}

		}
	}

	void run() {
		cout << "run start" << endl;
		checkNearPoints();
		int k = 0;
		int temp_k;
		for (int i = 0; i < size; i++) {
		//	if (points[i].cluster != not_classified) continue;
			if (iscore_pts(i)) {
				def(i);
				//k++;
			}
		}
		//cout << "核心点的数量: " << k << endl;
		cout << "run end" << endl;
	}
	void get_cluster() {
		cout << "get_cluster start" << endl;
		bool add_cluster = 0;
		vector<int>temp_cluster;
		temp_cluster.push_back(points[0].cluster);
		cout << "size: " << size << endl;
		//int test1 = 0;
		for (int i = 0; i < size; i++) {
			
			for (int j = 0; j < temp_cluster.size(); j++) {
				if (points[i].cluster == temp_cluster[j]) {
					int _cluster = find(temp_cluster.begin(), temp_cluster.end(), points[i].cluster) - temp_cluster.begin();
					//cout << "_cluster: " << _cluster << endl;
					cluster[_cluster].push_back(i);
					//cout << "cluster: " << i << endl;
					add_cluster = 1;
					//test1++;
				}
				
			}
			if (!add_cluster) {		
				temp_cluster.push_back(points[i].cluster);
				cluster[temp_cluster.size() - 1].push_back(i);
			}
			
			add_cluster = 0;
		}
		
		//cout << "cluster0: " << cluster[0].size() << "cluster1: " << cluster[1].size() << "cluster2: " << cluster[2].size() << endl;
		for (int k = 0; k < 2; k++) {
			for (int i = cluster.size() - 1; i >0; i--) {
				if (cluster[i].size() >cluster[i - 1].size())
					cluster[i].swap(cluster[i - 1]);
			}
		}
	cout << "get cluster end: " << endl;

	}


	void transto_cloud() {
		int size0 = cluster[0].size();
		int size1 = cluster[1].size();
		cout << "size0: " << size0 << "size1: " << size1 << endl;
		k0->points.resize(size0);
		k1->points.resize(size1);
		for (int i = 0; i < size0; i++) {
			k0->points[i] = points[cluster[0][i]].pts;
		}
		for (int i = 0; i < size1; i++) {
			k1->points[i] = points[cluster[1][i]].pts;
		}
	}
};


void DBSCAN_segmentation(PointCloud::Ptr &cloud_in, PointCloud::Ptr &cloud_k0, PointCloud::Ptr &cloud_k1) {
	vector<Point> _points;
	for (int i = 0; i < cloud_in->points.size(); i++) {
		Point _pts;
		_pts.pts = cloud_in->points[i];
		_points.push_back(_pts);
	}
	DBSCAN dbscan(7, 29, _points, cloud_k0, cloud_k1,cloud_in);
	dbscan.run();
	dbscan.get_cluster();
	dbscan.transto_cloud();
}