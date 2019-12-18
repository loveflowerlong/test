#pragma once
#include"h_file/name_def.h"
#include<array>


void get_sum_err(float theta0, float theta1,std::vector<float> &y,std::vector<float> &x,float &sum_err) {
	sum_err = 0.0;
	//std::cout << "循环中 sum_err: " << sum_err << "\n";
	float h_hat = 0.0;
	for (int i = 0; i <y.size(); i++) {
		
		
		h_hat = theta0 + theta1*x[i];
		sum_err += powf((y[i] - h_hat),2.0);
	}
	sum_err /= (float)(2.0 * y.size());

}

void get_normalize_data(std::vector<float> &y, std::vector<float> &x) {
	float mean_y = 0.0;
	float mean_x = 0.0;
	for (int i = 0; i < y.size(); i++) {
		mean_y += y[i] / y.size();
		mean_x += x[i] / y.size();
	}
	std::vector<float> y_normal;
	std::vector<float> x_normal;
	float temp_y = 0.0;
	float temp_x = 0.0;
	for (int j = 0; j < y.size(); j++) {
		temp_y = (float)(y[j] - mean_y);
		temp_x = (float)(x[j] - mean_x);
		y_normal.push_back(temp_y);
		x_normal.push_back(temp_x);
	}

	y.swap(y_normal);
	x.swap(x_normal);


}

void update_theta(float &theta0, float &theta1,  std::vector<float> y,std::vector<float> x,float alfa) {
	float gradience_1 = 0.0;
	float gradience_0 = 0.0;
	float sum_err = 0;
	float temp_1 = 0.0;
	float temp_0 = 0.0;
	
	for (int i = 0; i < y.size(); i++) {
		temp_1=(theta0 +theta1*x[i]-y[i])*x[i];
		temp_0= (theta0+ theta1*x[i]-y[i]);
		gradience_1 +=temp_1;
		gradience_0 += temp_0;
	}
	get_sum_err(theta0, theta1, y, x, sum_err);
	//std::cout << "sum_err: " << sum_err << "\n";

	//std::cout << "gradience_1: " << gradience_1/y.size() << "\n";
	//std::cout << "gradience_0: " << gradience_0 /y.size()<< "\n";
	theta1 = theta1 - alfa*gradience_1/y.size();
	theta0 = theta0 - alfa*gradience_0/y.size();
	
}
void get_edgeline_slope_through_fit_curve(PointCloud::Ptr &cloud_in ,float &rotation_angle) {
	clock_t func_stt, func_end;
	func_stt = clock();
	float x_max = cloud_in->points[0].x;
	float x_min = cloud_in->points[0].x;
	
	for (int i = 0; i < cloud_in->points.size(); i++) {
		if (cloud_in->points[i].x > x_max)x_max = cloud_in->points[i].x;
		if (cloud_in->points[i].x < x_min) x_min = cloud_in->points[i].x;
	}

	std::array<float, 150>edge_point;
	std::array<float, 150>edge_point_tox;
	std::vector<float> interval_y;
	std::vector<float> interval_x;

	edge_point.fill(-10000);
	edge_point_tox.fill(0.0);
	float x_cell = (x_max - x_min) / 150;
	std::cout << "x_cell: " << x_max << "   " << x_min << "   " << x_cell << std::endl;
	int temp_edge;
	for (int j = 0; j < cloud_in->points.size(); j++) {
		temp_edge = floor((cloud_in->points[j].x - x_min) / x_cell);
		if (temp_edge == 150)temp_edge =149;
		if (cloud_in->points[j].y > edge_point[temp_edge]) {
			edge_point[temp_edge] = cloud_in->points[j].y;
			edge_point_tox[temp_edge] = cloud_in->points[j].x;
		}		
	}

	std::vector<float> reduced_edge;
	std::vector<float> reduced_edge_tox;
	std::vector<float> reduced_temp;
	float reduced_cmp=-10000.0;
	std::vector<int> reduced_indices;
	float mean_interval = 0.0;
	std::vector<float> interval;
	float temp_interval = 0.0;
	//找出边缘点
	for (int i = 0; i < 50; i++) {
		reduced_cmp = edge_point[3 * i];
		for (int j = 0; j < 3; j++) {
			reduced_temp.push_back(edge_point[i * 3 + j]);
			
			reduced_indices.push_back(i * 3 + j);
		}
		for (int k = 0; k < 2; k++) {
			for (int n = 0; n < (2 - k); n++) {
				if (reduced_temp[n] > reduced_temp[n + 1]) {
					std::swap(reduced_temp[n], reduced_temp[n + 1]);
					std::swap(reduced_indices[n], reduced_indices[n + 1]);
				}
			}
		}
		reduced_edge.push_back(reduced_temp[1]);
		reduced_edge_tox.push_back(edge_point_tox[reduced_indices[1]]);

		std::cout << "reduced_edge[i]: "<< " = " <<reduced_edge[i] << "    "<<"reduced_edge_tox[i]: " << " = " << reduced_edge_tox[i] << "\n";
		if (i > 0) {
			temp_interval = fabs(reduced_edge[i] - reduced_edge[i - 1]);
			interval.push_back(temp_interval);
		
			mean_interval += temp_interval / 49.0;
		}
		
		reduced_temp.clear();
		reduced_indices.clear();
	}
	//若本身已经方正，则直接判定rotate_angle=0
	bool already_correct = false;
	int out_interval = 0;
	float sum_interval_0=0.0;
	for (int i = 0; i < 49; i++) {
		std::cout << "interval: " << interval[i - 1] << "    mean_interval: "<<mean_interval<<std::endl;
		if (interval[i] - mean_interval > 20) {
			out_interval++;
			mean_interval = (mean_interval * 49 - interval[i]) / (49-out_interval);
		}
		
	}
	std::cout << "out_interval: " << out_interval << std::endl;
	for (int i = 0; i < 49; i++) {
		if (interval[i] < 1.0) {
			sum_interval_0 += fabs(interval[i] - mean_interval);
		}
	}
	std::cout << "sum_interval_0: " << sum_interval_0 << std::endl;

	//若摆正，直接赋值rotation_angle
	if (out_interval < 2&& sum_interval_0 < 48&&mean_interval<1.0) {
		rotation_angle=0.0;
		std::cout << "rotation angle: " << rotation_angle << std::endl;
	}
	
	else {//若未摆正,找出长边的点,并求其斜率
	
		std::vector<int> interval_indices;
		std::vector<float> longline;
		std::vector<float> longline_tox;
		float sum_interval = 0.0;

		std::vector<int> p_cls;
		std::vector<int> change_cls_indices;
		int temp_cls;
		int num_up = 0;
		int num_dn = 0;
		std::vector<float> test_cls;
		for (int m = 0; m < reduced_edge.size(); m++) {
			if (reduced_edge[m + 1] - reduced_edge[m] >= 0) {
				p_cls.push_back(1);
				num_up++;
			}
			else {
				p_cls.push_back(0);
				num_dn++;
			}
		}
		p_cls.push_back(p_cls.back());

		for (int j = 2; j < reduced_edge.size() - 2; j++) {
			temp_cls = round((p_cls[j - 2] + p_cls[j - 1] + p_cls[j] + p_cls[j + 1] + p_cls[j + 2]) / 5);
			if (temp_cls != p_cls[j]) {
				if (temp_cls = 1)num_up++;
				else num_dn++;
				change_cls_indices.push_back(j);
			}
		}
		for (int n = 0; n < change_cls_indices.size(); n++) {
			p_cls[change_cls_indices[n]] = 1 - p_cls[change_cls_indices[n]];
		}
		if (num_up >= num_dn) {
			for (int n = 2; n < reduced_edge.size()-2; n++) {
				if (p_cls[n] == 1) {
					longline.push_back(reduced_edge[n]);
					longline_tox.push_back(reduced_edge_tox[n]);
					std::cout << "n: " << n << "    " << "longline_y: " << reduced_edge[n] << "    " <<
						"longline_tox: " << reduced_edge_tox[n] << "\n\n";
				}
			}

		}
		if (num_up <num_dn) {
			for (int n = 2; n < reduced_edge.size()-2; n++) {
				if (p_cls[n] == 0) {
					longline.push_back(reduced_edge[n]);
					longline_tox.push_back(reduced_edge_tox[n]);
					std::cout << "n: " << n << "    " << "longline_y: " << reduced_edge[n] << "    " <<
						"longline_tox: " << reduced_edge_tox[n] << "\n\n";
				}
			}

		}

		float h_hat = 0.0;
		float theta0 = 0.0;
		float theta1 = 0.0;
		float sum_err = 0.0;
		float sum_err_temp = 0.0;
		std::cout << "longline[0]: " << longline[0] << "\n\n";

		float temp_theta1 = 0.0, temp_theta0 = 0.0;
		int k = 0;
		//特征缩放
		//get_normalize_data(longline, longline_tox);
		std::cout << "theta1: " << theta1 << "\n\n";
		std::cout << "theta0: " << theta0 << "\n\n";
		for (int i = 0; i < longline.size(); i++) {
			std::cout << "normalized longline_y: " << longline[i] << "    " << "normalized longline_x: " << longline_tox[i] << "\n\n\n";
		}
		update_theta(theta0, theta1, longline, longline_tox, 0.0001);
		std::cout << "new theta1: " << theta1 << "\n";
		std::cout << "new theta0: " << theta0 << "\n\n";
		get_sum_err(theta0, theta1, longline, longline_tox, sum_err);
		cout << "sum_err: " << sum_err << "\n";
		//float temp_sum = FLT_MAX;
		//cout << "temp_sum: " << temp_sum << "\n";

		//while ((fabs(theta0-temp_theta0) > 0.01) ||(fabs(theta1-temp_theta1 )>0.1)) {

		//梯度下降法，迭代循环
		while (sum_err>1.0 ) {//通过前期选点处理，保证点的直线性，若存在较大误差点，此迭代条件需要改变
			k++;
			//temp_sum = sum_err;
			get_sum_err(theta0, theta1, longline, longline_tox, sum_err);
			//cout << "sum_err: " << sum_err << "\n";
			//cout << "temp_sum: " << temp_sum << "\n";
			//std::cout << "sum_err: " << sum_err << std::endl;
			temp_theta0 = theta0;
			temp_theta1 = theta1;
			update_theta(theta0, theta1, longline, longline_tox, 0.0001);
			//std::cout << "new theta1: " << theta1 << "\n";
			//std::cout << "new theta0: " << theta0 << "\n";
			//std::cout << "k次迭代： " << k << std::endl << "\n";
			if (fabs(temp_theta1 - theta1)<0.00001 && fabs(temp_theta0 - theta0)< 0.00001) {
				break;
			}
		}
		std::cout << "k次迭代： " << k << std::endl << "\n";


		std::cout << "theta0: " << theta0 << endl << "theta1: " << theta1 << endl;

		rotation_angle = -1 * atan(theta1);

		std::cout << "rotation angle: " << rotation_angle << std::endl;


	}
	func_end = clock();
	cout << "get_edgeline_slope_through_fit_curve: " << func_end - func_stt << endl;

}

/*...............................................
当特征数量小于10000时，正规方程法速度更快，经过比较，
梯度下降法在384ms,正规方程法54ms
正规方程法缺点：
	1、在特征数量多于点数时，无法计算，即x*x.transpose为奇异值矩阵时，无法获得解
................................................*/
//正规方程
void get_edgeline_slope_through_regular_func_matrix(PointCloud::Ptr &cloud_in, float &rotation_angle) {
	clock_t func_stt, func_end;
	func_stt = clock();
	float x_max = cloud_in->points[0].x;
	float x_min = cloud_in->points[0].x;

	for (int i = 0; i < cloud_in->points.size(); i++) {
		if (cloud_in->points[i].x > x_max)x_max = cloud_in->points[i].x;
		if (cloud_in->points[i].x < x_min) x_min = cloud_in->points[i].x;
	}

	std::array<float, 150>edge_point;
	std::array<float, 150>edge_point_tox;
	std::vector<float> interval_y;
	std::vector<float> interval_x;

	edge_point.fill(-10000);
	edge_point_tox.fill(0.0);
	float x_cell = (x_max - x_min) / 150;
	std::cout << "x_cell: " << x_max << "   " << x_min << "   " << x_cell << std::endl;
	int temp_edge;
	for (int j = 0; j < cloud_in->points.size(); j++) {
		temp_edge = floor((cloud_in->points[j].x - x_min) / x_cell);
		if (temp_edge == 150)temp_edge = 149;
		if (cloud_in->points[j].y > edge_point[temp_edge]) {
			edge_point[temp_edge] = cloud_in->points[j].y;
			edge_point_tox[temp_edge] = cloud_in->points[j].x;
		}
	}

	std::vector<float> reduced_edge;
	std::vector<float> reduced_edge_tox;
	std::vector<float> reduced_temp;
	float reduced_cmp = -10000.0;
	std::vector<int> reduced_indices;
	float mean_interval = 0.0;
	std::vector<float> interval;
	float temp_interval = 0.0;
	//找出边缘点
	for (int i = 0; i < 50; i++) {
		reduced_cmp = edge_point[3 * i];
		for (int j = 0; j < 3; j++) {
			reduced_temp.push_back(edge_point[i * 3 + j]);

			reduced_indices.push_back(i * 3 + j);
		}
		for (int k = 0; k < 2; k++) {
			for (int n = 0; n < (2 - k); n++) {
				if (reduced_temp[n] > reduced_temp[n + 1]) {
					std::swap(reduced_temp[n], reduced_temp[n + 1]);
					std::swap(reduced_indices[n], reduced_indices[n + 1]);
				}
			}
		}
		reduced_edge.push_back(reduced_temp[1]);
		reduced_edge_tox.push_back(edge_point_tox[reduced_indices[1]]);

		std::cout << "reduced_edge[i]: " << " = " << reduced_edge[i] << "    " << "reduced_edge_tox[i]: " << " = " << reduced_edge_tox[i] << "\n";
		if (i > 0) {
			temp_interval = fabs(reduced_edge[i] - reduced_edge[i - 1]);
			interval.push_back(temp_interval);

			mean_interval += temp_interval / 49.0;
		}

		reduced_temp.clear();
		reduced_indices.clear();
	}
	//若本身已经方正，则直接判定rotate_angle=0
	bool already_correct = false;
	int out_interval = 0;
	float sum_interval_0 = 0.0;
	for (int i = 0; i < 49; i++) {
		std::cout << "interval: " << interval[i - 1] << "    mean_interval: " << mean_interval << std::endl;
		if (interval[i] - mean_interval > 20) {
			out_interval++;
			mean_interval = (mean_interval * 49 - interval[i]) / (49 - out_interval);
		}

	}
	std::cout << "out_interval: " << out_interval << std::endl;
	for (int i = 0; i < 49; i++) {
		if (interval[i] < 1.0) {
			sum_interval_0 += fabs(interval[i] - mean_interval);
		}
	}
	std::cout << "sum_interval_0: " << sum_interval_0 << std::endl;

	//若摆正，直接赋值rotation_angle
	if (out_interval < 2 && sum_interval_0 < 48 && mean_interval<1.0) {
		rotation_angle = 0.0;
		std::cout << "rotation angle: " << rotation_angle << std::endl;
	}

	else {//若未摆正,找出长边的点,并求其斜率
		std::vector<int> interval_indices;
		std::vector<float> longline;
		std::vector<float> longline_tox;
		float sum_interval = 0.0;

		std::vector<int> p_cls;
		std::vector<int> change_cls_indices;
		int temp_cls;
		int num_up = 0;
		int num_dn = 0;
		std::vector<float> test_cls;
		for (int m = 0; m < reduced_edge.size(); m++) {
			if (reduced_edge[m + 1] - reduced_edge[m] >= 0) {
				p_cls.push_back(1);
				num_up++;
			}
			else {
				p_cls.push_back(0);
				num_dn++;
			}
		}
		p_cls.push_back(p_cls.back());

		for (int j = 2; j < reduced_edge.size() - 2; j++) {
			temp_cls = round((p_cls[j - 2] + p_cls[j - 1] + p_cls[j] + p_cls[j + 1] + p_cls[j + 2]) / 5);
			if (temp_cls != p_cls[j]) {
				if (temp_cls = 1)num_up++;
				else num_dn++;
				change_cls_indices.push_back(j);
			}
		}
		for (int n = 0; n < change_cls_indices.size(); n++) {
			p_cls[change_cls_indices[n]] = 1 - p_cls[change_cls_indices[n]];
		}
		if (num_up >= num_dn) {
			for (int n = 2; n < reduced_edge.size() - 2; n++) {
				if (p_cls[n] == 1) {
					longline.push_back(reduced_edge[n]);
					longline_tox.push_back(reduced_edge_tox[n]);
					std::cout << "n: " << n << "    " << "longline_y: " << reduced_edge[n] << "    " <<
						"longline_tox: " << reduced_edge_tox[n] << "\n\n";
				}
			}

		}
		if (num_up < num_dn) {
			for (int n = 2; n < reduced_edge.size() - 2; n++) {
				if (p_cls[n] == 0) {
					longline.push_back(reduced_edge[n]);
					longline_tox.push_back(reduced_edge_tox[n]);
					std::cout << "n: " << n << "    " << "longline_y: " << reduced_edge[n] << "    " <<
						"longline_tox: " << reduced_edge_tox[n] << "\n\n";
				}
			}

		}


		int longline_size = longline.size();
		Eigen::MatrixXf x_mat(longline_size, 2);
		Eigen::MatrixXf y_mat(longline_size, 1);
		Eigen::MatrixXf res_temp0(longline_size, longline_size);
		Eigen::MatrixXf res_temp1(longline_size, longline_size);
		Eigen::MatrixXf res_temp2(2, longline_size);
		Eigen::VectorXf theta_mat(2);
		//Eigen::VectorXf temp_I = Eigen::VectorXf::Ones();
		
		//x_mat.col(0) = temp_I;
		//y_mat.col(0) = temp_I;
		for (int i = 0; i < longline_size; i++) {
			x_mat(i,1) = longline_tox[i];
			x_mat(i, 0) = 1.0;
			y_mat(i,0) = longline[i];
		}
		res_temp0 = x_mat.transpose()*x_mat;
		res_temp1 = res_temp0.inverse();
		res_temp2 = res_temp1*(x_mat.transpose());
		cout << res_temp2.size() << endl << "res_temp2.size: " << res_temp2.rows() <<"  "<< res_temp2.cols() << endl;
		cout << y_mat.size() << endl << "y_mat.size: " << y_mat.rows() <<"    "<< y_mat.cols() << endl;;
		theta_mat = res_temp2*y_mat;

		cout << "theta_mat0: " << theta_mat[0] << "theta_mat1: " << theta_mat[1] << endl;
		
		rotation_angle = -1 * atan(theta_mat[1]);
		
	}
	func_end = clock();
	cout << "time of regular func: " << func_end - func_stt << endl;
}




