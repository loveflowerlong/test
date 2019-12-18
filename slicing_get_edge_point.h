#pragma once
#include"name_def.h"
#include<limits.h>
void slicing_get_edge_point(PointCloud::Ptr cloud_in,float &x_max_x,float &x_max_y, float &x_min_x, float &x_min_y, float &y_max_y,float &y_max_x, float &y_min_y,float &y_min_x) {//cloud_in要求为一个平面正规（边与坐标基本垂直或平行）近似矩形
	int size_p = cloud_in->points.size();
	float min_x = _FMAX;
	float max_x =-1*_FMAX;
	float min_y = _FMAX;
	float max_y = -1 * _FMAX;
	float x_left_p =  0.0 ;
	float x_right_p = 0.0 ;
	float x_left[50] = { 0.0 };
	float x_right[50] = { 0.0 };
	float x_left_temp[250] = { _FMAX };
	float x_right_temp[250] = { -1 * _FMAX };
	for (int i = 0; i < size_p; i++) {
		if (cloud_in->points[i].x > max_x)
			max_x = cloud_in->points[i].x;
		if (cloud_in->points[i].x < min_x)
			min_x = cloud_in->points[i].x;
		if (cloud_in->points[i].y > max_y)
			max_y = cloud_in->points[i].y;
		if (cloud_in->points[i].y < min_y)
			min_y = cloud_in->points[i].y;
	}
	float per_y = (max_y - min_y)/50;
	float per_x = (max_x - min_x)/50;
	float min_per_y = (max_y - min_y) / 250;
	float min_per_x = (max_x - min_x) / 250;
	for (int j = 0; j < size_p; j++) {
		for (int m = 0; m < 50; m++) {
			if (cloud_in->points[j].y >= (min_y + per_y*m) && cloud_in->points[j].y <(min_y + per_y*(m + 1))){//定位点在那个大的区间
	
				for (int n = 0; n < 5; n++) {//定位点在大区间的哪个小区间中
					if (cloud_in->points[j].y >= (min_y + per_y*m + min_per_y*n) && cloud_in->points[j].y < (min_y + per_y*m + min_per_y*(n + 1))) {
						if (cloud_in->points[j].x < x_left_temp[5*m+n])
							x_left_temp[5*m+n] = cloud_in->points[j].x;
						if (cloud_in->points[j].x > x_right_temp[5*m+n])
							x_right_temp[5*m+n] = cloud_in->points[j].x;					
					}
					//x_left_p += x_left_temp;
					//x_right_p += x_right_temp;
				}

			}
			
		}
	}

	float x_left_extreme = _FMAX;
	float x_right_extreme = -1 * _FMAX;
	float num_x_left = 0.0;
	float num_x_right = 0.0;
	float err_left_temp[5] = { 0.0 };
	float err_right_temp[5] = { 0.0 };
	int y_min_per_indices[50] = { 0 };
	int y_min_per_r_indices[50] = { 0 };
	float x_left_extreme_correspond_y;
	float x_right_extreme_correspond_y;

	for (int mm = 0; mm < 50; mm++) {
		for (int nn = 0; nn < 5; nn++) {
			x_left_p += x_left_temp[5 * mm + nn];
			x_right_p += x_right_temp[5 * mm + nn];

		}
		


		x_left[mm] = x_left_p / 5;
		x_right[mm] = x_right_p / 5;

		for (int nnn = 0; nnn < 5; nnn++) {
			err_left_temp[nnn]= fabs(x_left[5*mm] - x_left_temp[5*mm + nnn]);
		}
		float err_left_temp_min = err_left_temp[0];
		for (int nnnn = 0; nnnn < 5; nnnn++) {
			if (err_left_temp_min > err_left_temp[nnnn]) {
				err_left_temp_min = err_left_temp[nnnn];
				y_min_per_indices[mm] = nnnn;
			}
		}


		for (int nnnr = 0; nnnr < 5; nnnr++) {
			err_right_temp[nnnr] = fabs(x_right[5 * mm] - x_right_temp[5 * mm + nnnr]);
		}
		float err_right_temp_min = err_right_temp[0];
		for (int nnnnr = 0; nnnnr < 5; nnnnr++) {
			if (err_right_temp_min > err_right_temp[nnnnr]) {
				err_right_temp_min = err_right_temp[nnnnr];
				y_min_per_r_indices[mm] = nnnnr;
			}
		}



		num_x_left += x_left[mm];
		num_x_right += x_right[mm];
		if (x_left[mm] < x_left_extreme) { 
			x_left_extreme = x_left[mm];
			x_left_extreme_correspond_y = per_y*mm + min_per_y*y_min_per_indices[mm];
		}


		if (x_right[mm] > x_right_extreme) {
			x_right_extreme = x_right[mm];
			x_right_extreme_correspond_y = per_y*mm + min_per_y*y_min_per_r_indices[mm];
		}
	}

	//在点云正规时，以上程序失效，所以要避免
	float x_left_average = num_x_left / 50;
	float x_right_average = num_x_right / 50;
	float err_left=0.0;
	float err_right=0.0;
	for (int p = 0; p < 50; p++) {
		err_left = err_left + fabs((x_left[p] - x_left_average));
		err_right = err_right + fabs((x_right[p] - x_right_average));
	}
	if (err_left < 20.0 || err_right < 20) {
		std::cerr << "点云正规    " << "err: " << err_left << "&" << err_right << std::endl;
	}










	float y_left_p = 0.0;
	float y_right_p = 0.0;
	float y_left[50] = { 0.0 };
	float y_right[50] = { 0.0 };
	float y_left_temp[250] = { _FMAX };
	float y_right_temp[250] = { -1 * _FMAX };

	for (int j = 0; j < size_p; j++) {
		for (int m = 0; m < 50; m++) {
			if (cloud_in->points[j].y >= (min_x + per_x*m) && cloud_in->points[j].x <(min_x + per_x*(m + 1))) {//定位点在那个大的区间

				for (int n = 0; n < 5; n++) {//定位点在大区间的哪个小区间中
					if (cloud_in->points[j].x >= (min_x + per_x*m + min_per_x*n) && cloud_in->points[j].x < (min_x + per_x*m + min_per_x*(n + 1))) {
						if (cloud_in->points[j].y < y_left_temp[5 * m + n])
							y_left_temp[5 * m + n] = cloud_in->points[j].y;
						if (cloud_in->points[j].y > y_right_temp[5 * m + n])
							y_right_temp[5 * m + n] = cloud_in->points[j].y;
					}
					//x_left_p += x_left_temp;
					//x_right_p += x_right_temp;
				}

			}

		}
	}

	float y_left_extreme = _FMAX;
	float y_right_extreme = -1 * _FMAX;
	float num_y_left = 0.0;
	float num_y_right = 0.0;
	float y_err_left_temp[5] = { 0.0 };
	float y_err_right_temp[5] = { 0.0 };
	
	int x_min_per_indices[50] = { 0 };
	int x_min_per_r_indices[50] = { 0 };
	float y_left_extreme_correspond_x;
	float y_right_extreme_correspond_x;

	for (int mm = 0; mm < 50; mm++) {
		for (int nn = 0; nn < 5; nn++) {
			y_left_p += y_left_temp[5 * mm + nn];
			y_right_p += y_right_temp[5 * mm + nn];

		}



		y_left[mm] = y_left_p / 5;
		y_right[mm] = y_right_p / 5;

		for (int nnn = 0; nnn < 5; nnn++) {
			y_err_left_temp[nnn] = fabs(y_left[5 * mm] - y_left_temp[5 * mm + nnn]);
		}
		float y_err_left_temp_min = y_err_left_temp[0];
		for (int nnnn = 0; nnnn < 5; nnnn++) {
			if (y_err_left_temp_min > y_err_left_temp[nnnn]) {
				y_err_left_temp_min = y_err_left_temp[nnnn];
				x_min_per_indices[mm] = nnnn;
			}
		}


		for (int nnnr = 0; nnnr < 5; nnnr++) {
			y_err_right_temp[nnnr] = fabs(y_right[5 * mm] - y_right_temp[5 * mm + nnnr]);
		}
		float y_err_right_temp_min = err_right_temp[0];
		for (int nnnnr = 0; nnnnr < 5; nnnnr++) {
			if (y_err_right_temp_min > y_err_right_temp[nnnnr]) {
				y_err_right_temp_min = y_err_right_temp[nnnnr];
				x_min_per_r_indices[mm] = nnnnr;
			}
		}



		num_y_left += y_left[mm];
		num_y_right += y_right[mm];
		if (y_left[mm] < y_left_extreme) {
			y_left_extreme = y_left[mm];
			y_left_extreme_correspond_x = per_x*mm + min_per_x*x_min_per_indices[mm];
		}


		if (y_right[mm] > y_right_extreme) {
			y_right_extreme = y_right[mm];
			y_right_extreme_correspond_x = per_x*mm + min_per_x*x_min_per_r_indices[mm];
		}
	}

	//在点云正规时，以上程序失效，所以要避免
	float y_left_average = num_y_left / 50;
	float y_right_average = num_y_right / 50;
	float y_err_left = 0.0;
	float y_err_right = 0.0;
	for (int p = 0; p < 50; p++) {
		y_err_left = y_err_left + fabs((y_left[p] - y_left_average));
		y_err_right = y_err_right + fabs((y_right[p] - y_right_average));
	}
	if (y_err_left < 20.0 || y_err_right < 20) {
		std::cerr << "点云正规    " << "err: " << y_err_left << "&" << y_err_right << std::endl;
	}
	


	 







	x_max_x = x_right_extreme;
	x_max_y= x_right_extreme_correspond_y;
	y_max_x= y_right_extreme_correspond_x;
	y_max_y=y_right_extreme;
	x_min_x = x_left_extreme;
	x_min_y=x_left_extreme_correspond_y;
	y_min_x = y_left_extreme_correspond_x;
	y_min_y= y_left_extreme;










}
