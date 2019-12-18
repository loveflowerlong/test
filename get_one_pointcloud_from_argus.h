#pragma once
#include <ArgusSDK.h>
#include <iostream>
#include <string>
#include<ctime>
int get_one_pointcloud_from_argus() {
	std::cout << "相机开始连接"<<std::endl;
	ARGUS_INFO *ArgusInfo;
	//开辟存放相机信息的数组空间，这默认为10
	ArgusInfo = (ARGUS_INFO *)malloc(10 * sizeof(ARGUS_INFO));
	if (ArgusInfo == NULL)
		return -1;
	//这里的 P_Width * P_High表示点云点的总数，3表示一个点的坐标信息为(x,y,z),Recv_Type(float)为(x,y,z)均为float类型的数据
	Recv_Type *cloud;
	cloud = (Recv_Type*)malloc(sizeof(Recv_Type)* P_Width * P_High * 3);
	if (cloud == NULL)
		return -1;
	//定义局域网内的相机有多少个
	int camera_count = 0;
	//定义保存点云数据的文件名称
	char pcd_name[20] = "pcd0.pcd";
	//定义保存深度图的文件名称
	char png_name[20] = "png0.png";
	//以广播的形式来搜索局域网内有多少个相机
	camera_count = Argus_Discover(&ArgusInfo);
	if (camera_count <= 0) {
		cout << "当前局域网内没有相机" << endl;
		return -3;
	}
	cout << "该局域网内的相机个数：camera_count = " << camera_count << endl;
	for (int i = 0; i < camera_count; i++)
	{
		if (strcmp(Argus_IP(&ArgusInfo[i]), "192.168.1.12") == 0)
		{
			cout << "所要连接的相机的IP = " << Argus_IP(&ArgusInfo[i]) << endl;
			cout << "所要连接的相机的序列号为： " << Argus_SerialNumber(&ArgusInfo[i]) << endl;
			//连接相机，1表示在接收数据时，采用的是多帧序列循环采集的方式，这里预先开辟1个存放点云数据的内存空间
			int connect_flag = Argus_Connect(&ArgusInfo[i], 1);
			if (connect_flag < 0) {
				cout << "连接失败" << endl;
				return -1;
			}
			//打开数据流，打开点云的数据流通道，0表示开启的是点云数据流通道
			Argus_StreamOn(&ArgusInfo[i], 0);
			Argus_StreamOn(&ArgusInfo[i], 2);

			//char pcd_name[20] = "test0.pcd";
			//char png_name[20] = "test0.png";
			//int camera_postion; //输入位置 
			//int MultiExposure = 1;
			//int buf_size = 0;
			//cout << "请输入要连接第几个相机(对应的是ArgusInfo数组中的下标，即从0开始计数)：";
			//cin >> camera_postion;
			//cout << endl;
			//cout << "请输入接收数据的缓冲区的大小（默认为1，该值和单次触发数量保持一致）：";
			//cin >> buf_size;
			//cout << endl;
			//  		int trgger_count = 0;
			//cout << "请输入要触发的张数（需要拍几张照片,建议这个值等于缓冲区大小）：";
			//cin >> trgger_count;
			//cout << endl;
			//cout << "请选择要打开的数据流通道：0->PointCloud_Stream（点云）,1->GrayImage_Stream（红外）: ";
			//int stream_flag = 0;
			//cin >> stream_flag;
			//cout << endl;

			////连接相机
			//if (Argus_Connect(&ArgusInfo[camera_postion], buf_size) == 0)
			//	cout << "connect succeed" << endl;
			//else {
			//	cout << "connect failed" << endl;
			//	return -1;
			//}
			//for (int j = 0; j < trgger_count; j++) {
			//	cout << "第 " << j << " 次触发开始" << endl;
			//	pcd_name[7] = '0' + j;      //对应触发的次数
			//	//png_name[4] = '0' + j;	
			//	if (stream_flag == 0) {
			//		Argus_CapturePointCloud(&ArgusInfo[camera_postion], cloud);
			//		//Argus_FilterPointCloud的第二个参数是表示过滤的是离群点，越大滤掉的越多（建议值在10~50之前），第三个参数是过滤的是悬浮在表面的点，该值越小滤掉的点越多（建议值在1.00~1.80）
			//		Argus_FilterPointCloud(cloud, 10, 1.80);
			//		Argus_SavePointCloudToPcd(cloud, pcd_name);
			//	}		
			//	cout << "第 " << j << " 次触发结束" << endl;
			//}

			//采集点云
			string pcdname = ".pcd";
			char name[20];
			clock_t capture_start, capture_end;
			capture_start = clock();
			capture_end = capture_start + 1000 * 10;
			for (int m = 0; capture_start + 1000 * m < capture_end; m++) {
				Argus_CapturePointCloud(&ArgusInfo[i], cloud);
				Argus_FilterPointCloud(cloud, 10, 1.80);
				pcdname = to_string(m) + pcdname;
				strcpy_s(name, pcdname.c_str());

				Argus_SavePointCloudToPcd(cloud, name);
				pcdname = ".pcd";

			}

			if (Argus_StreamOff(&ArgusInfo[i], 0) == 0)
				cout << "stream off" << endl;
			if (Argus_StreamOff(&ArgusInfo[i], 2) == 0)
				cout << "stream off" << endl;
			if (Argus_DisConnect(&ArgusInfo[i]) == 0)
				cout << "camera disconnect" << endl;
		}
	}
	free(ArgusInfo);
	ArgusInfo = NULL;
	free(cloud);
	cloud = NULL;
	//system("pause");
	return 0;

}

