#pragma once
#include <ArgusSDK.h>
#include <iostream>
#include <string>
#include<ctime>
int get_one_pointcloud_from_argus() {
	std::cout << "�����ʼ����"<<std::endl;
	ARGUS_INFO *ArgusInfo;
	//���ٴ�������Ϣ������ռ䣬��Ĭ��Ϊ10
	ArgusInfo = (ARGUS_INFO *)malloc(10 * sizeof(ARGUS_INFO));
	if (ArgusInfo == NULL)
		return -1;
	//����� P_Width * P_High��ʾ���Ƶ��������3��ʾһ�����������ϢΪ(x,y,z),Recv_Type(float)Ϊ(x,y,z)��Ϊfloat���͵�����
	Recv_Type *cloud;
	cloud = (Recv_Type*)malloc(sizeof(Recv_Type)* P_Width * P_High * 3);
	if (cloud == NULL)
		return -1;
	//����������ڵ�����ж��ٸ�
	int camera_count = 0;
	//���屣��������ݵ��ļ�����
	char pcd_name[20] = "pcd0.pcd";
	//���屣�����ͼ���ļ�����
	char png_name[20] = "png0.png";
	//�Թ㲥����ʽ���������������ж��ٸ����
	camera_count = Argus_Discover(&ArgusInfo);
	if (camera_count <= 0) {
		cout << "��ǰ��������û�����" << endl;
		return -3;
	}
	cout << "�þ������ڵ����������camera_count = " << camera_count << endl;
	for (int i = 0; i < camera_count; i++)
	{
		if (strcmp(Argus_IP(&ArgusInfo[i]), "192.168.1.12") == 0)
		{
			cout << "��Ҫ���ӵ������IP = " << Argus_IP(&ArgusInfo[i]) << endl;
			cout << "��Ҫ���ӵ���������к�Ϊ�� " << Argus_SerialNumber(&ArgusInfo[i]) << endl;
			//���������1��ʾ�ڽ�������ʱ�����õ��Ƕ�֡����ѭ���ɼ��ķ�ʽ������Ԥ�ȿ���1����ŵ������ݵ��ڴ�ռ�
			int connect_flag = Argus_Connect(&ArgusInfo[i], 1);
			if (connect_flag < 0) {
				cout << "����ʧ��" << endl;
				return -1;
			}
			//�����������򿪵��Ƶ�������ͨ����0��ʾ�������ǵ���������ͨ��
			Argus_StreamOn(&ArgusInfo[i], 0);
			Argus_StreamOn(&ArgusInfo[i], 2);

			//char pcd_name[20] = "test0.pcd";
			//char png_name[20] = "test0.png";
			//int camera_postion; //����λ�� 
			//int MultiExposure = 1;
			//int buf_size = 0;
			//cout << "������Ҫ���ӵڼ������(��Ӧ����ArgusInfo�����е��±꣬����0��ʼ����)��";
			//cin >> camera_postion;
			//cout << endl;
			//cout << "������������ݵĻ������Ĵ�С��Ĭ��Ϊ1����ֵ�͵��δ�����������һ�£���";
			//cin >> buf_size;
			//cout << endl;
			//  		int trgger_count = 0;
			//cout << "������Ҫ��������������Ҫ�ļ�����Ƭ,�������ֵ���ڻ�������С����";
			//cin >> trgger_count;
			//cout << endl;
			//cout << "��ѡ��Ҫ�򿪵�������ͨ����0->PointCloud_Stream�����ƣ�,1->GrayImage_Stream�����⣩: ";
			//int stream_flag = 0;
			//cin >> stream_flag;
			//cout << endl;

			////�������
			//if (Argus_Connect(&ArgusInfo[camera_postion], buf_size) == 0)
			//	cout << "connect succeed" << endl;
			//else {
			//	cout << "connect failed" << endl;
			//	return -1;
			//}
			//for (int j = 0; j < trgger_count; j++) {
			//	cout << "�� " << j << " �δ�����ʼ" << endl;
			//	pcd_name[7] = '0' + j;      //��Ӧ�����Ĵ���
			//	//png_name[4] = '0' + j;	
			//	if (stream_flag == 0) {
			//		Argus_CapturePointCloud(&ArgusInfo[camera_postion], cloud);
			//		//Argus_FilterPointCloud�ĵڶ��������Ǳ�ʾ���˵�����Ⱥ�㣬Խ���˵���Խ�ࣨ����ֵ��10~50֮ǰ���������������ǹ��˵��������ڱ���ĵ㣬��ֵԽС�˵��ĵ�Խ�ࣨ����ֵ��1.00~1.80��
			//		Argus_FilterPointCloud(cloud, 10, 1.80);
			//		Argus_SavePointCloudToPcd(cloud, pcd_name);
			//	}		
			//	cout << "�� " << j << " �δ�������" << endl;
			//}

			//�ɼ�����
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

