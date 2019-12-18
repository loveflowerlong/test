#pragma once
#include"name_def.h"
#include<pcl/common/centroid.h>
#include<pcl/common/eigen.h>
#include<pcl/common/transforms.h>
#include<iostream>

void centroid_to_origin(PointCloud::Ptr cloud_in, PointCloud::Ptr cloud_out,
	Eigen::Affine3f transform_translate,Eigen::Affine3f transform_rotate,Eigen::Vector4f centroid ,bool only_totate_to_z=true)
{
	Eigen::Vector4f pcaCentroid;
	pcl::compute3DCentroid(*cloud_in, pcaCentroid);
	std::cout << "质心" << pcaCentroid << std::endl;
	Eigen::Matrix3f covariance;
	pcl::computeCovarianceMatrixNormalized(*cloud_in, pcaCentroid, covariance);
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
	Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
	Eigen::Vector3f eigenValuesPCA = eigen_solver.eigenvalues();
	eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));
	eigenVectorsPCA.col(1) = eigenVectorsPCA.col(2).cross(eigenVectorsPCA.col(0));


	Eigen::Affine3f transform = Eigen::Affine3f::Identity();
	Eigen::Affine3f transform1 = Eigen::Affine3f::Identity();
	Eigen::Affine3f transform2 = Eigen::Affine3f::Identity();
	Eigen::Vector3f principal_z = eigenVectorsPCA.col(0);
	//计算主方向到坐标系z轴的变换矩阵

	float alpha = atan(principal_z[1] / principal_z[2]);
	float phi = -1 * atan((principal_z[0] * cos(alpha)) / principal_z[2]);


	transform1.translation() << -1 * pcaCentroid[0], -1 * pcaCentroid[1], -1 * pcaCentroid[2];
	transform2.rotate(Eigen::AngleAxisf(alpha, Eigen::Vector3f::UnitX()));
	transform2.rotate(Eigen::AngleAxisf(phi, Eigen::Vector3f::UnitY()));
	transform = transform2*transform1;

	pcl::transformPointCloud(*cloud_in, *cloud_out, transform);//transform 移动第一主方向与坐标系z轴重合
    //pcl::transformPointCloud(*cloud_out, *cloud_out, transform1);
	//pcl::transformPointCloud(*cloud_out, *cloud_out, transform2);
	std::cout << "transform: " <<std::endl<< transform2.matrix() << std::endl << transform1.matrix() << std::endl << transform.matrix() << std::endl;
	transform_translate = transform1;
	transform_rotate = transform2;
	centroid = pcaCentroid;

}




inline void centroid_to_origin(PointCloud::Ptr &cloud_in, PointCloud::Ptr &cloud_out)
{
	/*if (only_rotate_to_z != false) {
		cerr << "are you sure only_rotate_to_z ?" << endl;
		return;
	}*/
	Eigen::Vector4f pcaCentroid;
	pcl::compute3DCentroid(*cloud_in, pcaCentroid);
	std::cout << "质心" << pcaCentroid << std::endl;
	Eigen::Matrix3f covariance;
	pcl::computeCovarianceMatrixNormalized(*cloud_in, pcaCentroid,covariance);
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
	Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
	Eigen::Vector3f eigenValuesPCA = eigen_solver.eigenvalues();
	eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));
	eigenVectorsPCA.col(1) = eigenVectorsPCA.col(2).cross(eigenVectorsPCA.col(0));

	Eigen::Vector3f principal_x = eigenVectorsPCA.col(1);//设为点云的z轴

	Eigen::Affine3f transform = Eigen::Affine3f::Identity();
	Eigen::Affine3f transform1 = Eigen::Affine3f::Identity();
	Eigen::Affine3f transform2 = Eigen::Affine3f::Identity();
	Eigen::Vector3f principal_z = eigenVectorsPCA.col(0);
	//计算主方向到坐标系z轴的变换矩阵

	
	float alpha = atan(principal_z[1] / principal_z[2]);
	float phi = -1*atan((principal_z[0] * cos(alpha)) / principal_z[2]);
	

	transform1.translation() << -1 * pcaCentroid[0], -1 * pcaCentroid[1], -1 * pcaCentroid[2];
	transform2.rotate(Eigen::AngleAxisf(alpha, Eigen::Vector3f::UnitX()));
	transform2.rotate(Eigen::AngleAxisf(phi, Eigen::Vector3f::UnitY()));
	transform = transform2*transform1;


	std::cout << "alpha: " << alpha << std::endl << "phi: " << phi << std::endl;
	principal_x = transform1*eigenVectorsPCA.col(1);
	principal_x = transform2*principal_x;

	Eigen::Affine3f transformr_x = Eigen::Affine3f::Identity();
	float r_x = -1*atan(principal_x[1] / principal_x[0]);
	std::cout << "？？？  z=0?" << principal_x[2] << std::endl << "r_x"<<r_x<<std::endl;
	
	transformr_x.rotate(Eigen::AngleAxisf(r_x, Eigen::Vector3f::UnitZ()));
	//std::cout << ":transformr_x:  " << std::endl <<transformr_x.matrix() << std::endl;

	pcl::transformPointCloud(*cloud_in,*cloud_out, transform);//transform 移动第一主方向与坐标系z轴重合
	pcl::transformPointCloud(*cloud_out, *cloud_out, transformr_x);//在上一步的基础上transformr_x 移动第二主方向与X轴重合
	

	//pcl::transformPointCloud(*cloud_out, *cloud_out, transform2);
	std::cout << "transform: " << transform2.matrix()<<std::endl<<transform1.matrix()<<std::endl<<transform.matrix() << std::endl;
	//pcl::transformPointCloud(*cloud_out, *cloud_out, transform1);

}
inline void centroid_to_origin(PointCloud::Ptr cloud_in, PointCloud::Ptr cloud_out,Eigen::Affine3f &transform,Eigen::Affine3f &transformr_x)
{
	Eigen::Vector4f pcaCentroid;
	pcl::compute3DCentroid(*cloud_in, pcaCentroid);
	std::cout << "质心" << pcaCentroid << std::endl;
	Eigen::Matrix3f covariance;
	pcl::computeCovarianceMatrixNormalized(*cloud_in, pcaCentroid, covariance);
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
	Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
	Eigen::Vector3f eigenValuesPCA = eigen_solver.eigenvalues();
	eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));
	eigenVectorsPCA.col(1) = eigenVectorsPCA.col(2).cross(eigenVectorsPCA.col(0));

	Eigen::Vector3f principal_x = eigenVectorsPCA.col(1);//设为点云的z轴

	Eigen::Affine3f transform1 = Eigen::Affine3f::Identity();
	Eigen::Affine3f transform2 = Eigen::Affine3f::Identity();
	Eigen::Vector3f principal_z = eigenVectorsPCA.col(0);
	//计算主方向到坐标系z轴的变换矩阵
	float alpha = atan(principal_z[1] / principal_z[2]);
	float phi = -1 * atan((principal_z[0] * cos(alpha)) / principal_z[2]);
	transform1.translation() << -1*pcaCentroid[0], -1*pcaCentroid[1], -1*pcaCentroid[2];

	transform2.rotate(Eigen::AngleAxisf(alpha, Eigen::Vector3f::UnitX()));
	transform2.rotate(Eigen::AngleAxisf(phi, Eigen::Vector3f::UnitY()));
	transform = transform2*transform1;

	principal_x = transform1*eigenVectorsPCA.col(1);
	principal_x = transform2*principal_x;


	//Eigen::Affine3f transformr_x = Eigen::Affine3f::Identity();
	float r_x = -1 * atan(principal_x[1] / principal_x[0]);
	std::cout << "？？？  z=0?" << principal_x[2] << std::endl << "r_x" << r_x << std::endl;

	transformr_x.rotate(Eigen::AngleAxisf(r_x, Eigen::Vector3f::UnitZ()));


	pcl::transformPointCloud(*cloud_in, *cloud_out, transform);//transform 移动第一主方向与坐标系z轴重合
	pcl::transformPointCloud(*cloud_out, *cloud_out, transformr_x);//在上一步的基础上transformr_x 移动第二主方向与X轴重合


}

inline void centroid_to_origin(PointCloud::Ptr cloud_in, PointCloud::Ptr cloud_out,Eigen::Affine3f &transform,
	Eigen::Affine3f &transformr_x,float &z_rotate_by_x, float &z_rotate_by_y,float &x_rotate_by_z )
{
	Eigen::Vector4f pcaCentroid;
	pcl::compute3DCentroid(*cloud_in, pcaCentroid);
	std::cout << "质心" << pcaCentroid << std::endl;
	Eigen::Matrix3f covariance;
	pcl::computeCovarianceMatrixNormalized(*cloud_in, pcaCentroid, covariance);
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
	Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
	Eigen::Vector3f eigenValuesPCA = eigen_solver.eigenvalues();
	eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));
	eigenVectorsPCA.col(1) = eigenVectorsPCA.col(2).cross(eigenVectorsPCA.col(0));

	Eigen::Vector3f principal_x = eigenVectorsPCA.col(1);//设为点云的z轴

	Eigen::Affine3f transform1 = Eigen::Affine3f::Identity();
	Eigen::Affine3f transform2 = Eigen::Affine3f::Identity();
	Eigen::Vector3f principal_z = eigenVectorsPCA.col(0);
	//计算主方向到坐标系z轴的变换矩阵
	float alpha = atan(principal_z[1] / principal_z[2]);
	float phi = -1 * atan((principal_z[0] * cos(alpha)) / principal_z[2]);
	transform1.translation() << -1 * pcaCentroid[0], -1 * pcaCentroid[1], -1 * pcaCentroid[2];

	transform2.rotate(Eigen::AngleAxisf(alpha, Eigen::Vector3f::UnitX()));
	transform2.rotate(Eigen::AngleAxisf(phi, Eigen::Vector3f::UnitY()));
	transform = transform2*transform1;

	principal_x = transform1*eigenVectorsPCA.col(1);
	principal_x = transform2*principal_x;


//	Eigen::Affine3f transformr_x = Eigen::Affine3f::Identity();
	float r_x = -1 * atan(principal_x[1] / principal_x[0]);
	std::cout << "？？？  z=0?" << principal_x[2] << std::endl << "r_x" << r_x << std::endl;

	transformr_x.rotate(Eigen::AngleAxisf(r_x, Eigen::Vector3f::UnitZ()));


	pcl::transformPointCloud(*cloud_in, *cloud_out, transform);//transform 移动第一主方向与坐标系z轴重合
	pcl::transformPointCloud(*cloud_out, *cloud_out, transformr_x);//在上一步的基础上transformr_x 移动第二主方向与X轴重合
	z_rotate_by_x = alpha ;
	z_rotate_by_y = phi;
	x_rotate_by_z = r_x;


}


inline void centroid_to_origin(PointCloud::Ptr cloud_in, PointCloud::Ptr cloud_out,  float &z_rotate_by_x, float &z_rotate_by_y, float &x_rotate_by_z)
{
	Eigen::Affine3f transform=Eigen::Affine3f::Identity();
	Eigen::Affine3f transformr_x=Eigen::Affine3f::Identity();
	Eigen::Vector4f pcaCentroid;
	pcl::compute3DCentroid(*cloud_in, pcaCentroid);
	std::cout << "质心" << pcaCentroid << std::endl;
	Eigen::Matrix3f covariance;
	pcl::computeCovarianceMatrixNormalized(*cloud_in, pcaCentroid, covariance);
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
	Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
	Eigen::Vector3f eigenValuesPCA = eigen_solver.eigenvalues();
	eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));
	eigenVectorsPCA.col(1) = eigenVectorsPCA.col(2).cross(eigenVectorsPCA.col(0));

	Eigen::Vector3f principal_x = eigenVectorsPCA.col(1);//设为点云的z轴

	Eigen::Affine3f transform1 = Eigen::Affine3f::Identity();
	Eigen::Affine3f transform2 = Eigen::Affine3f::Identity();
	Eigen::Vector3f principal_z = eigenVectorsPCA.col(0);
	//计算主方向到坐标系z轴的变换矩阵
	float alpha = atan(principal_z[1] / principal_z[2]);
	float phi = -1 * atan((principal_z[0] * cos(alpha)) / principal_z[2]);
	transform1.translation() << -1 * pcaCentroid[0], -1 * pcaCentroid[1], -1 * pcaCentroid[2];

	transform2.rotate(Eigen::AngleAxisf(alpha, Eigen::Vector3f::UnitX()));
	transform2.rotate(Eigen::AngleAxisf(phi, Eigen::Vector3f::UnitY()));
	transform = transform2*transform1;

	principal_x = transform1*eigenVectorsPCA.col(1);
	principal_x = transform2*principal_x;


	//	Eigen::Affine3f transformr_x = Eigen::Affine3f::Identity();
	float r_x = -1 * atan(principal_x[1] / principal_x[0]);
	std::cout << "？？？  z=0?" << principal_x[2] << std::endl << "r_x" << r_x << std::endl;

	transformr_x.rotate(Eigen::AngleAxisf(r_x, Eigen::Vector3f::UnitZ()));


	pcl::transformPointCloud(*cloud_in, *cloud_out, transform);//transform 移动第一主方向与坐标系z轴重合
	pcl::transformPointCloud(*cloud_out, *cloud_out, transformr_x);//在上一步的基础上transformr_x 移动第二主方向与X轴重合
	z_rotate_by_x = alpha;
	z_rotate_by_y = phi;
	x_rotate_by_z = r_x;


}