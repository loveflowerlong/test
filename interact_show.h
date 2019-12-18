#pragma once
//点云交互坐标显示
#include<iostream>
#include<fstream>
#include<pcl/io/pcd_io.h>
#include<pcl/point_cloud.h>
#include<pcl/point_types.h>
#include<vtkAutoInit.h>


VTK_MODULE_INIT(vtkRenderingOpenGL);
VTK_MODULE_INIT(vtkInteractionStyle);
VTK_MODULE_INIT(vtkRenderingFreeType);

#include<pcl/visualization/pcl_visualizer.h>

using namespace std;
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::Normal NormalT;
typedef pcl::PointCloud<NormalT> PointCloudNormal;

struct callback_args {
	PointCloud::Ptr clicked_points_3d;
	pcl::visualization::PCLVisualizer::Ptr viewerPtr;
};
void pp_callback(const pcl::visualization::PointPickingEvent& event, void *args)
{
	struct callback_args *data = (struct callback_args *) args;
	if (event.getPointIndex() == -1)
		return;
	PointT current_point;
	event.getPoint(current_point.x, current_point.y, current_point.z);
	data->clicked_points_3d->points.push_back(current_point);
	pcl::visualization::PointCloudColorHandlerCustom<PointT> red(data->clicked_points_3d, 255, 0, 0);
	data->viewerPtr->removePointCloud("clicked_points");
	data->viewerPtr->addPointCloud(data->clicked_points_3d, red, "clicked_points");
	data->viewerPtr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "clicked_points");
	cout << current_point.x << "  " << current_point.y << "  " << current_point.z << endl;
}

void interact_show(PointCloud::Ptr cloud_in) {
	boost::mutex cloud_mutex;
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer);
	viewer->addPointCloud(cloud_in, "interact_show");
	viewer->addCoordinateSystem(100);
	struct callback_args cb_args;
	PointCloud::Ptr clicked_points_3d(new PointCloud);
	cb_args.clicked_points_3d = clicked_points_3d;
	cb_args.viewerPtr = pcl::visualization::PCLVisualizer::Ptr(viewer);
	viewer->registerPointPickingCallback(pp_callback, (void*)&cb_args);
	viewer->spin();
	cloud_mutex.unlock();
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));

	}
}


