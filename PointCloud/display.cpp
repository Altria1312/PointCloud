#include "display.h"

using namespace std::chrono_literals;
/******************************************************************/
void fromViewer(std::string pcd_path)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr data(new pcl::PointCloud<pcl::PointXYZ>);

	if (pcl::io::loadPCDFile(pcd_path, *data) == -1)
	{
		printf("Load Failed");
		return;
	}

	pcl::visualization::CloudViewer viewer("CloudViewer");
	viewer.showCloud(data);

	while (!viewer.wasStopped())
	{

	}
}

/******************************************************************/
void fromVisualizer(std::string pcd_path)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr data(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile(pcd_path, *data) == -1)
	{
		printf("Load failed");
		return;
	}
	else std::cout << "load " << pcd_path << " successful" << std::endl;

	pcl::visualization::PCLVisualizer viewer("visualizer");

	viewer.setBackgroundColor(0, 0, 0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color1(data, 255, 255, 255);
	viewer.addPointCloud(data, single_color1, "rabbit1");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "rabbit1");
	viewer.addCoordinateSystem(0.5);
	viewer.initCameraParameters();
	viewer.registerMouseCallback(MouseEvent, static_cast<void*>(&viewer));
	viewer.registerKeyboardCallback(KeyboardEvent, static_cast<void*>(&viewer));

	viewer.addLine((*data)[0], (*data)[100], 255,0, 0, "line");

	while (!viewer.wasStopped())
	{
		viewer.spinOnce(100);
		std::this_thread::sleep_for(100ms);
	}

	int k = 0;
}

void MouseEvent(const pcl::visualization::MouseEvent & event, void * viewer_void)
{
	pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer *> (viewer_void);
	if (event.getButton() == pcl::visualization::MouseEvent::LeftButton &&
		event.getType() == pcl::visualization::MouseEvent::MouseButtonRelease)
	{
		char str[128];
		sprintf(str, "%d %d pressed", event.getX(), event.getY());
		viewer->addText(str, event.getX(), event.getY(), "text");
	}
}

void KeyboardEvent(const pcl::visualization::KeyboardEvent & event, void * viewer_void)
{
	pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer *> (viewer_void);
	if (event.getKeySym() == "d" && event.keyDown())
	{
		viewer->removeShape("text");
	}
}

/******************************************************************/
