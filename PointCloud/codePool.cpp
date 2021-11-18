#include "codePool.h"
#include <iostream>
#include <vector>
#include <string>
#include <ctime>
#include <pcl/point_cloud.h>
#include <pcl/octree/octree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <boost/thread/thread.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/on_nurbs/fitting_surface_tdm.h>
#include <pcl/surface/on_nurbs/fitting_curve_2d_asdm.h>
#include <pcl/surface/on_nurbs/triangulation.h>
#include <pcl/io/vtk_io.h>


// 测试pcl安装成功
int test()
{
	srand((unsigned int)time(NULL));
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	// 创建点云数据
	cloud->width = 1000;
	cloud->height = 1;
	cloud->points.resize(cloud->width * cloud->height);
	for (size_t i = 0; i < cloud->points.size(); ++i)
	{
		cloud->points[i].x = 1024.0f * rand() / (RAND_MAX + 1.0f);
		cloud->points[i].y = 1024.0f * rand() / (RAND_MAX + 1.0f);
		cloud->points[i].z = 1024.0f * rand() / (RAND_MAX + 1.0f);
	}

	pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(0.1);
	octree.setInputCloud(cloud);
	octree.addPointsFromInputCloud();
	pcl::PointXYZ searchPoint;
	searchPoint.x = 1024.0f * rand() / (RAND_MAX + 1.0f);
	searchPoint.y = 1024.0f * rand() / (RAND_MAX + 1.0f);
	searchPoint.z = 1024.0f * rand() / (RAND_MAX + 1.0f);

	//半径内近邻搜索
	std::vector<int>pointIdxRadiusSearch;
	std::vector<float>pointRadiusSquaredDistance;
	float radius = 256.0f * rand() / (RAND_MAX + 1.0f);
	cout << "Neighbors within radius search at (" << searchPoint.x
		<< " " << searchPoint.y
		<< " " << searchPoint.z
		<< ") with radius=" << radius << endl;
	if (octree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
	{
		for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
			cout << "    " << cloud->points[pointIdxRadiusSearch[i]].x
			<< " " << cloud->points[pointIdxRadiusSearch[i]].y
			<< " " << cloud->points[pointIdxRadiusSearch[i]].z
			<< " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << endl;
	}
	// 初始化点云可视化对象
	boost::shared_ptr<pcl::visualization::PCLVisualizer>viewer(new pcl::visualization::PCLVisualizer("显示点云"));
	viewer->setBackgroundColor(0, 0, 0);  //设置背景颜色为黑色
	// 对点云着色可视化 (red).
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>target_color(cloud, 255, 0, 0);
	viewer->addPointCloud<pcl::PointXYZ>(cloud, target_color, "target cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "target cloud");

	// 等待直到可视化窗口关闭
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(1000));
	}

	return (0);
}
// 创建、保存点云数据
void create_write(void)
{
	pcl::PointCloud<pcl::PointXYZ> cloud;

	cloud.width = 5;
	cloud.height = 1;
	cloud.is_dense = false;
	cloud.points.resize(cloud.width * cloud.height);

	for (std::size_t i = 0; i < cloud.points.size(); ++i)
	{
		cloud.points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud.points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud.points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
	}

	pcl::io::savePCDFileASCII("./random.pcd", cloud);
	std::cerr << "Saved Cerr" << std::endl;
	std::cout << "Saved Cout" << std::endl;

}
// 读取点云数据
void read_pcd(void)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	
	if (pcl::io::loadPCDFile<pcl::PointXYZ>("./random.pcd", *cloud) == -1)
	{
		std::cout << "Read Fail" << std::endl;
		PCL_ERROR("could not read");
		return;
	}

	std::cout << "Total: " << cloud->width << '*' << cloud->height << "points\n";
	for (std::size_t i = 0; i < cloud->points.size(); ++i)
	{
		std::cout << cloud->points[i].x << ' ' <<cloud->points[i].y << ' ' << cloud->points[i].z <<'\n';
	}

}

//pcd数据显示
void show_PCD(std::string path)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	if (pcl::io::loadPCDFile(path, *cloud) == -1)
	{
		std::cout << "Read Fail" << std::endl;
		return;
	}

	pcl::visualization::CloudViewer viewer("show cloud");
	viewer.showCloud(cloud);

	while (!viewer.wasStopped())
	{

	}
}

void show_PCD_2(std::string path)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	if (pcl::io::loadPCDFile(path, *cloud) == -1)
	{
		std::cout << "Read Fail" << std::endl;
		return;
	}

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("PCLVisualizer"));
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> green(cloud, 0, 255, 0);

	viewer->setBackgroundColor(0, 0, 0);
	viewer->addPointCloud<pcl::PointXYZ>(cloud, green, "1");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "1");
	viewer->addCoordinateSystem(0.1);

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(1000));
	}

	int i = 0;
}

//重建曲面
void reconstruction(std::string path)
{
	//读取点云数据
	pcl::PointCloud<pcl::PointXYZ>::Ptr data(new pcl::PointCloud<pcl::PointXYZ>());
	if (pcl::io::loadPCDFile(path, *data))
	{
		std::cout << "Read faile" << std::endl;
		return;
	}

	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointNormal> pts_norm;

	mls.setComputeNormals(true);
	mls.setInputCloud(data);
	mls.setPolynomialFit(true);
	mls.setSearchMethod(tree);
	mls.setSearchRadius(0.03);

	mls.process(pts_norm);

	pcl::io::savePCDFile("./bn_mls.pcd", pts_norm);

	show_PCD_2("./bn_mls.pcd");
}

void trangulation(std::string path)
{
	//读取点云数据
	pcl::PointCloud<pcl::PointXYZ>::Ptr data(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PCLPointCloud2 data_blob;
	if (pcl::io::loadPCDFile(path, data_blob))
	{
		std::cout << "Read faile" << std::endl;
		return;
	}

	pcl::fromPCLPointCloud2(data_blob, *data);

	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

	tree->setInputCloud(data);
	n.setInputCloud(data);
	n.setSearchMethod(tree);
	n.setKSearch(20);
	n.compute(*normals);

	pcl::PointCloud<pcl::PointNormal>::Ptr data_norm(new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields(*data, *normals, *data_norm);

	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
	tree2->setInputCloud(data_norm);

	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
	pcl::PolygonMesh triangles;

	gp3.setInputCloud(data_norm);
	gp3.setSearchRadius(0.025);
	gp3.setMu(2.5);
	gp3.setMaximumNearestNeighbors(100);
	gp3.setMaximumSurfaceAngle(M_PI / 4);
	gp3.setMinimumAngle(M_PI / 18);
	gp3.setMaximumAngle(2 *M_PI / 3);
	gp3.setNormalConsistency(false);

	gp3.setInputCloud(data_norm);
	gp3.setSearchMethod(tree2);
	gp3.reconstruct(triangles);

	std::vector<int> parts = gp3.getPartIDs();
	std::vector<int> states = gp3.getPointStates();

	pcl::io::saveVTKFile("./view.vtk", triangles);
}


void PointCloud2Vector3d(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::on_nurbs::vector_vec3d &data)
{
	for (unsigned i = 0; i < cloud->size(); i++)
	{
		pcl::PointXYZ &p = cloud->at(i);
		if (!std::isnan(p.x) && !std::isnan(p.y) && !std::isnan(p.z))
			data.push_back(Eigen::Vector3d(p.x, p.y, p.z));
	}
}

void visualizeCurve(ON_NurbsCurve &curve, ON_NurbsSurface &surface, pcl::visualization::PCLVisualizer &viewer)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr curve_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	pcl::on_nurbs::Triangulation::convertCurve2PointCloud(curve, surface, curve_cloud, 4);
	for (std::size_t i = 0; i < curve_cloud->size() - 1; i++)
	{
		pcl::PointXYZRGB &p1 = curve_cloud->at(i);
		pcl::PointXYZRGB &p2 = curve_cloud->at(i + 1);
		std::ostringstream os;
		os << "line" << i;
		viewer.removeShape(os.str());
		viewer.addLine<pcl::PointXYZRGB>(p1, p2, 1.0, 0.0, 0.0, os.str());
	}

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr curve_cps(new pcl::PointCloud<pcl::PointXYZRGB>);
	for (int i = 0; i < curve.CVCount(); i++)
	{
		ON_3dPoint p1;
		curve.GetCV(i, p1);

		double pnt[3];
		surface.Evaluate(p1.x, p1.y, 0, 3, pnt);
		pcl::PointXYZRGB p2;
		p2.x = float(pnt[0]);
		p2.y = float(pnt[1]);
		p2.z = float(pnt[2]);

		p2.r = 255;
		p2.g = 0;
		p2.b = 0;

		curve_cps->push_back(p2);
	}
	viewer.removePointCloud("cloud_cps");
	viewer.addPointCloud(curve_cps, "cloud_cps");
}

int triangle(std::string pcd_file)
{
	std::string file_3dm = "./bunny_3d", vtk_file = "./bunny.vtk";


	pcl::visualization::PCLVisualizer viewer("B-spline surface fitting");
	viewer.setSize(800, 600);

	// ############################################################################
	// load point cloud

	printf("  loading %s\n", pcd_file.c_str());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PCLPointCloud2 cloud2;
	pcl::on_nurbs::NurbsDataSurface data;

	if (pcl::io::loadPCDFile(pcd_file, cloud2) == -1)
		throw std::runtime_error("  PCD file not found.");

	fromPCLPointCloud2(cloud2, *cloud);
	PointCloud2Vector3d(cloud, data.interior);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> handler(cloud, 0, 255, 0);
	viewer.addPointCloud<pcl::PointXYZ>(cloud, handler, "cloud_cylinder");
	printf("  %lu points in data set\n", cloud->size());

	// ############################################################################
	// fit B-spline surface

	// parameters
	unsigned order(3);
	unsigned refinement(5);
	unsigned iterations(10);
	unsigned mesh_resolution(256);

	pcl::on_nurbs::FittingSurface::Parameter params;
	params.interior_smoothness = 0.2;
	params.interior_weight = 1.0;
	params.boundary_smoothness = 0.2;
	params.boundary_weight = 0.0;

	// initialize
	printf("  surface fitting ...\n");
	ON_NurbsSurface nurbs = pcl::on_nurbs::FittingSurface::initNurbsPCABoundingBox(order, &data);
	pcl::on_nurbs::FittingSurface fit(&data, nurbs);
	//  fit.setQuiet (false); // enable/disable debug output

	// mesh for visualization
	pcl::PolygonMesh mesh;
	pcl::PointCloud<pcl::PointXYZ>::Ptr mesh_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	std::vector<pcl::Vertices> mesh_vertices;
	std::string mesh_id = "mesh_nurbs";
	pcl::on_nurbs::Triangulation::convertSurface2PolygonMesh(fit.m_nurbs, mesh, mesh_resolution);
	viewer.addPolygonMesh(mesh, mesh_id);

	// surface refinement
	for (unsigned i = 0; i < refinement; i++)
	{
		fit.refine(0);
		fit.refine(1);
		fit.assemble(params);
		fit.solve();
		pcl::on_nurbs::Triangulation::convertSurface2Vertices(fit.m_nurbs, mesh_cloud, mesh_vertices, mesh_resolution);
		viewer.updatePolygonMesh<pcl::PointXYZ>(mesh_cloud, mesh_vertices, mesh_id);
		viewer.spinOnce();
	}

	// surface fitting with final refinement level
	for (unsigned i = 0; i < iterations; i++)
	{
		fit.assemble(params);
		fit.solve();
		pcl::on_nurbs::Triangulation::convertSurface2Vertices(fit.m_nurbs, mesh_cloud, mesh_vertices, mesh_resolution);
		viewer.updatePolygonMesh<pcl::PointXYZ>(mesh_cloud, mesh_vertices, mesh_id);
		viewer.spinOnce();
	}

	// ############################################################################
	// fit B-spline curve

	// parameters
	pcl::on_nurbs::FittingCurve2dAPDM::FitParameter curve_params;
	curve_params.addCPsAccuracy = 5e-2;
	curve_params.addCPsIteration = 3;
	curve_params.maxCPs = 200;
	curve_params.accuracy = 1e-3;
	curve_params.iterations = 100;

	curve_params.param.closest_point_resolution = 0;
	curve_params.param.closest_point_weight = 1.0;
	curve_params.param.closest_point_sigma2 = 0.1;
	curve_params.param.interior_sigma2 = 0.00001;
	curve_params.param.smooth_concavity = 1.0;
	curve_params.param.smoothness = 1.0;

	// initialisation (circular)
	printf("  curve fitting ...\n");
	pcl::on_nurbs::NurbsDataCurve2d curve_data;
	curve_data.interior = data.interior_param;
	curve_data.interior_weight_function.push_back(true);
	ON_NurbsCurve curve_nurbs = pcl::on_nurbs::FittingCurve2dAPDM::initNurbsCurve2D(order, curve_data.interior);

	// curve fitting
	pcl::on_nurbs::FittingCurve2dASDM curve_fit(&curve_data, curve_nurbs);
	// curve_fit.setQuiet (false); // enable/disable debug output
	curve_fit.fitting(curve_params);
	visualizeCurve(curve_fit.m_nurbs, fit.m_nurbs, viewer);

	// ############################################################################
	// triangulation of trimmed surface

	printf("  triangulate trimmed surface ...\n");
	viewer.removePolygonMesh(mesh_id);
	pcl::on_nurbs::Triangulation::convertTrimmedSurface2PolygonMesh(fit.m_nurbs, curve_fit.m_nurbs, mesh,
		mesh_resolution);
	viewer.addPolygonMesh(mesh, mesh_id);

	pcl::io::saveVTKFile("./view.vtk", mesh);
	// save trimmed B-spline surface
	/*if (fit.m_nurbs.IsValid())
	{
		ONX_Model model;
		ONX_Model_Object& surf = model.m_object_table.AppendNew();
		surf.m_object = new ON_NurbsSurface(fit.m_nurbs);
		surf.m_bDeleteObject = true;
		surf.m_attributes.m_layer_index = 1;
		surf.m_attributes.m_name = "surface";

		ONX_Model_Object& curv = model.m_object_table.AppendNew();
		curv.m_object = new ON_NurbsCurve(curve_fit.m_nurbs);
		curv.m_bDeleteObject = true;
		curv.m_attributes.m_layer_index = 2;
		curv.m_attributes.m_name = "trimming curve";

		model.Write(file_3dm.c_str());
		printf("  model saved: %s\n", file_3dm.c_str());
	}*/

	printf("  ... done.\n");

	viewer.spin();
	return 0;
}
