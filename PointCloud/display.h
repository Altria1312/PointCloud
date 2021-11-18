#pragma once

#include <iostream>
#include <thread>
#include <string>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

// ʹ��CloudViewer���ӻ�
void fromViewer(std::string pcd_path);

// ʹ��Visualizer���ӻ�
void fromVisualizer(std::string pcd_path);
// ����������ʱ��
void MouseEvent(const pcl::visualization::MouseEvent &event, void* viewer_void);
void KeyboardEvent(const pcl::visualization::KeyboardEvent &event, void* viewer_void);

//��ͼ��
