#pragma once

#include <iostream>
#include <thread>
#include <string>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

// 使用CloudViewer可视化
void fromViewer(std::string pcd_path);

// 使用Visualizer可视化
void fromVisualizer(std::string pcd_path);
// 加入鼠标键盘时间
void MouseEvent(const pcl::visualization::MouseEvent &event, void* viewer_void);
void KeyboardEvent(const pcl::visualization::KeyboardEvent &event, void* viewer_void);

//话图表
