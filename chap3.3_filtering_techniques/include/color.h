#pragma once
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>

void colormap(float t, int method, uint8_t& r, uint8_t& g, uint8_t& b);


pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorOnDepth(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int method);