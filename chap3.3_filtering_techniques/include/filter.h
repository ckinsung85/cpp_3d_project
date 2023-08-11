#pragma once
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>


void StatisticalOutlierRemovalFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input, 
                pcl::PointCloud<pcl::PointXYZ>::Ptr& inliers,
                pcl::PointCloud<pcl::PointXYZ>::Ptr& outliers);