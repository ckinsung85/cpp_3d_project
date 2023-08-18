#pragma once
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/octree/octree.h>
#include <pcl/octree/octree_pointcloud.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>


void StatisticalOutlierRemovalFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input, 
                pcl::PointCloud<pcl::PointXYZ>::Ptr& inliers,
                pcl::PointCloud<pcl::PointXYZ>::Ptr& outliers);

void RadiusOutlierRemovalFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, 
                pcl::PointCloud<pcl::PointXYZ>::Ptr& inliers,
                pcl::PointCloud<pcl::PointXYZ>::Ptr& outliers);

void RadiusOutlierRemovalKDTreeFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input, 
                pcl::PointCloud<pcl::PointXYZ>::Ptr& inliers,
                pcl::PointCloud<pcl::PointXYZ>::Ptr& outliers);

void RadiusOutlierRemovalOctreeFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, 
                pcl::PointCloud<pcl::PointXYZ>::Ptr& inliers,
                pcl::PointCloud<pcl::PointXYZ>::Ptr& outliers);

/*
    Add more filters including 
    voxelize filter
    range conditioner filter
    passthrough filter (ROI filter)
*/