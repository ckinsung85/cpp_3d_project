#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/octree/octree.h>
#include <pcl/octree/octree_pointcloud.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include "filter.h"

// Filtering of the point cloud by statistical means above a threshold
void StatisticalOutlierRemovalFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input, 
                pcl::PointCloud<pcl::PointXYZ>::Ptr& inliers,
                pcl::PointCloud<pcl::PointXYZ>::Ptr& outliers) {
                
    // Invoke statistical outliers
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(input);
    sor.setMeanK(25);
    sor.setStddevMulThresh(1);

    // Stores filtered inliers
    sor.filter(*inliers);   

    // Stores outliers
    sor.setNegative(true);
    sor.filter(*outliers);

}

// Filtering of the point cloud based on radius below a threshold
void RadiusOutlierRemovalFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, 
                pcl::PointCloud<pcl::PointXYZ>::Ptr& inliers,
                pcl::PointCloud<pcl::PointXYZ>::Ptr& outliers){

    // Load point cloud data into cloud
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> ror;
    ror.setInputCloud(cloud);
    ror.setRadiusSearch(50); 
    ror.setMinNeighborsInRadius(25);

    // Stores filtered inliers
    ror.filter(*inliers);

    // Stores outliers
    ror.setNegative(true);
    ror.filter(*outliers);

}

// Filtering of the point cloud in KDTree format based on radius below a threshold
void RadiusOutlierRemovalKDTreeFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, 
                pcl::PointCloud<pcl::PointXYZ>::Ptr& inliers,
                pcl::PointCloud<pcl::PointXYZ>::Ptr& outliers){

    // Create KDTree
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud);

    // Define the search parameters
    float radius = 50; // Radius for outlier search
    int min_neighbors = 25; // Minimum number of neighbors for a point to be considered an inlier

    // Get inliers by searching for points within a radius
    std::vector<int> k_indices;
    std::vector<float> k_distances;
    for (size_t i = 0; i < cloud->points.size(); ++i) {
        if (kdtree.radiusSearch(cloud->points[i], radius, k_indices, k_distances) > min_neighbors) {
            inliers->push_back(cloud->points[i]);
        }else{
            outliers->push_back(cloud->points[i]);
        }
    }

}

// This is a bad program, totally useless, too slow (minutes), though works
void RadiusOutlierRemovalOctreeFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, 
                pcl::PointCloud<pcl::PointXYZ>::Ptr& inliers,
                pcl::PointCloud<pcl::PointXYZ>::Ptr& outliers){

    // Convert to octree
    double resolution = 100;
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(resolution);
    octree.setInputCloud(cloud);
    octree.addPointsFromInputCloud();

    // Parameters for outlier removal
    double radius = 50;  // Set search radius
    int min_neighbors = 25;  // Minimum number of neighbors for a point to be considered an inlier

    // Initialization 
    std::vector<int> k_indices;  // Vector to store point indices
    std::vector<float> k_distances;  // Vector to store corresponding point distances

    // Octree nearest neighbor search  
    for (size_t i = 0; i < cloud->points.size(); i++) {

        // Store the point as either inlier or outlier
        if (octree.radiusSearch(cloud->points[i], radius, k_indices, k_distances) > min_neighbors){
            inliers->push_back(cloud->points[i]);
        }else{
            outliers->push_back(cloud->points[i]);
        }
        
    }

}