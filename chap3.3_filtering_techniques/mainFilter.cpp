#include <iostream>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include "color.h"
#include "filter.h"
#include <chrono>

/*
    This is the main function of the program.
    It reads the input point cloud and filters it using various methods
        1) StatisticalOutlierRemovalFilter (Stndard Deviation of nearest neighbors)
        2) RadiusOutlierRemovalFilter (Number of points inside a radius below a threshold)
        3) RadiusOutlierRemovalKDTreeFilter (Number of points inside a radius below a threshold using KDTree)
        4) RadiusOutlierRemovalOctreeFilter (Number of points inside a radius below a threshold using Octree)

    Example of usage:
        mkdir build && cd build && cmake.. && make 
        ./mainFilter ../../ply_files/pointCloud_center.ply
*/


int main(int argc, char** argv){

    // Start the timer
    auto start = std::chrono::high_resolution_clock::now();

    // Filepath
    // std::string filename = "./ply_files/pointCloud.ply";
    std::string filename = argv[1];   // .ply file path
    int method = std::stoi(argv[2]); // choice of filtering method to choose

    // Create point cloud pointer 
    // This creates a shared pointer cloud. pcl::PointCloud created dynamically on the heap
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_outliers(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_inliers(new pcl::PointCloud<pcl::PointXYZ>());

    // Load the ply file
    pcl::io::loadPLYFile(filename, *cloud);

    // Choose a filtering method
    switch (method) {
        case 1: 
            StatisticalOutlierRemovalFilter(cloud, cloud_inliers, cloud_outliers);
            break;
        case 2:
            RadiusOutlierRemovalFilter(cloud, cloud_inliers, cloud_outliers);
            break;
        case 3: 
            RadiusOutlierRemovalKDTreeFilter(cloud, cloud_inliers, cloud_outliers);
            break;
        case 4:
            RadiusOutlierRemovalOctreeFilter(cloud, cloud_inliers, cloud_outliers);
            break;
    }

    // Assign colormap to the point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_inliers = colorOnDepth(cloud_inliers, 0);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_outliers = colorOnDepth(cloud_outliers, -1);

    // Combine the inliers and outliers with different colors for better visualization
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr combined_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    *combined_cloud=*colored_inliers+*colored_outliers;

    // Print out the inliers, outliers and the combined cloud information
    std::cout << "inliers " << *colored_inliers << std::endl;
    std::cout << "outliers " << *colored_outliers << std::endl;
    std::cout << "combined " << *combined_cloud << std::endl;

    // Calculate the duration and print the runtime
    auto end = std::chrono::high_resolution_clock::now();  // End the timer
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cout << "Runtime: " << static_cast<float>(duration.count())/1000 << " seconds" << std::endl;  

    // Visualize the point cloud
    pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");  
    viewer.showCloud(combined_cloud);  
    while (!viewer.wasStopped()){}  
    
    return 0;
}