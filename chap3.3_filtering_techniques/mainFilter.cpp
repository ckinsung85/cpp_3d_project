#include <iostream>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include "color.h"


void sorFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input, 
                pcl::PointCloud<pcl::PointXYZ>::Ptr& inliers,
                pcl::PointCloud<pcl::PointXYZ>::Ptr& outliers) {
                
    // Invoke statistical outliers
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(input);
    sor.setMeanK(50);
    sor.setStddevMulThresh(0.5);

    // Stores filtered inliers
    sor.filter(*inliers);   

    // Stores outliers
    sor.setNegative(true);
    sor.filter(*outliers);

}


int main(int argc, char** argv){

    // Filepath
    // std::string filename = "./ply_files/pointCloud.ply";
    std::string filename = argv[1];

    // Create point cloud pointer 
    // This creates a shared pointer cloud. pcl::PointCloud created dynamically on the heap
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_outliers(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_inliers(new pcl::PointCloud<pcl::PointXYZ>());

    // Load the ply file
    pcl::io::loadPLYFile(filename, *cloud);

    // Generate the inliers and outliers based on statistical filtering
    sorFilter(cloud, cloud_inliers, cloud_outliers);

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

    // Visualize the point cloud
    pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");  
    viewer.showCloud(combined_cloud);  
    while (!viewer.wasStopped()){}  
    
    return 0;
}