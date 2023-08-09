#include <iostream>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl/visualization/cloud_viewer.h>

int main(int argc, char** argv){

    // Filepath
    // std::string filename = "./ply_files/pointCloud.ply";
    std::string filename = argv[1];

    // Create point cloud pointer
    // This creates a pcl::PointCloud object directly on the stack
    pcl::PointCloud<pcl::PointXYZ> cloud1;
    pcl::PointCloud<pcl::PointXYZ> cloud2;

    // Load the ply file
    pcl::io::loadPLYFile(filename, cloud1);

    // Calculate mean of points
	Eigen::Vector4f centroid1;	
	pcl::compute3DCentroid(cloud1, centroid1);

    // Minus the point cloud from its calculated centroid
    pcl::demeanPointCloud(cloud1, centroid1, cloud2);

    // Computer the new centroid values after deduction from the centroid 
	Eigen::Vector4f centroid2;	
	pcl::compute3DCentroid(cloud2, centroid2);

    // Print out the centroid before and after
    std::cout << "centroid1 = " << centroid1 << std::endl;
    std::cout << "centroid2 = " << centroid2 << std::endl;

    // Print out information of the point cloud
    std::cout << "Point cloud data: " << std::endl;
    std::cout << cloud1 << std::endl;

    // Print first point
    std::cout << cloud1.points[0].x << " " << cloud1.points[0].y << " " << cloud1.points[0].z << std::endl;
    std::cout << cloud2.points[0].x << " " << cloud2.points[0].y << " " << cloud2.points[0].z << std::endl;

    // Visualize the point cloud
    pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");  
    viewer.showCloud(cloud2.makeShared());  
    while (!viewer.wasStopped()){}  

    return 0;
}