#include <iostream>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl/visualization/cloud_viewer.h>

pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorOnDepth(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());

    // Get XYZ points
    colored_cloud->points.resize(cloud->points.size());
    for(size_t i = 0; i < cloud->points.size(); i++) {
        colored_cloud->points[i].x = cloud->points[i].x;
        colored_cloud->points[i].y = cloud->points[i].y;
        colored_cloud->points[i].z = cloud->points[i].z; 
    }

    for(size_t i = 0; i < colored_cloud->points.size(); i++) {

        // Map depth (z) to RGB
        float depth = colored_cloud->points[i].z;
        int r = (depth * 255) / 20;
        int g = ((depth * 255) / 20) * 0.5;
        int b = 255 - ((depth * 255) / 20);

        colored_cloud->points[i].r = r;
        colored_cloud->points[i].g = g;
        colored_cloud->points[i].b = b;
    }

    return colored_cloud;

}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorOnDepth2(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());

    // Get XYZ points
    colored_cloud->points.resize(cloud->points.size());
    for(size_t i = 0; i < cloud->points.size(); i++) {
        colored_cloud->points[i].x = cloud->points[i].x;
        colored_cloud->points[i].y = cloud->points[i].y;
        colored_cloud->points[i].z = cloud->points[i].z; 
    }

    // Get min and max depth
    float min_z = std::numeric_limits<float>::max();
    float max_z = std::numeric_limits<float>::min();

    for(const auto& pt : cloud->points) {
        min_z = std::min(min_z, pt.z);
        max_z = std::max(max_z, pt.z);
    }

    // Color each point
    for(size_t i = 0; i < cloud->points.size(); i++) {

        float z = cloud->points[i].z;

        // Map depth to 0-1 range
        float t = (z - min_z) / (max_z - min_z);

        // Assign color based on t
        colored_cloud->points[i].r = (1 - t) * 255; // Red 
        colored_cloud->points[i].g = 100;
        colored_cloud->points[i].b = t * 255; // Blue

    }

    return colored_cloud;

}

int main(int argc, char** argv){

    // Filepath
    // std::string filename = "./ply_files/pointCloud_1.ply";
    std::string filename = argv[1];

    // Create point cloud pointer 
    // This creates a shared pointer cloud2 to a pcl::PointCloud created dynamically on the heap
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>());

    // Load the ply file
    pcl::io::loadPLYFile(filename, *cloud1);

    // Calculate mean of points
	Eigen::Vector4f centroid1;	
	pcl::compute3DCentroid(*cloud1, centroid1);

    // Minus the point cloud from its calculated centroid
    pcl::demeanPointCloud(*cloud1, centroid1, *cloud2);

    // Computer the new centroid values after deduction from the centroid 
	Eigen::Vector4f centroid2;	
	pcl::compute3DCentroid(*cloud2, centroid2);

    // Print out the centroid before and after
    std::cout << "centroid1 = " << centroid1 << std::endl;
    std::cout << "centroid2 = " << centroid2 << std::endl;

    // // Print out information of the point cloud
    std::cout << "Point cloud data: " << std::endl;
    std::cout << *cloud1 << std::endl;

    // Print first point
    std::cout << cloud1->points[0].x << " " << cloud1->points[0].y << " " << cloud1->points[0].z << std::endl;
    std::cout << cloud2->points[0].x << " " << cloud2->points[0].y << " " << cloud2->points[0].z << std::endl;

    // Add color to the point cloud
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud2 = colorOnDepth(cloud2);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud2 = colorOnDepth2(cloud2);

    // Visualize the point cloud
    pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");  
    viewer.showCloud(colored_cloud2);  
    while (!viewer.wasStopped()){}  

    return 0;
}