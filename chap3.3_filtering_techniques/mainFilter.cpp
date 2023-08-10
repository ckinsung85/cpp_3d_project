#include <iostream>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/statistical_outlier_removal.h>
// #include "colorOnDepth.h"


void colormap(float t, int method, uint8_t& r, uint8_t& g, uint8_t& b){

    // One can design the various color rendering methods based on variable t
    switch (method) {

        case -1: 

            // All red
            r = 255; // Red 
            g = 0; // Green
            b = 0; // Blue
            break;

        case 0: 

            // All white
            r = 255; // Red 
            g = 255; // Green
            b = 255; // Blue
            break;

        case 1: 

            // Grayscale rendering intensity =  t * 255
            r = t * 255; // Red 
            g = t * 255; // Green
            b = t * 255; // Blue
            break;

        case 2:

            // Rainbow rendering
            r = (1 - t) * 255; // Red 
            g = 0.5 * 255; // Green
            b = t * 255; // Blue
            break;
            
        case 3:

            // Heatmap rendering
            r = t * 255; // Red 
            g = (1 - t) * 255; // Green
            b = 0; // Blue
            break;

        case 4:

            // Inverted Heatmap rendering
            r = (1 - t) * 255; // Red 
            g = t * 255; // Green
            b = 0; // Blue
            break;

        case 5:
        
            // Blue to red rainbow
            r = t * 255;
            g = 0; 
            b = (1 - t) * 255;
            break;

        case 6: 

            // Cool to warm
            r = t * 255;
            g = t * 64;  
            b = (1 - t) * 255;
            break;

        case 7:

            // Red-green color blindness friendly
            r = 0;
            g = t * 255;
            b = (1 - t) * 255;
            break;

        case 8:

            // Ocean color
            r = 0;
            g = t * 127;
            b = (1 - t) * 255;
            break;

    }
 
}


pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorOnDepth(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int method) {

    // Define the point cloud data
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());

    // Get XYZ points
    colored_cloud->points.resize(cloud->points.size());
    for(size_t i = 0; i < cloud->points.size(); i++) {
        colored_cloud->points[i].x = cloud->points[i].x;
        colored_cloud->points[i].y = cloud->points[i].y;
        colored_cloud->points[i].z = cloud->points[i].z; 
    }

    // Define min and max depth
    float min_z = std::numeric_limits<float>::max();
    float max_z = std::numeric_limits<float>::min();

    // Obtain the min max value of the point cloud data
    for(const auto& pt : cloud->points) {
        min_z = std::min(min_z, pt.z);
        max_z = std::max(max_z, pt.z);
    }

    // Color each point
    for(size_t i = 0; i < cloud->points.size(); i++) {

        float z = cloud->points[i].z;

        // Map depth to 0-1 range
        float t = (z - min_z) / (max_z - min_z);

        // Choose a colormap for rendering the point cloud data (return r, g, b uint8 values)
        uint8_t r, g, b;
        colormap(t, method, r, g, b);

        // Assign the color to the point cloud
        colored_cloud->points[i].r = r; // Red 
        colored_cloud->points[i].g = g; // Green
        colored_cloud->points[i].b = b; // Blue

    }

    return colored_cloud;

}


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