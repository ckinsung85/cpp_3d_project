#include <iostream>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl/visualization/cloud_viewer.h>

/*
    Assign colormap to the pointcloud data for better visualization
*/

// Specify return type
void colormap(float t, int method, uint8_t& r, uint8_t& g, uint8_t& b){

    // One can design the various color rendering methods based on variable t
    switch (method) {

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

pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorOnDepth(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {

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
        int method = 1;  // Choose various colormap cases (1, 2, 3, ...)
        colormap(t, method, r, g, b);

        // Assign the color to the point cloud
        colored_cloud->points[i].r = r; // Red 
        colored_cloud->points[i].g = g; // Green
        colored_cloud->points[i].b = b; // Blue

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
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud2 = colorOnDepth(cloud2);

    // Visualize the point cloud
    pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");  
    viewer.showCloud(colored_cloud2);  
    while (!viewer.wasStopped()){}  

    return 0;
}