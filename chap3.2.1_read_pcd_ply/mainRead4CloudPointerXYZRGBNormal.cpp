#include <iostream>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl/visualization/cloud_viewer.h>

/*
    Reads an XYZRGBNormal cloud and visualize it

    CloudViewer only accepts XYZ or XYZRGB data type point cloud. TWo solutions:
    1) Read the .ply file as XYZRGB data type 
    2) Read the .ply file as XYZRGBNormal data type and extract the XYZRGB components to a variable

    This program choose the 2nd type
*/

pcl::PointCloud<pcl::PointXYZRGB>::Ptr genCloudXYZRGB(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud){

    // Extract RGB part 
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb(new pcl::PointCloud<pcl::PointXYZRGB>());

    // Fill in XYZ and RGB
    for(const auto& p : cloud->points) {

        //  Creates a single point variable of type pcl::PointXYZRGB
        // For temporary usage, it is fine (rgb_pt is just tmp)
        pcl::PointXYZRGB rgb_pt;

        // Assign RGB from original point
        rgb_pt.x = p.x;
        rgb_pt.y = p.y;
        rgb_pt.z = p.z;
        rgb_pt.r = p.r;
        rgb_pt.g = p.g;
        rgb_pt.b = p.b;

        // Now push back this new RGB point
        cloud_rgb->points.push_back(rgb_pt);
    }

    return cloud_rgb;

}

int main(int argc, char** argv){

    // Filepath (pointCloud.ply has XYZRGBNormal components)
    // std::string filename = "./ply_files/pointCloud.ply";  
    std::string filename = argv[1];

    // Create point cloud pointer 
    // This creates a shared pointer cloud. pcl::PointCloud created dynamically on the heap
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZRGBNormal>());
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZRGBNormal>());

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

    // Print out information of the point cloud
    std::cout << "Point cloud data: " << std::endl;
    std::cout << *cloud1 << std::endl;

    // Print first point
    std::cout << cloud1->points[0].x << " " << cloud1->points[0].y << " " << cloud1->points[0].z << std::endl;
    std::cout << cloud2->points[0].x << " " << cloud2->points[0].y << " " << cloud2->points[0].z << std::endl;

    // Extract XYZRGB from XYZRGBNormal point cloud data
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb = genCloudXYZRGB(cloud2);

    // Visualize the point cloud (CloudViewer only accpet data type: XYZ or XYZRGB format, not XYZRGBNormal)
    pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");  
    viewer.showCloud(cloud_rgb);  
    while (!viewer.wasStopped()){}  

    return 0;
}