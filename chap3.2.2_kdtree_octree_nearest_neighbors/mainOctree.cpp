#include <iostream>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/octree/octree.h>
#include <pcl/octree/octree_pointcloud.h>
#include <pcl/common/centroid.h>

int main(int argc, char** argv){

    // Filepath
    // std::string filename = "./ply_files/pointCloud.ply";
    std::string filename = argv[1];

    // Create point cloud pointer 
    // This creates a shared pointer cloud. pcl::PointCloud created dynamically on the heap
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());

    // Load the ply file
    pcl::io::loadPLYFile(filename, *cloud);

    // Calculate mean of points
	Eigen::Vector4f centroid;	
	pcl::compute3DCentroid(*cloud, centroid);

    // Convert to octree
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(0.1); 
    octree.setInputCloud(cloud);
    octree.addPointsFromInputCloud();

    // Octree nearest neighbor search 
    pcl::PointXYZ searchPoint;
    searchPoint.x = centroid[0];
    searchPoint.y = centroid[1];
    searchPoint.z = centroid[2];
    std::vector<int> o_indices(5);
    std::vector<float> o_distances(5);
    octree.nearestKSearch(searchPoint, 5, o_indices, o_distances);

    std::cout << "search point (centroid) X: " << searchPoint.x 
        << "  Y: " << searchPoint.y << "  Z: " << searchPoint.z << std::endl;
        
    // Print octree nearest neighbors  
    for(int i = 0; i < o_indices.size(); ++i) {

        // Get the point based on the index
        pcl::PointXYZ pt = cloud->points[o_indices[i]];

        std::cout << "Point " << i + 1 << "; Index: " << o_indices[i] << std::endl;
        std::cout << "   X: " << pt.x << "  Y: " << pt.y << "  Z: " << pt.z << std::endl;
        std::cout << "   Distance: " << o_distances[i] << std::endl;

    }

    return 0;
}