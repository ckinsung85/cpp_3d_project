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

    // Print octree nearest neighbors  
    std::cout << "Octree 5 nearest neighbors:" << std::endl;
    for(int i = 0; i < o_indices.size(); ++i) {
        std::cout << "Index: " << o_indices[i] << " Distance: " << o_distances[i] << std::endl;
    }

    return 0;
}