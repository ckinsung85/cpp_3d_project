#include <iostream>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/kdtree/kdtree_flann.h>
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

    // Convert to kd-tree
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud);

    // kd-tree nearest neighbor search 
    // Create search point from centroid
    pcl::PointXYZ searchPoint;
    searchPoint.x = centroid[0];
    searchPoint.y = centroid[1];
    searchPoint.z = centroid[2];
    std::vector<int> k_indices(5);
    std::vector<float> k_sqr_distances(5);
    kdtree.nearestKSearch(searchPoint, 5, k_indices, k_sqr_distances);

    // Print kd-tree nearest neighbors
    std::cout << "KD-tree 5 nearest neighbors:" << std::endl;

    std::cout << "search point (centroid) X: " << searchPoint.x 
        << "  Y: " << searchPoint.y << "  Z: " << searchPoint.z << std::endl;

    for(int i = 0; i < k_indices.size(); ++i) {

        // Get the point based on the index
        pcl::PointXYZ pt = cloud->points[k_indices[i]];

        std::cout << "Point " << i + 1 << "; Index: " << k_indices[i] << std::endl;
        std::cout << "   X: " << pt.x << "  Y: " << pt.y << "  Z: " << pt.z << std::endl;
        std::cout << "   Distance: " << k_sqr_distances[i] << std::endl;

    }

    return 0;
}