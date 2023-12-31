#include <iostream>  
#include <pcl/point_types.h>  
#include <pcl/visualization/cloud_viewer.h> // 追加  
  
int main (int argc, char** argv)  
{  
    pcl::PointCloud<pcl::PointXYZ> cloud;  

    // Fill in the cloud data  
    cloud.width    = 5;  
    cloud.height   = 1;  
    cloud.is_dense = false;  
    cloud.points.resize (cloud.width * cloud.height);  

    for (size_t i = 0; i < cloud.points.size (); ++i)  
    {  
        cloud.points[i].x = 1024 * rand () / (RAND_MAX + 1.0);  
        cloud.points[i].y = 1024 * rand () / (RAND_MAX + 1.0);  
        cloud.points[i].z = 1024 * rand () / (RAND_MAX + 1.0);  
    }  

    // Visualize the point cloud
    pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");  
    viewer.showCloud (cloud.makeShared());  
    while (!viewer.wasStopped ()) {  
    }  

    return (0);  
}  