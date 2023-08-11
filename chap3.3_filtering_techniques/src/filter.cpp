#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include "filter.h"


void StatisticalOutlierRemovalFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input, 
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
