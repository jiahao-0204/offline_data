#pragma once

#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/filters/voxel_grid.h>
#include <pcl_ros/filters/passthrough.h>
#include <pcl_ros/filters/extract_indices.h>


namespace offline_data{
    pcl::PointCloud<pcl::PointWithRange> add_range(pcl::PointCloud<pcl::PointXYZ> cloud);
};
