#include "add_attribute.hpp"

namespace offline_data{

    //TODO make custom data type to include information on about view point etc.
    pcl::PointCloud<pcl::PointWithRange> add_range(pcl::PointCloud<pcl::PointXYZ> cloud){

        // define variable
        pcl::PointCloud<pcl::PointWithRange> cloud_range;

        // loop over all points in the cloud
        for (int p = 0; p < cloud.points.size(); ++p) {
            
            // prepare point
            pcl::PointWithRange point;
            point.x = cloud.points[p].x;
            point.y = cloud.points[p].y;
            point.z = cloud.points[p].z;
            point.range = std::sqrt(std::pow(point.x, 2) + std::pow(point.y, 2) + std::pow(point.z, 2));

            // add to new cloud
            cloud_range.push_back(point);
        };

        // return
        return cloud_range;
    };




template <typename PointT>
sensor_msgs::PointCloud2 msg_filter(pcl::Filter<PointT> & filter, const sensor_msgs::PointCloud2 & msg_input){
    
    // ================ define variables =================
    typename pcl::PointCloud<PointT>::Ptr pc_input    (new pcl::PointCloud<PointT>);
    typename pcl::PointCloud<PointT>::Ptr pc_output   (new pcl::PointCloud<PointT>);
    sensor_msgs::PointCloud2 msg_output;
    // ===================================================

    
    // ================ convert msg -> pc ================
    pcl::fromROSMsg(msg_input, *pc_input);
    // ===================================================
    

    // ===================== Filter ======================
    filter.setInputCloud(pc_input);
    filter.filter(*pc_output);
    // ===================================================


    // ================ convert pc -> msg ================
    pcl::toROSMsg(*pc_output, msg_output);
    // ===================================================


    // ===================== return ======================
    return msg_output;
    // ===================================================
};



};
