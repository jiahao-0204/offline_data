#include <string>
#include <iostream>
#include <sstream>

#include <fstream>



#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/filters/passthrough.h>


#include <pcl/io/pcd_io.h>



int main(int argc, char **argv){

    // =================== ARGUMENTS =====================
    std::string path_input = argv[1];
    float step = std::stof(std::string(argv[2]));
    // ===================================================

    std::cout<<"test1"<<std::endl;
    // ===================== NODE ========================
    ros::init(argc, argv, "visualize");
    ros::NodeHandle nh;
    ros::Rate rate(10);
    // ===================================================


    // ================== PUBLISHERS =====================
    ros::Publisher pub_cloud = nh.advertise<sensor_msgs::PointCloud2> ("all_ds", 100, true);
    std::cout<<"test2"<<std::endl;

    // display by range
    ros::Publisher pub_range1 = nh.advertise<sensor_msgs::PointCloud2> ("range1", 100, true);
    ros::Publisher pub_range2 = nh.advertise<sensor_msgs::PointCloud2> ("range2", 100, true);
    ros::Publisher pub_range3 = nh.advertise<sensor_msgs::PointCloud2> ("range3", 100, true);
    ros::Publisher pub_range4 = nh.advertise<sensor_msgs::PointCloud2> ("range4", 100, true);
    ros::Publisher pub_range5 = nh.advertise<sensor_msgs::PointCloud2> ("range5", 100, true);
    // ===================================================
    std::cout<<"test3"<<std::endl;



    // =============== READ POINTCLOUD DATA ==============
    pcl::PointCloud<pcl::PointWithRange> cloud;
    pcl::io::loadPCDFile<pcl::PointWithRange>(path_input, cloud);
    // ===================================================

    std::cout<<"test4"<<std::endl;

    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud, cloud_msg);
    cloud_msg.header.frame_id = "map";
    pub_cloud.publish(cloud_msg);

    std::cout<<"test5"<<std::endl;



    // ================ PUBLISH BY RANGE =================
    pcl::PassThrough<pcl::PointWithRange> filter_rg;
    filter_rg.setFilterFieldName("range");

    std::cout<<"test6"<<std::endl;

    boost::shared_ptr<pcl::PointCloud<pcl::PointWithRange>> cloud_ptr(&cloud);
    filter_rg.setInputCloud(cloud_ptr);

    std::cout<<"test7"<<std::endl;

    pcl::PointCloud<pcl::PointWithRange> cloud1;
    pcl::PointCloud<pcl::PointWithRange> cloud2;
    pcl::PointCloud<pcl::PointWithRange> cloud3;
    pcl::PointCloud<pcl::PointWithRange> cloud4;
    pcl::PointCloud<pcl::PointWithRange> cloud5;

    filter_rg.setFilterLimits(0*step, 1*step); 
    filter_rg.filter(cloud1);
    filter_rg.setFilterLimits(1*step, 2*step); 
    filter_rg.filter(cloud2);
    filter_rg.setFilterLimits(2*step, 3*step); 
    filter_rg.filter(cloud3);
    filter_rg.setFilterLimits(3*step, 4*step); 
    filter_rg.filter(cloud4);
    filter_rg.setFilterLimits(4*step, 5*step); 
    filter_rg.filter(cloud5);

    sensor_msgs::PointCloud2 cloud1_msg;
    sensor_msgs::PointCloud2 cloud2_msg;
    sensor_msgs::PointCloud2 cloud3_msg;
    sensor_msgs::PointCloud2 cloud4_msg;
    sensor_msgs::PointCloud2 cloud5_msg;

    pcl::toROSMsg(cloud1, cloud1_msg);
    pcl::toROSMsg(cloud2, cloud2_msg);
    pcl::toROSMsg(cloud3, cloud3_msg);
    pcl::toROSMsg(cloud4, cloud4_msg);
    pcl::toROSMsg(cloud5, cloud5_msg);

    cloud1_msg.header.frame_id = "map";
    cloud2_msg.header.frame_id = "map";
    cloud3_msg.header.frame_id = "map";
    cloud4_msg.header.frame_id = "map";
    cloud5_msg.header.frame_id = "map";

    pub_range1.publish(cloud1_msg);
    pub_range2.publish(cloud2_msg);
    pub_range3.publish(cloud3_msg);
    pub_range4.publish(cloud4_msg);
    pub_range5.publish(cloud5_msg);
    // ===================================================


    // ================ KEEP NODE ALIVE ==================
    while (ros::ok()){ rate.sleep(); };
    // ===================================================


    return 0;
};
