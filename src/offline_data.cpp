#include <iostream>
#include <fstream>
#include <string>
#include <sstream>

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/filters/voxel_grid.h>
#include <pcl_ros/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

#include <math.h>



sensor_msgs::PointCloud2 filter_voxelgrid(const sensor_msgs::PointCloud2 & msg_input, 
    float size_x, float size_y, float size_z){
    // downsample pointclouds in msg format using voxelgrid
    
    // define variables
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc_input    (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc_output   (new pcl::PointCloud<pcl::PointXYZ>);
    sensor_msgs::PointCloud2 msg_output;
    

    // ================ convert msg -> pc ================
    pcl::fromROSMsg(msg_input, *pc_input);
    // ===================================================
    

    // ================ Voxelgrid Filter =================
    pcl::VoxelGrid<pcl::PointXYZ> filter;
    filter.setInputCloud(pc_input);
    filter.setLeafSize(size_x, size_y, size_z);
    filter.filter(*pc_output);
    // ===================================================


    // ================ convert pc -> msg ================
    pcl::toROSMsg(*pc_output, msg_output);
    // ===================================================

    // return
    return msg_output;
};



sensor_msgs::PointCloud2 filter_range(const sensor_msgs::PointCloud2 & msg_input, 
    const float threshold){
    // filter point cloud is msgs format by range
    
    // define variables
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc_input    (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc_output   (new pcl::PointCloud<pcl::PointXYZ>);
    sensor_msgs::PointCloud2 msg_output;
    

    // ================ convert msg -> pc ================
    pcl::fromROSMsg(msg_input, *pc_input);
    // ===================================================


    // ================== Range Filter ===================
    pcl::ExtractIndices<pcl::PointXYZ> filter;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

    for (int p = 0; p < pc_input->points.size(); ++p) {        
        // calcualte distance
        float distance2 =   (pc_input->points[p].x * pc_input->points[p].x) +
                            (pc_input->points[p].y * pc_input->points[p].y) + 
                            (pc_input->points[p].z * pc_input->points[p].z);
        float distance = std::sqrt(distance2);
        // record points outside threshold
        if (distance > threshold){
            inliers->indices.push_back(p);
        };
    };
    filter.setInputCloud(pc_input);
    filter.setIndices(inliers);
    filter.filter(*pc_output);
    // ===================================================


    // ================ convert pc -> msg ================
    pcl::toROSMsg(*pc_output, msg_output);
    // ===================================================

    // return
    return msg_output;
};






int main(int argc, char **argv){
    // to run this node, add path to g2o file as argument
    // e.g. 
    // rosrun offline_data offline_data /home/ori/Desktop/vilens_slam_offline_data/2022-03-03-hilti-runs/exp02/exp02-01/slam_pose_graph.g2o


    // =================== ARGUMENTS =====================
    std::string path_posegraph = argv[1];
    float range_threshold = std::stof(std::string(argv[2]));
    float voxelgrid_size = std::stof(std::string(argv[3]));
    // ===================================================


    // ===================== NODE ========================
    ros::init(argc, argv, "offline_data");
    ros::NodeHandle nh;
    ros::Rate rate(10);
    // ===================================================



    // ================== PUBLISHERS =====================
    // PointCloud2 to /transformed
    ros::Publisher pub_transformed = nh.advertise<sensor_msgs::PointCloud2> ("transformed", 100);

    // PointCloud2 to /all_ds
    ros::Publisher pub_all_ds = nh.advertise<sensor_msgs::PointCloud2> ("all_ds", 100);

    // Pose to /pose
    ros::Publisher pub_pose = nh.advertise<geometry_msgs::PoseStamped> ("pose", 100);

    // Path to /path
    ros::Publisher pub_path = nh.advertise<nav_msgs::Path> ("path", 100);

    // TF to /tf2
    tf2_ros::TransformBroadcaster br;

    // ===================================================
    


    // ================== LISTENER ======================
    // transform listener should be initialized before the transformation is posted, 
    // otherwise the transform won't be registered
    // preferabiliy outside while loop, thus don't get reinitialized every time

    tf2_ros::Buffer tfBuffer; 
    tf2_ros::TransformListener tfListener(tfBuffer);
    // ===================================================

    

    // ============ READ POSEGRAPH FILE ==================
    
    // extract path to pose graph file & path to its folder
    std::string path_parent = path_posegraph.substr(0, path_posegraph.rfind("/"));

    // read posegraph file
    std::ifstream infile(path_posegraph);
    // ===================================================



    // define variable that will be updated every loop
    sensor_msgs::PointCloud2 pc_msg_all;
    nav_msgs::Path path; 
    std::string line; 

    while ( std::getline(infile, line) && ros::ok()){
        
        // ================ EXECUTE FOR EACH LINE ================
        // 
        // read the first word of each line to decide which 
        // code to run
        //

        // put line into string stream
        std::stringstream line_ss;
        line_ss << line;

        // read first word
        std::string type;
        line_ss >> type;
        
        // condition on the first word to decide which code to run
        if (type == "#"){ 
            
            // ================= READ COMMENT ====================
            //
            continue;
            //
            // ===================================================

        } else if (type == "EDGE_SE3:QUAT"){ 
            
            // ================== READ EDGES =====================
            //
            continue; 
            // 
            // ===================================================


        } else if (type == "VERTEX_SE3:QUAT_TIME"){ 

            // ================== READ NODE DATA =================
            // 
            // 1. read data from string stream 
            // 2. convert to transformation / pose / path
            // 3. publish 
            //      - transformation to /tf2 topic
            //      - path to /path
            // 

            // define variables
            int id;
            double x, y, z, qx, qy, qz, qw;
            uint32_t sec, nsec;
            std::string frame_local = "map";
            std::string frame_lidar = "lidar";

            // populate data
            line_ss >> id >> x >> y >> z >> qx >> qy >> qz >> qw >> sec >> nsec;
            ros::Time stamp; stamp.sec = sec; stamp.nsec = nsec;

            // stored in transform object
            geometry_msgs::TransformStamped transformStamped;
            transformStamped.header.stamp = stamp;
            transformStamped.header.frame_id = frame_local;
            transformStamped.child_frame_id = frame_lidar;
            transformStamped.transform.translation.x = x;
            transformStamped.transform.translation.y = y;
            transformStamped.transform.translation.z = z;
            transformStamped.transform.rotation.x = qx;
            transformStamped.transform.rotation.y = qy;
            transformStamped.transform.rotation.z = qz;
            transformStamped.transform.rotation.w = qw;

            // stored as pose object
            geometry_msgs::PoseStamped pose;
            pose.header.frame_id = frame_local;
            pose.header.stamp = stamp;
            pose.pose.position.x = x;
            pose.pose.position.y = y;
            pose.pose.position.z = z;
            pose.pose.orientation.x = qx;
            pose.pose.orientation.y = qy;
            pose.pose.orientation.z = qz;
            pose.pose.orientation.w = qw;

            // stored as path object
            path.header.frame_id = frame_local;
            path.header.stamp = stamp;
            path.poses.push_back(pose);
            
            // broadcast transform to /tf2
            br.sendTransform(transformStamped);

            // boradcast pose to /pose
            pub_pose.publish(pose);

            // broadcast path to /path
            pub_path.publish(path);

            // ===================================================



            // =============== READ POINTCLOUD DATA ==============
            // 
            // 1. read pcd data from local file
            // 2. convert pcd file to msg format (used later for transformation)            
            // 

            // define variable
            pcl::PointCloud<pcl::PointXYZ> cloud;
            std::stringstream path_sub;
            std::string path_pcd;

            // compute path for pcd file
            path_sub << "/individual_clouds/cloud_" << sec << "_" << std::setfill('0') << std::setw(9) << nsec << ".pcd";
            path_pcd = path_parent + path_sub.str();

            // read from file
            if (pcl::io::loadPCDFile<pcl::PointXYZ>(path_pcd, cloud) < 0){
                // file not found, do next while loop
                continue; 
            };
    
            // convert cloud to msg format
            sensor_msgs::PointCloud2 pc_msg;
            pcl::toROSMsg(cloud, pc_msg);

            // added stamp and frame_id
            pc_msg.header.frame_id = frame_lidar;
            pc_msg.header.stamp = stamp;
            // ===================================================



            // ================= FILTER BY RANGE =================
            pc_msg = filter_range(pc_msg, range_threshold);
            // ===================================================



            // ============ TRANSFORM TO LOCAL FRAME =============
            // 
            // 1. wait for transformation to local frame to be available
            // 2. transform point cloud
            // 3. upload to /transformed topic 
            //

            // define variable
            sensor_msgs::PointCloud2 pc_msg_transformed;

            // wait for transformation to be available
            tfBuffer.lookupTransform(frame_local, frame_lidar, stamp, ros::Duration(1));

            // transform point cloud to local frame
            pcl_ros::transformPointCloud(frame_local, pc_msg, pc_msg_transformed, tfBuffer);
            
            // upload to /transformed topic (to be visualized in rviz)
            pub_transformed.publish(pc_msg_transformed);
            // ===================================================



            // ================ MERGE POINT CLOUD ================
            // 
            // 1. merge transformed point cloud with previous point clouds
            // 
            sensor_msgs::PointCloud2 pc_msg_merged;
            pcl::concatenatePointCloud(pc_msg_transformed, pc_msg_all, pc_msg_merged);
            // ===================================================



            // ============= DOWNSAMPLE POINT CLOUD ==============            
            pc_msg_all = filter_voxelgrid(pc_msg_merged, voxelgrid_size, voxelgrid_size, voxelgrid_size);
            // ===================================================



            // =============== PUBLISH TO /all_ds ================
            pub_all_ds.publish(pc_msg_all); // publish to /all_ds topic
            // ===================================================

            

        };
        

    rate.sleep();
        
    };

    // ===================================================
    
};

