#include <string>


#include <ros/ros.h>

#include <tf2/LinearMath/Quaternion.h>

#include <pcl/search/kdtree.h>
#include <pcl/surface/mls.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>



#include <cmath>
tf2::Quaternion quat_between_vectors(tf2::Vector3 a, tf2::Vector3 b){
    // compute one quaternion solution for transformation between two vectors

    // define variable
    tf2::Quaternion q;
    tf2::Vector3 cross; 
    double dot;
    double x, y, z, w;

    // calculate
    cross = tf2::tf2Cross(a, b);
    dot = tf2::tf2Dot(a, b);
    x = cross.getX();
    y = cross.getY();
    z = cross.getZ();
    w = std::sqrt(std::pow(a.length(), 2) * std::pow(b.length(), 2) + dot);

    // store, normalize and returun
    q.setX(x); q.setY(y); q.setZ(z); q.setW(w);
    q.normalize();

    std::cout << q.getX() << q.getY() << q.getZ()<< q.getW() << std::endl;
    return q;
};



int main(int argc, char **argv){


    // =================== ARGUMENTS =====================
    std::string path_input = argv[1];
    float published_marker = std::stof(std::string(argv[2]));
    float ds_size = 0.3;
    // ===================================================



    // ===================== NODE ========================
    ros::init(argc, argv, "visualize_surfel");
    ros::NodeHandle nh;
    ros::Rate rate(10);
    // ===================================================



    // ================== PUBLISHERS =====================
    ros::Publisher pub_markers = nh.advertise<visualization_msgs::MarkerArray> ("surfel_marker", 100, true);
    ros::Publisher pub_pc_original = nh.advertise<sensor_msgs::PointCloud2> ("all", 100, true);
    ros::Publisher pub_pc_dsampled = nh.advertise<sensor_msgs::PointCloud2> ("ds", 100, true);
    ros::Publisher pub_surfel = nh.advertise<sensor_msgs::PointCloud2> ("surfel", 100, true);
    // ===================================================



    // =============== READ POINTCLOUD DATA ==============
    pcl::PointCloud<pcl::PointXYZ> cloud_xyz;
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud_xyz_ptr(&cloud_xyz);

    pcl::io::loadPCDFile<pcl::PointXYZ>(path_input, cloud_xyz);
    // ===================================================



    // ================== PUB ORIGINAL ===================
    // cloud_xyz to msg
    sensor_msgs::PointCloud2 msg_pc_original; 
    pcl::toROSMsg(cloud_xyz, msg_pc_original);
    msg_pc_original.header.frame_id = "map";

    // pub
    pub_pc_original.publish(msg_pc_original);
    // ===================================================



    // ================ PUB DOWNSAMPLED ==================
    // filter_vg
    pcl::VoxelGrid<pcl::PointXYZ> filter_vg;
    filter_vg.setInputCloud(cloud_xyz_ptr);
    filter_vg.setLeafSize(ds_size, ds_size, ds_size);
    filter_vg.filter(cloud_xyz);

    // cloud_xyz to msg
    sensor_msgs::PointCloud2 msg_pc_dsampled;
    pcl::toROSMsg(cloud_xyz, msg_pc_dsampled);
    msg_pc_dsampled.header.frame_id = "map";
    
    // pub
    pub_pc_dsampled.publish(msg_pc_dsampled);
    // ===================================================



    // ====================== SURFEL =====================
    pcl::PointCloud<pcl::PointSurfel> cloud_surfel;
    // ===================================================



    // =================== ADD NORMAL ====================
    // add normal and smoothing filter

    // define variable
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);

    // moving least square filter
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointSurfel> filter_mls;
    filter_mls.setInputCloud (cloud_xyz_ptr);
    filter_mls.setComputeNormals (true);
    filter_mls.setPolynomialOrder (1);
    filter_mls.setSearchMethod (tree);
    filter_mls.setSearchRadius (1);
    filter_mls.process(cloud_surfel);
    // ===================================================



    // =================== ADD RADIUS ====================
    // using fixed radius
    for (int p = 0; p < cloud_surfel.points.size(); ++p) {
        cloud_surfel.points[p].radius = ds_size/2;
        std::cout << cloud_surfel.points[p].normal_x << cloud_surfel.points[p].normal_y << cloud_surfel.points[p].normal_z << std::endl;
    };
    // ===================================================


    // publish surfel
    sensor_msgs::PointCloud2 msg_pc_surfel; 
    pcl::toROSMsg(cloud_surfel, msg_pc_surfel);
    msg_pc_surfel.header.frame_id = "map";

    // pub
    pub_surfel.publish(msg_pc_surfel);
    // ===================================================

    if (published_marker) {

        // ================ CONVERT TO MARKER ================
        // ! could combine with surfel generation, but seperate for compatability
        // TODO need to find if there is simpler way to display surfel
        
        // marker array
        visualization_msgs::MarkerArray marker_array;
        
        
        for (int p = 0; p < cloud_surfel.points.size(); ++p) {

            // surfel, normal, quaternion
            pcl::PointSurfel surfel = cloud_surfel.points[p];
            tf2::Vector3 n(surfel.normal_x, surfel.normal_y, surfel.normal_z);
            tf2::Quaternion q = quat_between_vectors(tf2::Vector3(0, 0, 1), n);
            
            // marker
            visualization_msgs::Marker marker;
            marker.type = visualization_msgs::Marker::CYLINDER;
            marker.header.frame_id = "map";
            marker.id = p;     

            marker.scale.x = surfel.radius;
            marker.scale.y = surfel.radius;       
            marker.scale.z = 0.01;

            marker.color.r = 1;
            marker.color.g = 1;
            marker.color.b = 1;
            marker.color.a = 1;

            marker.pose.position.x = surfel.x;
            marker.pose.position.y = surfel.y;
            marker.pose.position.z = surfel.z;
            
            marker.pose.orientation.x = q.getX();
            marker.pose.orientation.y = q.getY();
            marker.pose.orientation.z = q.getZ();
            marker.pose.orientation.w = q.getW();


            // store in marker array
            marker_array.markers.push_back(marker);
        };    
        // ===================================================



        // ================= PUBLISH ARRAY ===================
        pub_markers.publish(marker_array);
        std::cout << "finished" << std::endl;
        // ===================================================


    };

    // ================== KEEP ALIVE =====================
    while(ros::ok()){ rate.sleep(); };
    // ===================================================
};
