#ifndef POINT_CLOUD_OBJECT_DETECTOR_H_
#define POINT_CLOUD_OBJECT_DETECTOR_H_

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <geometry_msgs/Point32.h>
#include <pcl/point_types_conversion.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/filter.h>
#include <pcl_ros/point_cloud.h>
#include <std_msgs/Header.h>

#include <string>
#include <vector>

#include "darknet_ros_msgs/BoundingBox.h"
#include "darknet_ros_msgs/BoundingBoxes.h"
#include "object_detector_msgs/ObjectPosition.h"
#include "object_detector_msgs/ObjectPositions.h"

class PointCloudObjectDetector {
public:
    PointCloudObjectDetector();
    void process();

private:
    void pc_callback(const sensor_msgs::PointCloud2ConstPtr& msg);
    void bbox_callback(const darknet_ros_msgs::BoundingBoxesConstPtr& msg);

    //add
    void detect_target_cluster(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &arranged_pc,const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &target_pc);
    void euclidean_clustering(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &pc,std::vector<pcl::PointIndices> &output);
    void get_target_cluster(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &pc,std::vector<pcl::PointIndices> &pc_indices,const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &output);
    void check_roomba_color(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &target);
    void color_checker();
    void color_counter(double h,double s,double v);
    void color_counter_reset();
    void reduce_points(int n,const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pc);
    //end_add

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud {new pcl::PointCloud<pcl::PointXYZRGB>};

    std::string pc_topic_name;
    std::string bbox_topic_name;
    std::string obj_topic_name;
    std::string obj_frame_name;
    std::string raw_pc;
    std::string arranged_pc;
    std::string target_pc;

    bool has_received_pc;
    //add
    double tolerance;
    int min_cluster_size;
    int max_cluster_size;
    bool roomba_dist_checker;
    int target_roomba_num;

    //colors
    int color_cluster_th;

    double green;
    double h_green_lower;
    double h_green_upper;
    double s_green_lower;
    double s_green_upper;
    double v_green_lower;
    double v_green_upper;

    double yellow;
    double h_yellow_lower;
    double h_yellow_upper;
    double s_yellow_lower;
    double s_yellow_upper;
    double v_yellow_lower;
    double v_yellow_upper;

    double blue;
    double h_blue_lower;
    double h_blue_upper;
    double s_blue_lower;
    double s_blue_upper;
    double v_blue_lower;
    double v_blue_upper;

    double orange;
    double h_orange_lower;
    double h_orange_upper;
    double s_orange_lower;
    double s_orange_upper;
    double v_orange_lower;
    double v_orange_upper;

    double purple;
    double h_purple_lower;
    double h_purple_upper;
    double s_purple_lower;
    double s_purple_upper;
    double v_purple_lower;
    double v_purple_upper;

    double red;
    double h_red_lower;
    double h_red_upper;
    double s_red_lower;
    double s_red_upper;
    double v_red_lower;
    double v_red_upper;

    double other_color;
    //

    sensor_msgs::PointCloud2::Ptr ros_pc;

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber pc_sub_;
    ros::Subscriber bbox_sub_;
    ros::Publisher obj_pub_;
    ros::Publisher check_pub_1;
    ros::Publisher check_pub_2;
    ros::Publisher check_pub_3;
};

#endif  // POINT_CLOUD_OBJECT_DETECTOR_H_
