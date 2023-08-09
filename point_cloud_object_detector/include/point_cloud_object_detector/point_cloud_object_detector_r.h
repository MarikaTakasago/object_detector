#ifndef POINT_CLOUD_OBJECT_DETECTOR_H_
#define POINT_CLOUD_OBJECT_DETECTOR_H_

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/CompressedImage.h>
#include <geometry_msgs/Point32.h>
#include <pcl/point_types_conversion.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/filter.h>
#include <pcl_ros/point_cloud.h>
#include <std_msgs/Header.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>

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
    struct pixnum{
        int x;
        int y;
    };

    void pc_callback(const sensor_msgs::PointCloud2ConstPtr& msg);
    void bbox_callback(const darknet_ros_msgs::BoundingBoxesConstPtr& msg);
    void image_callback(const sensor_msgs::ImageConstPtr& msg);

    //add
    void detect_target_cluster(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &arranged_pc,const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &target_pc);
    void euclidean_clustering(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &pc,std::vector<pcl::PointIndices> &output);
    void get_target_cluster(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &pc,std::vector<pcl::PointIndices> &pc_indices,const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &output);
    void check_roomba_color(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &target);
    void color_checker(int th); //for no clustering ver
    // void color_checker(); //for clustering ver
    void check_colored_parts(cv::Mat image,double xmin,double ymin,double xmax,double ymax);
    void color_counter(double h,double s,double v);
    void make_color_array(int h,int s,int v,int i,int j);
    // void make_color_array(double h,double s,double v,int i,int j);
    void color_counter_reset();
    void check_pixel(int i,int j);
    void reduce_points(int n,const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pc);
    //end_add

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ {new pcl::PointCloud<pcl::PointXYZRGB>};

    std::string pc_topic_name_;
    std::string bbox_topic_name_;
    std::string img_topic_name_;
    std::string obj_topic_name_;
    std::string obj_frame_name_;
    std::string raw_pc_name_;
    std::string arranged_pc_;
    std::string target_pc_;
    std::string param_file_name_;

    bool has_received_pc_;
    bool get_image_;
    //add
    double tolerance_;
    int min_cluster_size_;
    int max_cluster_size_;
    bool roomba_dist_checker_;
    bool is_roomba_pixel_;
    int target_roomba_num_;

    cv::Mat image_;
    cv::Mat hsv_image_;
    int image_width_;
    int image_height_;

    //colors
    int color_cluster_th_;
    int img_th_;

    std::vector<pixnum> target_color_array_;

    int green_;
    int h_green_lower_;
    int h_green_upper_;
    int s_green_lower_;
    int s_green_upper_;
    int v_green_lower_;
    int v_green_upper_;
    std::vector<pixnum> green_array_;

    int yellow_;
    int h_yellow_lower_;
    int h_yellow_upper_;
    int s_yellow_lower_;
    int s_yellow_upper_;
    int v_yellow_lower_;
    int v_yellow_upper_;
    std::vector<pixnum> yellow_array_;

    int blue_;
    int h_blue_lower_;
    int h_blue_upper_;
    int s_blue_lower_;
    int s_blue_upper_;
    int v_blue_lower_;
    int v_blue_upper_;
    std::vector<pixnum> blue_array_;

    int orange_;
    int h_orange_lower_;
    int h_orange_upper_;
    int s_orange_lower_;
    int s_orange_upper_;
    int v_orange_lower_;
    int v_orange_upper_;
    std::vector<pixnum> orange_array_;

    int purple_;
    int h_purple_lower_;
    int h_purple_upper_;
    int s_purple_lower_;
    int s_purple_upper_;
    int v_purple_lower_;
    int v_purple_upper_;
    std::vector<pixnum> purple_array_;

    int red_;
    int h_red_lower_;
    int h_red_upper_;
    int s_red_lower_;
    int s_red_upper_;
    int v_red_lower_;
    int v_red_upper_;
    std::vector<pixnum> red_array_;

    int other_color_;
    //

    sensor_msgs::PointCloud2::Ptr ros_pc_;

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber pc_sub_;
    ros::Subscriber bbox_sub_;
    ros::Subscriber img_sub_;
    ros::Publisher obj_pub_;
    ros::Publisher check_pub_1_;
    ros::Publisher check_pub_2_;
    ros::Publisher check_pub_3_;
    ros::Publisher image_pub_;
};

#endif  // POINT_CLOUD_OBJECT_DETECTOR_H_
