#include "point_cloud_object_detector/point_cloud_object_detector_r.h"

PointCloudObjectDetector::PointCloudObjectDetector() : private_nh_("~"), has_received_pc_(false)
{
    private_nh_.param("pc_topic_name",pc_topic_name_,{"/camera/depth_registered/points"});
    private_nh_.param("bbox_topic_name",bbox_topic_name_,{"/darknet_ros/bounding_boxes"});
    private_nh_.param("obj_topic_name",obj_topic_name_,{"/object_positions"});
    private_nh_.param("obj_frame_name",obj_frame_name_,{"base_link"});
    private_nh_.param("img_topic_name",img_topic_name_,{"/camera/image_rect_color/compressed"});
    private_nh_.param("param_file_name",param_file_name_,{"/home/amsl/catkin_ws/src/point_cloud_object_detector/config/color_param.yaml"});

    private_nh_.getParam("tolerance",tolerance_);
    private_nh_.getParam("min_cluster_size",min_cluster_size_);
    private_nh_.getParam("max_cluster_size",max_cluster_size_);
    private_nh_.getParam("color_cluster_th",color_cluster_th_);
    private_nh_.getParam("img_th",img_th_);
    private_nh_.param("pc_topic_name",pc_topic_name_,{"/camera/depth_registered/points"});
    // private_nh_.param("bbox_topic_name",bbox_topic_name_,{"/roomba6/darknet_ros/bounding_boxes"});
    // private_nh_.param("obj_topic_name",obj_topic_name_,{"/roomba6/object_positions"});
    // private_nh_.param("obj_frame_name",obj_frame_name_,{"base_link"});
    // private_nh_.param("img_topic_name",img_topic_name_,{"/roomba6/camera/color/image_rect_color/compressed"});
    // private_nh_.param("raw_pc",raw_pc_name_,{"check_cloud"});
    // private_nh_.param("arranged_pc",arranged_pc_,{"check_cloud2"});
    // private_nh_.param("target_pc",target_pc_,{"check_cloud3"});

    private_nh_.getParam("h_green_lower",h_green_lower_);
    private_nh_.getParam("h_green_upper",h_green_upper_);
    private_nh_.getParam("s_green_lower",s_green_lower_);
    private_nh_.getParam("s_green_upper",s_green_upper_);
    private_nh_.getParam("v_green_lower",v_green_lower_);
    private_nh_.getParam("v_green_upper",v_green_upper_);
    private_nh_.getParam("h_yellow_lower",h_yellow_lower_);
    private_nh_.getParam("h_yellow_upper",h_yellow_upper_);
    private_nh_.getParam("s_yellow_lower",s_yellow_lower_);
    private_nh_.getParam("s_yellow_upper",s_yellow_upper_);
    private_nh_.getParam("v_yellow_lower",v_yellow_lower_);
    private_nh_.getParam("v_yellow_upper",v_yellow_upper_);
    private_nh_.getParam("h_blue_lower",h_blue_lower_);
    private_nh_.getParam("h_blue_upper",h_blue_upper_);
    private_nh_.getParam("s_blue_lower",s_blue_lower_);
    private_nh_.getParam("s_blue_upper",s_blue_upper_);
    private_nh_.getParam("v_blue_lower",v_blue_lower_);
    private_nh_.getParam("v_blue_upper",v_blue_upper_);
    private_nh_.getParam("h_orange_lower",h_orange_lower_);
    private_nh_.getParam("h_orange_upper",h_orange_upper_);
    private_nh_.getParam("s_orange_lower",s_orange_lower_);
    private_nh_.getParam("s_orange_upper",s_orange_upper_);
    private_nh_.getParam("v_orange_lower",v_orange_lower_);
    private_nh_.getParam("v_orange_upper",v_orange_upper_);
    private_nh_.getParam("h_purple_lower",h_purple_lower_);
    private_nh_.getParam("h_purple_upper",h_purple_upper_);
    private_nh_.getParam("s_purple_lower",s_purple_lower_);
    private_nh_.getParam("s_purple_upper",s_purple_upper_);
    private_nh_.getParam("v_purple_lower",v_purple_lower_);
    private_nh_.getParam("v_purple_upper",v_purple_upper_);
    private_nh_.getParam("h_red_lower",h_red_lower_);
    private_nh_.getParam("h_red_upper",h_red_upper_);
    private_nh_.getParam("s_red_lower",s_red_lower_);
    private_nh_.getParam("s_red_upper",s_red_upper_);
    private_nh_.getParam("v_red_lower",v_red_lower_);
    private_nh_.getParam("v_red_upper",v_red_upper_);


    pc_sub_ = nh_.subscribe(pc_topic_name_,1,&PointCloudObjectDetector::pc_callback,this);
    bbox_sub_ = nh_.subscribe(bbox_topic_name_,1,&PointCloudObjectDetector::bbox_callback,this);
    img_sub_ = nh_.subscribe(img_topic_name_,1,&PointCloudObjectDetector::image_callback,this);

    obj_pub_ = nh_.advertise<object_detector_msgs::ObjectPositions>(obj_topic_name_,1);
    // check_pub_1_ = nh_.advertise<sensor_msgs::PointCloud2>(raw_pc_name_,10);
    // check_pub_2_ = nh_.advertise<sensor_msgs::PointCloud2>(arranged_pc_,10);
    // check_pub_3_ = nh_.advertise<sensor_msgs::PointCloud2>(target_pc_,10);

    has_received_pc_ = false;
    get_image_ = false;

    ROS_INFO("PointCloudObjectDetector initialized");
    std::cout << "h_green_lower_ : " << h_green_lower_ << std::endl;

}

void PointCloudObjectDetector::pc_callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    cloud_->points.clear();
    pcl::fromROSMsg(*msg,*cloud_);
    has_received_pc_ = true;
    // ROS_INFO("Received point cloud");
    // std::cout<<"cloud size:"<<cloud->points.size()<<std::endl;
}

void PointCloudObjectDetector::image_callback(const sensor_msgs::ImageConstPtr& msg)
{
    //to cv mat
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s",e.what());
        return;
    }
    image_ = cv_ptr->image;

    //to hsv image
    // cv::cvtColor(rgb_image_,hsv_image_,cv::COLOR_RGB2HSV);
    cv::cvtColor(image_,hsv_image_,cv::COLOR_BGR2HSV);
    //check hsv image
    // cv::imshow("hsv_image",hsv_image_);
    // cv::waitKey(1);
    // get image size
    image_width_ = image_.cols;
    image_height_ = image_.rows;
    get_image_ = true;
}

void PointCloudObjectDetector::bbox_callback(const darknet_ros_msgs::BoundingBoxesConstPtr& msg)
{
    // ROS_INFO("Received bounding boxes");
    if(has_received_pc_ && get_image_)
    {
        cloud_->header.frame_id = "base_link";
        // check_pub_1_.publish(cloud_);
        object_detector_msgs::ObjectPositions positions;
        for(const auto &b : msg->bounding_boxes)
        {
            // std::cout << "Object_Class: " << b.Class << std::endl;
            // std::vector<pcl::PointXYZRGB> points;
            std::vector<pcl::PointCloud<pcl::PointXYZRGB>> rearranged_points(cloud_->height,pcl::PointCloud<pcl::PointXYZRGB>());
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr values(new pcl::PointCloud<pcl::PointXYZRGB>);
            object_detector_msgs::ObjectPosition position;

            // for(const auto &p : cloud->points) points.push_back(p);

            if(cloud_->points.size() == cloud_->width*cloud_->height)
            {
                for(int i = 0; i < cloud_->height; i++)
                {
                    for(int j = 0; j < cloud_->width; j++)
                    {
                        rearranged_points.at(i).push_back(cloud_->points.at(i*cloud_->width+j));
                    }
                }

                if(!(b.xmin == 0 && b.xmax == 0))
                {
                    if(b.Class == "roomba")
                    {
                        ROS_INFO("roomba detected");
                        printf("b.min x:%d b.max x:%d b.min y:%d b.max y:%d, image width:%d image height:%d\n",b.xmin,b.xmax,b.ymin,b.ymax,image_width_,image_height_);
                        check_colored_parts(hsv_image_,b.xmin,b.xmax,b.ymin,b.ymax);
                        for(int i = b.ymin; i < b.ymax; i++)
                        {
                            for(int j = b.xmin; j < b.xmax; j++)
                            {
                                check_pixel(j,i);
                                if(is_roomba_pixel_)
                                {
                                    values->push_back(rearranged_points.at(i).at(j));
                                    is_roomba_pixel_ = false;
                                }
                            }
                        }

                        std::cout<<"ninjin"<<values->points.size()<<std::endl;
                        values->header.frame_id = "base_link";
                        // check_pub_3_.publish(values);
                    }

                    if(b.Class != "roomba")
                    {
                        for(int x = b.xmin; x < b.xmax; x++)
                        {
                            for(int y = b.ymin; y < b.ymax; y++)
                            {
                                values->points.push_back(rearranged_points.at(y).at(x));
                            }
                        }
                    }

                    double sum_x = 0.0;
                    double sum_y = 0.0;
                    double sum_z = 0.0;
                    int finite_count = 0;
                    for(const auto &value : values->points){
                        if(isfinite(value.x) && isfinite(value.y) && isfinite(value.z)){
                            sum_x += value.x;
                            sum_y += value.y;
                            sum_z += value.z;
                            finite_count ++;
                        }
                    }

                    if(sum_x*sum_y*sum_z == 0) continue;

                    positions.header.frame_id = obj_frame_name_;
                    positions.header.stamp = ros::Time::now();
                    position.Class = b.Class;
                    position.roomba_num = target_roomba_num_; //roombaじゃないときは0
                    position.probability = b.probability;
                    position.x = sum_x/(double)finite_count;
                    position.y = sum_y/(double)finite_count;
                    position.z = sum_z/(double)finite_count;

                    double d = sqrt(pow(position.x,2)+pow(position.z,2));
                    double theta = atan2(position.z,position.x) - M_PI/2;

                    position.theta = theta;
                    position.d = d;

                    // std::cout << "(X,Y,Z): " << "(" << position.x << "," << position.y << "," << position.z << ")" << std::endl;
                    // std::cout << "distance[m]: : " << d << std::endl;
                    // std::cout << "theta[rad] : " << theta << std::endl;
                    std::cout << std::endl;
                }
            }
            positions.object_position.push_back(position);
        }
        obj_pub_.publish(positions);
    }
}


void PointCloudObjectDetector::check_colored_parts(cv::Mat input_img, double xmin, double xmax, double ymin, double ymax)
{
    ROS_INFO("check_colored_parts");
    color_counter_reset();
    ROS_INFO("color_counter_reset");
    // cv::Mat hsv_image;
    // cv::cvtColor(input_img,hsv_image,cv::COLOR_RGB2HSV);

    double sum_h=0;
    double sum_s=0;
    double sum_v=0;
    double sum=0;
    int x_min = int(xmin);
    int x_max = int(xmax);
    int y_min = int(ymin);
    int y_max = int(ymax);
    int dy = y_max - y_min;

    // check img size
    // printf("hsv_image width:%d height:%d\n",input_img.cols,input_img.rows);
    // printf("xmin:%d xmax:%d ymin:%d ymax:%d\n",x_min,x_max,y_min,y_max);

    if(input_img.rows == 0 || input_img.cols == 0) return;

    for(int i = x_min; i < x_max; i++)
    {
        // for(int j = y_min; j < y_max; j++)
        for(int j = y_min; j < y_min + dy/2; j++)
        {
            int h = input_img.at<cv::Vec3b>(j,i)[0];
            int s = input_img.at<cv::Vec3b>(j,i)[1];
            int v = input_img.at<cv::Vec3b>(j,i)[2];

            // cv::Vec3b hsv = input_img.at<cv::Vec3b>(i,j);
            // double h = hsv[0];
            // double s = hsv[1];
            // double v = hsv[2];
            make_color_array(h,s,v,i,j);

            // std::cout<<"h,s,v::"<<h<<","<<s<<","<<v<<std::endl;

            sum_h += h;
            sum_s += s;
            sum_v += v;
            sum += 1;
        }
    }
    std::cout<<"sum"<<sum<<std::endl;
    std::cout<<"other_color"<<other_color_<<std::endl;
    std::cout<<"ave_h,s,v::"<<sum_h/sum<<","<<sum_s/sum<<","<<sum_v/sum<<std::endl;
    std::cout<<"========================================"<<std::endl;
    color_checker(img_th_);
}

void PointCloudObjectDetector::color_checker(int th)
{

    std::cout<<"green:"<<green_<<" yellow:"<<yellow_<<" blue:"<<blue_<<" orange:"<<orange_<<" purple:"<<purple_<<" red:"<<red_<<" th"<<th<<std::endl;
    int c = 0;
    if(green_ > th)
    {
        target_roomba_num_ = 1;
        // target_roomba_num_ = 3;
        c = green_;
        target_color_array_ = green_array_;
    }
    if(yellow_ > th && yellow_ > c)
    {
        target_roomba_num_ = 2;
        c = yellow_;
        target_color_array_ = yellow_array_;
    }
    if(blue_ > th && blue_ > c)
    {
        target_roomba_num_ = 3;
        c = blue_;
        target_color_array_ = blue_array_;
    }
    if(orange_ > th && orange_ > c)
    {
        target_roomba_num_ = 4;
        // target_roomba_num_ = 2;
        c = orange_;
        target_color_array_ = orange_array_;
    }
    if(purple_ > th && purple_ > c)
    {
        target_roomba_num_ = 5;
        // target_roomba_num_ = 1;
        c = purple_;
        target_color_array_ = purple_array_;
    }
    if(red_ > th && red_ > c)
    {
        target_roomba_num_ = 6;
        c = red_;
        target_color_array_ = red_array_;
    }
    // else target_roomba_num_ = 0;

    // std::cout<<"yellow:"<<yellow_<<std::endl;
    std::cout<<"green:"<<green_<<" yellow:"<<yellow_<<" blue:"<<blue_<<" orange:"<<orange_<<" purple:"<<purple_<<" red:"<<red_<<" c"<<c<<std::endl;
    std::cout<<"target_roomba:roomba"<<target_roomba_num_<<std::endl;
}

// void PointCloudObjectDetector::make_color_array(double h,double s,double v,int i,int j)
void PointCloudObjectDetector::make_color_array(int h,int s,int v,int i,int j)
{
    pixnum current_pixel_ = {i,j};
    // printf("g,y,b,o,p,r::%d,%d,%d,%d,%d,%d\n",green_,yellow_,blue_,orange_,purple_,red_);
    if((h_green_upper_ > h && h_green_lower_ < h) && (s_green_upper_ > s && s_green_lower_ < s) && (v_green_upper_ > v && v_green_lower_ < v))
    {
        green_array_.push_back(current_pixel_);
        green_ ++;
    }
    else if((h_yellow_upper_ > h && h_yellow_lower_ < h) && (s_yellow_upper_ > s && s_yellow_lower_ < s) && (v_yellow_upper_ > v && v_yellow_lower_ < v))
    {
        yellow_array_.push_back(current_pixel_);
        yellow_ ++;
    }
    else if((h_blue_upper_ > h && h_blue_lower_ < h) && (s_blue_upper_ > s && s_blue_lower_ < s) && (v_blue_upper_ > v && v_blue_lower_ < v))
    {
        blue_array_.push_back(current_pixel_);
        blue_ ++;
    }
    else if((h_orange_upper_ > h && h_orange_lower_ < h) && (s_orange_upper_ > s && s_orange_lower_ < s) && (v_orange_upper_ > v && v_orange_lower_ < v))
    {
        orange_array_.push_back(current_pixel_);
        orange_ ++;
    }
    else if((h_purple_upper_ > h && h_purple_lower_ < h) && (s_purple_upper_ > s && s_purple_lower_ < s) && (v_purple_upper_ > v && v_purple_lower_ < v))
    {
        purple_array_.push_back(current_pixel_);
        purple_ ++;
    }
    else if((h_red_upper_ > h && h_red_lower_ < h) && (s_red_upper_ > s && s_red_lower_ < s) && (v_red_upper_ > v && v_red_lower_ < v))
    {
        red_array_.push_back(current_pixel_);
        red_ ++;
    }
    else other_color_ ++;
}

// void PointCloudObjectDetector::color_counter(double h,double s,double v)
// {
//         if((h_green_upper_ > h && h_green_lower_ < h) && (s_green_upper_ > s && s_green_lower_ < s) && (v_green_upper_ > v && v_green_lower_ < v)) green_ += 1;
//         else if((h_yellow_upper_ > h && h_yellow_lower_ < h) && (s_yellow_upper_ > s && s_yellow_lower_ < s) && (v_yellow_upper_ > v && v_yellow_lower_ < v)) yellow_ += 1;
//         else if((h_blue_upper_ > h && h_blue_lower_ < h) && (s_blue_upper_ > s && s_blue_lower_ < s) && (v_blue_upper_ > v && v_blue_lower_ < v)) blue_ += 1;
//         else if((h_orange_upper_ > h && h_orange_lower_ < h) && (s_orange_upper_ > s && s_orange_lower_ < s) && (v_orange_upper_ > v && v_orange_lower_ < v)) orange_ += 1;
//         else if((h_purple_upper_ > h && h_purple_lower_ < h) && (s_purple_upper_ > s && s_purple_lower_ < s) && (v_purple_upper_ > v && v_purple_lower_ < v)) purple_ += 1;
//         else if((h_red_upper_ > h && h_red_lower_ < h) && (s_red_upper_ > s && s_red_lower_ < s) && (v_red_upper_ > v && v_red_lower_ < v)) red_ += 1;
//         else other_color_ += 1;
// }

void PointCloudObjectDetector::color_counter_reset()
{
    green_ = yellow_ = blue_ = orange_ = purple_ = red_ = 0;
    other_color_ = 0;
    target_roomba_num_ = 0;
    //reset vectors
    green_array_.clear();
    yellow_array_.clear();
    blue_array_.clear();
    orange_array_.clear();
    purple_array_.clear();
    red_array_.clear();
    target_color_array_.clear();
}

void PointCloudObjectDetector::check_pixel(int i,int j)
{
    pixnum pix;
    pix.x = i;
    pix.y = j;
    for(int k = 0;k < target_color_array_.size();k++)
    {
        if(pix.x == target_color_array_[k].x && pix.y == target_color_array_[k].y)
        {
            target_color_array_.erase(target_color_array_.begin() + k);
            is_roomba_pixel_ = true;
            break;
        }
        is_roomba_pixel_ = false;
    }
}
//
// void PointCloudObjectDetector::reduce_points(int n,const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pc)
// {
//     pcl::PointCloud<pcl::PointXYZRGB> new_pc;
//     new_pc.header = pc->header;
//     for(size_t i=0;i<pc->size();i+=n) new_pc.push_back(pc->at(i));
//     *pc = std::move(new_pc);
// }

void PointCloudObjectDetector::process() { ros::spin(); }
