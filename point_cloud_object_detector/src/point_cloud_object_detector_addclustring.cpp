#include "point_cloud_object_detector/point_cloud_object_detector.h"

PointCloudObjectDetector::PointCloudObjectDetector() : private_nh_("~"), has_received_pc(false)
{
    private_nh_.param("pc_topic_name",pc_topic_name,{"/camera/depth_registered/points"});
    private_nh_.param("bbox_topic_name",bbox_topic_name,{"/darknet_ros/bounding_boxes"});
    private_nh_.param("obj_topic_name",obj_topic_name,{"/object_positions"});
    private_nh_.param("obj_frame_name",obj_frame_name,{"base_link"});
    private_nh_.param("arranged_pc",arranged_pc,{"check_cloud"});
    private_nh_.param("raw_pc",raw_pc,{"check_cloud2"});
    private_nh_.getParam("tolerance",tolerance);
    // std::cout<<"to??"<<tolerance<<std::endl;
    private_nh_.getParam("min_cluster_size",min_cluster_size);
    private_nh_.getParam("max_cluster_size",max_cluster_size);
    private_nh_.getParam("low_target_y",low_target_y);
    private_nh_.getParam("high_target_y",high_target_y);

    pc_sub_ = nh_.subscribe(pc_topic_name,1,&PointCloudObjectDetector::pc_callback,this);
    bbox_sub_ = nh_.subscribe(bbox_topic_name,1,&PointCloudObjectDetector::bbox_callback,this);
    obj_pub_ = nh_.advertise<object_detector_msgs::ObjectPositions>(obj_topic_name,1);
    check_pub_1 = nh_.advertise<sensor_msgs::PointCloud2>(arranged_pc,10);
    check_pub_2 = nh_.advertise<sensor_msgs::PointCloud2>(raw_pc,10);
}

void PointCloudObjectDetector::pc_callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    cloud->points.clear();
    pcl::fromROSMsg(*msg,*cloud);
    has_received_pc = true;
    std::cout<<"cloud size:"<<cloud->points.size()<<std::endl;
}

void PointCloudObjectDetector::bbox_callback(const darknet_ros_msgs::BoundingBoxesConstPtr& msg)
{
    if(has_received_pc)
    {
        cloud->header.frame_id = "base_link";
        check_pub_1.publish(cloud);
        object_detector_msgs::ObjectPositions positions;
        for(const auto &b : msg->bounding_boxes)
        {
            std::cout << "Object_Class: " << b.Class << std::endl;
            std::vector<pcl::PointXYZRGB> points;
            std::vector<std::vector<pcl::PointXYZRGB>> rearranged_points(cloud->height,std::vector<pcl::PointXYZRGB>());
            std::vector<pcl::PointXYZRGB> values;
            std::vector<pcl::PointXYZRGB> vavalues;
            object_detector_msgs::ObjectPosition position;

            for(const auto &p : cloud->points) points.push_back(p);

            if(points.size() == cloud->width*cloud->height)
            {
                for(int i = 0; i < cloud->height; i++)
                {
                    for(int j = 0; j < cloud->width; j++)
                    {
                        rearranged_points.at(i).push_back(points.at(i*cloud->width+j));
                    }
                }



                // std::cout << rearranged_points.size() << std::endl;
                // std::cout << rearranged_points[0].size() << std::endl;

                if(!(b.xmin == 0 && b.xmax == 0))
                {
                    // for(int x = b.xmin; x < b.xmax; x++){
                    //     for(int y = b.ymin; y < b.ymax; y++){
                    //         values.push_back(rearranged_points.at(y).at(x));
                    //     }
                    // }
                    if(b.Class == "roomba")
                    {
                        for(int x = b.xmin; x < b.xmax; x++)
                        {
                            for(int y = b.ymin; y < b.ymax; y++)
                            {
                                vavalues.push_back(rearranged_points.at(y).at(x));
                            }
                        }
                        pcl::PointCloud<pcl::PointXYZRGB>::Ptr arranged_pc(new pcl::PointCloud<pcl::PointXYZRGB>);
                        arranged_pc->width = b.xmax - b.xmin;
                        arranged_pc->height = b.ymax - b.ymin;
                        // arranged_pc->is_dense = cloud->is_dense;
                        arranged_pc->points.resize(arranged_pc->width * arranged_pc->height);

                        pcl::PointCloud<pcl::PointXYZRGB>::Ptr target_pc(new pcl::PointCloud<pcl::PointXYZRGB>);
                        target_pc->width = arranged_pc->width;
                        target_pc->height = arranged_pc->height;
                        // target_pc->is_dense = cloud->is_dense;
                        target_pc->points.resize(target_pc->width * target_pc->height);
                        std::cout<<"nyaa"<<std::endl;
                        int ar = 0;
                        for(const auto &v : vavalues)
                        {
                            if(!isnan(v.x)&&!isnan(v.y)&&!isnan(v.z))
                            {
                                arranged_pc->points.at(ar).x= v.x;
                                arranged_pc->points.at(ar).y= v.y;
                                arranged_pc->points.at(ar).z= v.z;
                                ar += 1;
                            }
                        }
                        // std::vector<int> mapping;
                        // pcl::removeNaNFromPointCloud(*arranged_pc,*arranged_pc,mapping);
                        std::cout<<"wauu" << arranged_pc->points.size() << std::endl;

                        //check arranged_pc
                        // arranged_pc->header = cloud->header;
                        // pcl::toROSMsg(*arranged_pc,*ros_pc);
                        // ros_pc->header = cloud->header;
                        //
                        // pub_pc
                        arranged_pc->header.frame_id = "base_link";
                        check_pub_2.publish(arranged_pc);

                        if(arranged_pc->points.size() > max_cluster_size)
                        {
                            int n = (arranged_pc->points.size() / max_cluster_size) + 1;
                            reduce_points(n,arranged_pc);
                            std::cout << "reduce:" << arranged_pc->points.size();
                        }

                        detect_target_cluster(arranged_pc,target_pc);
                        if(roomba_dist_checker) for(const auto &t :target_pc->points) values.push_back(t);
                        else if(!roomba_dist_checker) continue;
                        std::cout<<"ninjin"<<values.size()<<std::endl;
                        // std::cout<<"value.x:"<<values[20].x<<std::endl;
                    }

                    if(b.Class != "roomba")
                    {
                        for(int x = b.xmin; x < b.xmax; x++)
                        {
                            for(int y = b.ymin; y < b.ymax; y++)
                            {

                                values.push_back(rearranged_points.at(y).at(x));
                            }
                        }
                    }

                    double sum_x = 0.0;
                    double sum_y = 0.0;
                    double sum_z = 0.0;
                    int finite_count = 0;
                    if(b.Class == "roomba" && (!roomba_dist_checker)) continue;
                    for(const auto &value : values){
                        if(isfinite(value.x) && isfinite(value.y) && isfinite(value.z)){
                            sum_x += value.x;
                            sum_y += value.y;
                            sum_z += value.z;
                            finite_count ++;
                        }
                    }

                    if(sum_x*sum_y*sum_z == 0) continue;

                    positions.header.frame_id = obj_frame_name;
                    positions.header.stamp = ros::Time::now();
                    position.Class = b.Class;
                    position.probability = b.probability;
                    position.x = sum_x/(double)finite_count;
                    position.y = sum_y/(double)finite_count;
                    position.z = sum_z/(double)finite_count;

                    double d = sqrt(pow(position.x,2)+pow(position.z,2));
                    double theta = atan2(position.z,position.x) - M_PI/2;

                    position.theta = theta;
                    position.d = d;

                    std::cout << "(X,Y,Z): " << "(" << position.x << "," << position.y << "," << position.z << ")" << std::endl;
                    std::cout << "distance[m]: : " << d << std::endl;
                    std::cout << "theta[rad] : " << theta << std::endl;
                    std::cout << std::endl;
                }
            }
            positions.object_position.push_back(position);
        }
        obj_pub_.publish(positions);
    }
}

void PointCloudObjectDetector::detect_target_cluster(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &arranged_pc,const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &target_pc)
{
    // std::cout << "size_a" << arranged_pc->points.size() << std::endl;
    std::vector<pcl::PointIndices> pc_indices;
    euclidean_clustering(arranged_pc,pc_indices);
    get_target_cluster(arranged_pc,pc_indices,target_pc);
    std::cout << "sizet" << target_pc->points.size() << std::endl;
}

void PointCloudObjectDetector::euclidean_clustering(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &pc,std::vector<pcl::PointIndices> &output)
{

    if(pc->points.size() > max_cluster_size)
    {
        roomba_dist_checker = false;
        return;
    }
    // if(pc->points.size() > max_cluster_size)
    // {
    //     int n = (pc->points.size() / max_cluster_size) + 1;
    //     reduce_points(n,pc);
    // }
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud(pc);

    std::cout << "tole:" << tolerance <<std::endl;
    // check_pub_1.publish(pc);
    std::cout << "pc_size" << pc->points.size() << std::endl;
    std::cout << "size_a" << pc->points.size() << std::endl;
    std::vector<pcl::PointIndices> pc_indices;
    pcl::shared_ptr<pcl::EuclideanClusterExtraction<pcl::PointXYZRGB>> eu(new pcl::EuclideanClusterExtraction<pcl::PointXYZRGB>);
    eu->setClusterTolerance(tolerance);
    eu->setMinClusterSize(min_cluster_size);
    eu->setMaxClusterSize(max_cluster_size);
    std::cout<<"neko"<<std::endl;
    eu->setSearchMethod(tree);
    std::cout<<"tora"<<std::endl;
    eu->setInputCloud(pc);
    std::cout<<"tori"<<std::endl;
    eu->extract(pc_indices);
    std::cout<<"usaaa"<<std::endl;

    std::cout<<"indice"<<pc_indices.size()<<std::endl;
    output = std::move(pc_indices);
    if(output.size() != 0) roomba_dist_checker = true;
    std::cout<<"output"<<output.size()<<std::endl;

}

void PointCloudObjectDetector::get_target_cluster(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &pc,std::vector<pcl::PointIndices> &pc_indices,const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &output)
{
    for(const auto &point_indices:pc_indices)
    {
        std::cout<<"saru"<<std::endl;
        bool target_chcker = true;
        pcl::PointCloud<pcl::PointXYZRGB> cluster;

        for(const auto &itr : point_indices.indices)
        {
            cluster.push_back(pc->points.at(itr));
            // if(cluster.back().y < low_target_y || high_target_y < cluster.back().y)
            // {
            //     target_chcker = false;
            //     break;
            // }
        }
        if(target_chcker)
        {
            *output = std::move(cluster);
            // std::cout<<"cluster"<<cluster.points[0].x<<std::endl;
            return;
        }
    }
    return;
}

void PointCloudObjectDetector::reduce_points(int n,const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pc)
{
    pcl::PointCloud<pcl::PointXYZRGB> new_pc;
    new_pc.header = pc->header;
    for(size_t i=0;i<pc->size();i+=n) new_pc.push_back(pc->at(i));
    *pc = std::move(new_pc);
}

void PointCloudObjectDetector::process() { ros::spin(); }
