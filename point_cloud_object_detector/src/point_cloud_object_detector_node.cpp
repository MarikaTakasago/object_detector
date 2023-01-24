#include "point_cloud_object_detector/point_cloud_object_detector.h"

int main(int argc,char** argv)
{
    std::cout<<"start node"<<std::endl;
	ros::init(argc,argv,"point_cloud_object_detector");
	PointCloudObjectDetector point_cloud_object_detector;
	point_cloud_object_detector.process();
	return 0;
}
