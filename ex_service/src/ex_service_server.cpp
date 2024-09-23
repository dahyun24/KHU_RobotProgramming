#include "ros/ros.h"
#include "ex_service/myservice.h"

bool server_callback(ex_service::myservice::Request &req, ex_service::myservice::Response &res)
{
	res.result = req.a + req.b;
	ROS_INFO("request: x=%ld. y=%ld", (long int)req.a, (long int)req.b);
	ROS_INFO("Sending back response: [%ld]", (long int)res.result);
	return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "ex_service_server");
	ros::NodeHandle n;
	
	ros::ServiceServer service = n.advertiseService("my_service", server_callback);
	ROS_INFO("my_service: ready");
	ros::spin();

	return 0;
}

