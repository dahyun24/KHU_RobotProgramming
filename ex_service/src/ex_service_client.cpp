#include"ros/ros.h"
#include"ex_service/myservice.h"
#include<cstdlib>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "ex_service_client");
	if (argc != 3){
		ROS_INFO("usage: ex_server_client num1 num2");
		return 1;
	}

	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<ex_service::myservice>("my_service");
	ex_service::myservice srv;
	srv.request.a = atoi(argv[1]);
	srv.request.b = atoi(argv[2]);

	if (client.call(srv)){
		ROS_INFO("Sum: %ld" , (long int)srv.response.result);
	}
	else{
		ROS_ERROR("Falied to call service my_service");
		return 1;
	}
	return 0;
}
