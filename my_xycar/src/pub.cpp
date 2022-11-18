#include "ros/ros.h"
#include "std_msgs/Int32.h"

int main(int argc, char **argv){
	ros::init(argc, argv, "teacher", ros::init_options::AnonymousName);
	ros::NodeHandle nh;
	ros::Publisher pub = nh.advertise<std_msgs::Int32>("call_sing", 1);
	ros::Rate rate(1);
	std_msgs::Int32 i;
	i.data=1;
	while(ros::ok()){
		pub.publish(i);
		i.data+=1;
		rate.sleep();
	}
	return 0;
}
