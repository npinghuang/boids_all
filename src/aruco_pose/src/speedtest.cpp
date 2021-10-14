#include <ros/ros.h>
#include <iostream>
#include <aruco_pose/MarkerArray.h>
#include <aruco_pose/Marker.h>
#include <Eigen/Dense>
#include <fstream>
#include <sstream>
#include <string>

using namespace Eigen;  
using namespace std;  


void markersCallback(const aruco_pose::MarkerArray::ConstPtr& markers)
{
  
if(!markers->markers.empty()){ 


    	ROS_INFO("id:%ld",markers->markers[0].id );
	ROS_INFO("id:%ld",markers->markers[1].id );
	ROS_INFO("id:%ld",markers->markers[2].id );
	ROS_INFO("id:%ld",markers->markers[3].id );
	ROS_INFO("id:%ld",markers->markers[4].id );
	ROS_INFO("id:%ld",markers->markers[5].id );
	ROS_INFO("id:%ld",markers->markers[6].id );
	ROS_INFO("id:%ld",markers->markers[7].id );

	ROS_INFO("---------------------------------");
/***
    ROS_INFO("x:%f",markers->markers[0].pose.position.x );
    ROS_INFO("y:%f",markers->markers[0].pose.position.y );
    ROS_INFO("z:%f",markers->markers[0].pose.position.z );
***/
 
/***

	if(markers->markers[0].id == 70){
		ROS_INFO("id: %ld",markers->markers[0].id );			 
		ROS_INFO("x:%ld",markers->markers[0].pose.position.x);
		ROS_INFO("y:%ld",markers->markers[0].pose.position.y);
		ROS_INFO("z:%ld",markers->markers[0].pose.position.z);

	}
	else if(markers->markers[1].id == 70){
		ROS_INFO("id: %ld",markers->markers[0].id );			 
		ROS_INFO("x:%ld",markers->markers[0].pose.position.x);
		ROS_INFO("y:%ld",markers->markers[0].pose.position.y);
		ROS_INFO("z:%ld",markers->markers[0].pose.position.z);

	}
	else if(markers->markers[2].id == 70){
		ROS_INFO("id: %ld",markers->markers[0].id );			 
		ROS_INFO("x:%ld",markers->markers[0].pose.position.x);
		ROS_INFO("y:%ld",markers->markers[0].pose.position.y);
		ROS_INFO("z:%ld",markers->markers[0].pose.position.z);

	}
***/
}
  else{
    ROS_INFO("None");
  }

}



int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "listener");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/cam_R/aruco_detect/markers", 1000, markersCallback);

  ros::spin();

  return 0;
}

