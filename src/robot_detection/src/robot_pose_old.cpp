#include <ros/ros.h>
#include <aruco_pose/Marker.h>
#include <aruco_pose/MarkerArray.h>
#include <geometry_msgs/Pose.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <math.h>

using namespace std;
using cv::Mat;
using std::vector;
// ros::Subscriber aruco;
ros::Subscriber aruco_array;
aruco_pose::Marker marker;
void aruco_markers(const aruco_pose::Marker::ConstPtr& msg){
  ROS_INFO("sub1");
  ROS_INFO("I heard marker: [%d]", msg->id);
}
void markersCallback(const aruco_pose::MarkerArray::ConstPtr &markers){
  ROS_INFO("sub");
  if (!markers->markers.empty()){
    ROS_INFO("I heard marker: [%d]", markers->markers[0].id);
  }
}
void aruco_markers(aruco_pose::Marker& marker, const vector<cv::Point2f>& corners){
  marker.c1.x = corners[0].x;
  marker.c2.x = corners[1].x;
  marker.c3.x = corners[2].x;
  marker.c4.x = corners[3].x;
  marker.c1.y = corners[0].y;
  marker.c2.y = corners[1].y;
  marker.c3.y = corners[2].y;
  marker.c4.y = corners[3].y;
}
void fillPose(geometry_msgs::Pose& pose, const cv::Vec3d& rvec, const cv::Vec3d& tvec){
        
      //         int degree = 60;
      //         pose.position.x = (tvec[2] * 0.1) * cos(degree*M_PI/180);
  // pose.position.y = -tvec[0] * 0.1;
  // pose.position.z = -(tvec[1] * 0.1) * cos(degree*M_PI/180);


  pose.position.x = tvec[0];
  pose.position.y = tvec[1];
  pose.position.z = tvec[2];

  double angle = norm(rvec);
  cv::Vec3d axis = rvec / angle;

  tf2::Quaternion q;
  q.setRotation(tf2::Vector3(axis[0], axis[1], axis[2]), angle);

  pose.orientation.w = q.w();
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
}
int main(int argc, char* argv[])
{
  // This must be called before anything else ROS-related
  ros::init(argc, argv, "robot_pose");

  // Create a ROS node handle
  ros::NodeHandle n;

  // aruco = n.subscribe("aruco_detect/markers", 1000,  aruco_markers);
  aruco_array= n.subscribe("aruco_detect/markers", 1000, markersCallback);
  ROS_INFO("Hello, World!");

  ros::Rate loop_rate(100);
  while (ros::ok()){
    ros::spinOnce();
    loop_rate.sleep();
  }
  // Don't exit the program.
  ros::spin();
}
        // switch(markers->markers[i].id % 5 ){
        //     case 0:{
        //         robot.id=markers->markers[i].id;
        //         robot.pose = markers->markers[i].pose;
        //         robots_array.robots.push_back(robot);
        //         // robot.id.clear();
        //         ROS_INFO("id [%d], x : [%f]", robot.id , robot.pose.position.x);
        //         double yaww;
        //         yaww = quaternion_to_theta(robot.pose.orientation.x, robot.pose.orientation.y, robot.pose.orientation.z, robot.pose.orientation.w);
        //         ROS_INFO("yaw0 : [%f]", yaww);
        //         break;  
        //     }
        //     case 1:{
        //         robot.id=markers->markers[i].id;
        //         robot.pose = markers->markers[i].pose;
        //         robots_array.robots.push_back(robot);
        //         // robot.id.clear();
        //         ROS_INFO("id [%d], x : [%f]", robot.id , robot.pose.position.x);
        //         double yaww;
        //         yaww = quaternion_to_theta(robot.pose.orientation.x, robot.pose.orientation.y, robot.pose.orientation.z, robot.pose.orientation.w);
        //         ROS_INFO("yaw1 : [%f]", yaww);
        //         break;
        //     }
        //     case 3:{
        //         robot.id=markers->markers[i].id;
        //         robot.pose = markers->markers[i].pose;
        //         robots_array.robots.push_back(robot);
        //         // robot.id.clear();
        //         ROS_INFO("id [%d], x : [%f]", robot.id , robot.pose.position.x);
        //         double yaww;
        //         yaww = quaternion_to_theta(robot.pose.orientation.x, robot.pose.orientation.y, robot.pose.orientation.z, robot.pose.orientation.w);
        //         ROS_INFO("yaw3 : [%f]", yaww);
        //         break;
        //     }
            
        // }

        // void aruco_markers(aruco_pose::Marker& marker, const vector<cv::Point2f>& corners){
//   marker.c1.x = corners[0].x;
//   marker.c2.x = corners[1].x;
//   marker.c3.x = corners[2].x;
//   marker.c4.x = corners[3].x;
//   marker.c1.y = corners[0].y;
//   marker.c2.y = corners[1].y;
//   marker.c3.y = corners[2].y;
//   marker.c4.y = corners[3].y;
// }
// void fillPose(geometry_msgs::Pose& pose, const cv::Vec3d& rvec, const cv::Vec3d& tvec){
        
//       //         int degree = 60;
//       //         pose.position.x = (tvec[2] * 0.1) * cos(degree*M_PI/180);
//   // pose.position.y = -tvec[0] * 0.1;
//   // pose.position.z = -(tvec[1] * 0.1) * cos(degree*M_PI/180);


//   pose.position.x = tvec[0];
//   pose.position.y = tvec[1];
//   pose.position.z = tvec[2];

//   double angle = norm(rvec);
//   cv::Vec3d axis = rvec / angle;

//   tf2::Quaternion q;
//   q.setRotation(tf2::Vector3(axis[0], axis[1], axis[2]), angle);

//   pose.orientation.w = q.w();
//   pose.orientation.x = q.x();
//   pose.orientation.y = q.y();
//   pose.orientation.z = q.z();
// }