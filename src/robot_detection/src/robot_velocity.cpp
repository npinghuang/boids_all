#include <ros/ros.h>
#include "ros/time.h"
#include <robot_detection/Robot.h>
#include <robot_detection/RobotArray.h>
#include <robot_detection/Robot_Velocity.h>
#include <robot_detection/Robot_Velocity_Array.h>
#include <geometry_msgs/Pose.h>

#include <cmath>
using namespace std;

ros::Subscriber Pose_robot;
ros::Publisher Velocity_robot;

robot_detection::Robot_Velocity_Array robots_velocity_array;
robot_detection::Robot_Velocity robot_velocity;
//time
ros::Time begin_time;
ros::Duration duration;
geometry_msgs::Pose previous_pose;
_Float64 distance_calculate_total, distance_xy, velocity, velocity_x, velocity_y, velocity_z, velocity_xy;
std::vector<int> previous_status{0, 0, 0, 0, 0};
std::vector<geometry_msgs::Pose> previous_pose_array{previous_pose, previous_pose, previous_pose, previous_pose, previous_pose};

double max_velocity = 50; //won't send velocity data if exceed max_velocity
void robotsCallback(const robot_detection::RobotArray::ConstPtr &Robot){
    robots_velocity_array.robots.clear();
    robots_velocity_array.robots.resize(5);
    robots_velocity_array.state = 0;
    for (int i = 0; i < 5; i++){
        robots_velocity_array.robots[i].id = -1;
    }
    
    if (!Robot->robots.empty()){
        // ROS_INFO("time stamp %d", Robot ->header.stamp.nsec);
        if ( Robot ->header.seq != 0){
            duration = Robot -> header.stamp - begin_time;
           _Float64 tmp;
            tmp = 1.0 /( duration.nsec * pow(10, -6));
            ROS_INFO("duration %d, freq: %f", duration.nsec, tmp);
            int robot_index = 0; 
            for( int i = 0; i < Robot -> robots.size(); i++){
                if ( previous_status[Robot -> robots[i].id] == 1){ // it means that i have seen more than marker of this robot
                    // unit : distance (cm) velocity (cm/s)
                    distance_calculate_total = sqrtf128(powf64x(Robot -> robots[i].pose.position.x - previous_pose_array[Robot -> robots[i].id].position.x, 2) + 
                        powf64x(Robot -> robots[i].pose.position.y - previous_pose_array[Robot -> robots[i].id].position.y, 2) + 
                        powf64x(Robot -> robots[i].pose.position.z - previous_pose_array[Robot -> robots[i].id].position.z, 2));
                    distance_xy = sqrtf128(powf64x(Robot -> robots[i].pose.position.x - previous_pose_array[Robot -> robots[i].id].position.x, 2) + 
                        powf64x(Robot -> robots[i].pose.position.y - previous_pose_array[Robot -> robots[i].id].position.y, 2));
                    velocity = distance_calculate_total / (duration.nsec * pow(10, -9)); 
                    velocity_x = (Robot -> robots[i].pose.position.x - previous_pose_array[Robot -> robots[i].id].position.x )/ (duration.nsec * pow(10, -9)); 
                    velocity_y = (Robot -> robots[i].pose.position.y - previous_pose_array[Robot -> robots[i].id].position.y )/ (duration.nsec * pow(10, -9)); 
                    velocity_z = (Robot -> robots[i].pose.position.z - previous_pose_array[Robot -> robots[i].id].position.z )/ (duration.nsec * pow(10, -9)); 
                    velocity_xy = distance_xy / (duration.nsec * pow(10, -6)); 
                    ROS_INFO("id : [%d]", Robot -> robots[i].id);
                    // ROS_INFO("x : [%f]; y : [%f], z:  [%f], xy : [%f]", velocity_x, velocity_y, velocity_z, velocity_xy);
                    ROS_INFO("distance [%f] distance_xy [%f]  distance_z [%f] velocity [%f]", distance_calculate_total, distance_xy,  (Robot -> robots[i].pose.position.z - previous_pose_array[Robot -> robots[i].id].position.z ), velocity);
                    ROS_INFO("z now [%f] z prev [%f]" ,Robot -> robots[i].pose.position.z, previous_pose_array[Robot -> robots[i].id].position.z );
                    ROS_INFO(" x %f y %f z %f", Robot -> robots[i].pose.position.x, Robot -> robots[i].pose.position.y, Robot -> robots[i].pose.position.z);
                    if (velocity < max_velocity ){
                        robots_velocity_array.state = 1;
                        robots_velocity_array.robots[Robot -> robots[i].id].id = Robot -> robots[i].id;
                        robots_velocity_array.robots[Robot -> robots[i].id].velocity_x = float(velocity_x);
                        robots_velocity_array.robots[Robot -> robots[i].id].velocity_y = float(velocity_y);
                        robots_velocity_array.robots[Robot -> robots[i].id].velocity_z = float(velocity_z);
                        robots_velocity_array.robots[Robot -> robots[i].id].velocity_xyz = float(velocity);
                    }
                    // else{
                    //     ROS_ERROR("velocity too big!");
                    // }
                    
                }
                // else{ // first time seeing this robot
                //     previous_status[Robot -> robots[i].id] = 1;
                //     previous_pose_array[Robot -> robots[i].id] = Robot -> robots[i].pose;
                // }
            }  
        }
        //save data for next frame
        previous_pose_array.clear();
        previous_status = {0, 0, 0, 0, 0};
        for( int i = 0; i < Robot -> robots.size(); i++){
            previous_pose_array[Robot -> robots[i].id] = Robot -> robots[i].pose;
            previous_status[Robot -> robots[i].id] = 1;
        }
        begin_time = ros::Time::now();
        ROS_INFO("-------------");
    }
    else{ //no robots seen
        ROS_INFO("no velocity");
        robots_velocity_array.state = 0;
    }
    Velocity_robot.publish(robots_velocity_array);
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "robot_velocity");

    ros::NodeHandle n;

    Pose_robot = n.subscribe("pose_robot", 1, robotsCallback);
    Velocity_robot = n.advertise<robot_detection::Robot_Velocity_Array>("velocity_robot", 1);

    ros::Rate loop_rate(30);
    while (ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
    }
    ros::spin();
}