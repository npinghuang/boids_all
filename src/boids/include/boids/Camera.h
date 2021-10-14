#ifndef CAMERA_H_
#define CAMERA_H_

#include <ros/ros.h>
#include "boids/Robot.h"
#include "boids/RobotArray.h"
#include "boids/Robot_Velocity.h"
#include "boids/Robot_Velocity_Array.h"
#include "boids/Flock.h"
#include "boids/Leader.h"
#include "boids/Boid.h"
#include <vector>

class Camera {
private:
    ros::NodeHandle n;
    ros::Subscriber sub_pose;
    ros::Subscriber sub_v;

    Boid robot[4];
    Leader leader;
    bool see;
    int robotSize;

public:
    Camera();
    void PoseFeedback(const boids::RobotArray::ConstPtr& msg);
    void VelocityFeedback(const boids::Robot_Velocity_Array::ConstPtr& msg);
    void CallCamera(Flock& flock, Leader& lead);
    bool HadRobots();
    int WhichState();
};

#endif