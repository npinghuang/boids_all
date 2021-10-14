#include <ros/ros.h>
#include <iostream>
#include <aruco_pose/MarkerArray.h>
#include <aruco_pose/Marker.h>
#include "Header.h"
#include <Eigen/Dense>
#include <fstream>
#include <sstream>
#include <string>
#include <aruco_pose/position.h>
#include <math.h>

#define PI 3.141592653689793

using namespace Eigen;  
using namespace std;  

class SubscribeAndPublish
{
public:
	SubscribeAndPublish()
	{    
	//Topic you want to publish
	pub_ = n_.advertise<aruco_pose::position>("/enemy_pose_R", 1);

	//Topic you want to subscribe
	sub_ = n_.subscribe("/cam_R/aruco_detect/markers", 1, &SubscribeAndPublish::callback, this);
	
	}
	
	aruco_pose::position msg;

	float pre_A11_x;
	float pre_A11_y;	
	float pre_A12_x;
	float pre_A12_y;

/***
	float r = 0.325;	

	float yaw;
	float pitch;
	float roll;

	float yaw_a;
	float pitch_a;
	float roll_a;

	float yaw_b;
	float pitch_b;
	float roll_b;

	float tan2_yaw_a;
	float tan2_pitch_a;
	float tan2_roll_a;

	float tan2_yaw_b;
	float tan2_pitch_b;
	float tan2_roll_b;

	float nx_a;
	float ny_a;
	float nz_a;

	float nx_b;
	float ny_b;
	float nz_b;


	void AngleTransform(float x, float y, float z, float w)
	{
		yaw = atan2(2 * (w * z + x * y), 1 - 2 * (z * z + y * y));
		yaw *= -1;

		yaw /= 3.1415926;
		yaw *= 180;
		if(yaw < 0)
		    yaw += 360;

		pitch = asin( 2 * (w * y - x * z));
		pitch *= -1;

		pitch /= 3.1415926;
		pitch *= 180;
		if(pitch < 0)
		    pitch += 360;

		roll = -atan2(2 * (y * z + x * w), 1 - 2 * (z * z - w * w));
		roll *= -1;

		roll /= 3.1415926;
		roll *= 180;
		if(roll < 0)
		    roll += 360;

	}

***/
	void callback(const aruco_pose::MarkerArray::ConstPtr& markers)
	{	
		string Rarray[3][3];  
		ifstream Rf ("/home/ubuntu/catkin_ws/src/clever/aruco_pose/cfg/R_matrix_R"); 
		if (Rf.is_open()){
		    while (! Rf.eof() ) {
				for (int i = 0; i < 3; i++){
					for (int j = 0; j < 3; j++){ 
						Rf >> Rarray[i][j];
					}			
				}        
			}
		    Rf.close(); 
		}
		else cout << "can't open the file"; 

		float R00,R01,R02,R10,R11,R12,R20,R21,R22;
		R00 = std::stof(Rarray[0][0]);
		R01 = std::stof(Rarray[0][1]);
		R02 = std::stof(Rarray[0][2]);
		R10 = std::stof(Rarray[1][0]);
		R11 = std::stof(Rarray[1][1]);
		R12 = std::stof(Rarray[1][2]);
		R20 = std::stof(Rarray[2][0]);
		R21 = std::stof(Rarray[2][1]);
		R22 = std::stof(Rarray[2][2]);
	 
		Matrix3f R;
		R << R00,R01,R02,
		R10,R11,R12,
		R20,R21,R22;
		//std::cout << R << std::endl;

		string tarray[3][1];  
		ifstream tf ("/home/ubuntu/catkin_ws/src/clever/aruco_pose/cfg/t_matrix_R"); 
		if (tf.is_open()){
		    while (! tf.eof() ) {
				for (int i = 0; i < 3; i++){			
					tf >> tarray[i][0];			
				}        
			}
		    tf.close(); 
		}
		else cout << "can't open the file"; 

		float t00,t10,t20;	
		t00 = std::stof(tarray[0][0]);
		t10 = std::stof(tarray[1][0]);
		t20 = std::stof(tarray[2][0]);   

		MatrixXf t(3,1);
		t << t00,
		t10,
		t20;
		//std::cout << t << std::endl;

		msg.enemy1_x = 0;
		msg.enemy1_y = 0;
		msg.enemy2_x = 0;
		msg.enemy2_y = 0;

		if(!markers->markers.empty()){ 	
			/***
			ROS_INFO("id1 : %ld",markers1->markers[0].id );
			ROS_INFO("x1 : %f",markers1->markers[0].pose.position.x );
			ROS_INFO("y1 : %f",markers1->markers[0].pose.position.y );
			ROS_INFO("id1 : %ld",markers1->markers[1].id );
			ROS_INFO("x1 : %f",markers1->markers[1].pose.position.x );
			ROS_INFO("y1 : %f",markers1->markers[1].pose.position.y );	
			***/

			int a = 30;
			int b = 30;		
	
			for(int i = 0; i < 24; i++){
				if(markers->markers[i].id == 60){
					a = i;
//					AngleTransform(markers->markers[i].pose.orientation.x,markers->markers[i].pose.orientation.y,markers->markers[i].pose.orientation.z,markers->markers[i].pose.orientation.w);
//					pitch_a = pitch;
//					roll_a = roll;
//					yaw_a = yaw;
					//ROS_INFO("a--pitch, roll, yaw : %f, %f, %f",pitch_a,roll_a,yaw_a);
				}
				else if(markers->markers[i].id == 2){
					b = i;
//					AngleTransform(markers->markers[i].pose.orientation.x,markers->markers[i].pose.orientation.y,markers->markers[i].pose.orientation.z,markers->markers[i].pose.orientation.w);
//					pitch_b = pitch;
//					roll_b = roll;
//					yaw_b = yaw;
					//ROS_INFO("b--pitch, roll, yaw : %f, %f, %f",pitch_b,roll_b,yaw_b);
				}
			}

			MatrixXf A11(3,1);
			MatrixXf A12(3,1);
//			aruco_pose::position msg;
			
			if(a < 24){
				ROS_INFO("id11 : %ld",markers->markers[a].id );			
				MatrixXf A1(3,1);  
/***
				tan2_yaw_a = pow(tan(yaw_a),2);
				tan2_pitch_a = pow(tan(pitch_a),2);
				tan2_roll_a = pow(tan(roll_a),2);

				nx_a = sqrt(1/(1+tan2_pitch_a*tan2_roll_a)+tan2_roll_a);
				ny_a = sqrt(1/(1+tan2_yaw_a*tan2_roll_a)+tan2_yaw_a);
				nz_a = sqrt(1/(1+tan2_pitch_a*tan2_yaw_a)+tan2_pitch_a);
				
				A1 << markers->markers[a].pose.position.x - 3*nx_a,  
				markers->markers[a].pose.position.y - 3*ny_a,   
				markers->markers[a].pose.position.z - 3*nz_a; 
***/
				A1 << markers->markers[a].pose.position.x,  
				markers->markers[a].pose.position.y,   
				markers->markers[a].pose.position.z; 
				
				A11 = R*A1 + t;
				std::cout << A11.transpose() << std::endl;
				//std::cout << A11(1,0) << std::endl;	


				if(fabs(pre_A11_x-A11(0,0))<50 && fabs(pre_A11_y-A11(1,0))<50){
					msg.enemy1_x = A11(0,0);
					msg.enemy1_y = A11(1,0);
				}
				else{
					msg.enemy1_x = -1;
					msg.enemy1_y = -1;
				}		
				
				pre_A11_x = A11(0,0);
				pre_A11_y = A11(1,0);
			}

			if(b < 24){
				ROS_INFO("id12 : %ld",markers->markers[b].id );			
				MatrixXf A2(3,1);  
/***
				tan2_yaw_b = pow(tan(yaw_b),2);
				tan2_pitch_b = pow(tan(pitch_b),2);
				tan2_roll_b = pow(tan(roll_b),2);

				nx_b = sqrt(1/(1+tan2_pitch_b*tan2_roll_b)+tan2_roll_b);
				ny_b = sqrt(1/(1+tan2_yaw_b*tan2_roll_b)+tan2_yaw_b);
				nz_b = sqrt(1/(1+tan2_pitch_b*tan2_yaw_b)+tan2_pitch_b);

				A2 << markers->markers[b].pose.position.x - 3*nx_b,  
				markers->markers[b].pose.position.y - 3*ny_b,   
				markers->markers[b].pose.position.z - 3*nz_b; 
***/
				A2 << markers->markers[b].pose.position.x,  
				markers->markers[b].pose.position.y,   
				markers->markers[b].pose.position.z; 

				A12 = R*A2 + t;
				std::cout << A12.transpose() << std::endl;

				if(fabs(pre_A12_x-A12(0,0))<50 && fabs(pre_A12_y-A12(1,0))<50){
					msg.enemy2_x = A12(0,0);
					msg.enemy2_y = A12(1,0);
				}
				else{
					msg.enemy2_x = -1;
					msg.enemy2_y = -1;
				}		
				
				pre_A12_x = A12(0,0);
				pre_A12_y = A12(1,0);		
			}
			
			if(a > 24){
				msg.enemy1_x = -1;
				msg.enemy1_y = -1;
				
			}

			if(b > 24){
				msg.enemy2_x = -1;
				msg.enemy2_y = -1;
				
			}
			pub_.publish(msg);	
	
		}

		else{
			ROS_INFO("None");
			msg.enemy1_x = -1;
			msg.enemy1_y = -1;
			msg.enemy2_x = -1;
			msg.enemy2_y = -1;
		}								

	}
	


	

private:
	ros::NodeHandle n_; 
	ros::Publisher pub_;
	ros::Subscriber sub_;

};



int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "mapping_class_R");

  SubscribeAndPublish SAPObject;

  ros::spin();

  return 0;
}








