#include <ros/ros.h>
#include <iostream>
#include <aruco_pose/MarkerArray.h>
#include <aruco_pose/Marker.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "Header.h"
#include <Eigen/Dense>
#include <fstream>
#include <sstream>
#include <string>
#include <aruco_pose/position.h>

using namespace message_filters;
using namespace Eigen;  
using namespace std;  


void callback(const aruco_pose::MarkerArray::ConstPtr& markers1, const aruco_pose::MarkerArray::ConstPtr& markers2)
{
		
		string Rarray[3][3];  
		ifstream Rf ("/home/ubuntu/catkin_ws/src/clever/aruco_pose/cfg/R_matrix_L"); 
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
		ifstream tf ("/home/ubuntu/catkin_ws/src/clever/aruco_pose/cfg/t_matrix_L"); 
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

	if(!markers1->markers.empty()){ 	
		/***
		ROS_INFO("id1 : %ld",markers1->markers[0].id );
		ROS_INFO("x1 : %f",markers1->markers[0].pose.position.x );
		ROS_INFO("y1 : %f",markers1->markers[0].pose.position.y );
		ROS_INFO("id1 : %ld",markers1->markers[1].id );
		ROS_INFO("x1 : %f",markers1->markers[1].pose.position.x );
		ROS_INFO("y1 : %f",markers1->markers[1].pose.position.y );	
		***/

		if(markers1->markers[0].id == 60 || markers1->markers[0].id == 70){
//			ROS_INFO("id11 : %ld",markers1->markers[0].id );			
			MatrixXf A1(3,1);  
			A1 << markers1->markers[0].pose.position.x,  
			markers1->markers[0].pose.position.y,   
			markers1->markers[0].pose.position.z; 
			MatrixXf A11(3,1);
			A11 = R*A1 + t;
			std::cout << A11.transpose() << std::endl;
		}

		if(markers1->markers[1].id == 60 || markers1->markers[1].id == 70){
//			ROS_INFO("id12 : %ld",markers1->markers[1].id );			
			MatrixXf A2(3,1);  
			A2 << markers1->markers[1].pose.position.x,  
			markers1->markers[1].pose.position.y,   
			markers1->markers[1].pose.position.z; 
			MatrixXf A12(3,1);
			A12 = R*A2 + t;
			std::cout << A12.transpose() << std::endl;
		}

		//std::cout << R*A1 + t << std::endl;	
		
	}

	if(!markers2->markers.empty()){
/***
		ROS_INFO("id2 : %ld",markers2->markers[0].id );
		ROS_INFO("x2 : %f",markers2->markers[0].pose.position.x );
		ROS_INFO("y2 : %f",markers2->markers[0].pose.position.y );	
		ROS_INFO("id2 : %ld",markers2->markers[1].id );
		ROS_INFO("x2 : %f",markers2->markers[1].pose.position.x );
		ROS_INFO("y2 : %f",markers2->markers[1].pose.position.y );	
***/

		if(markers2->markers[0].id == 60 || markers2->markers[0].id == 70){
//			ROS_INFO("id21 : %ld",markers2->markers[0].id );			
			MatrixXf A3(3,1);  
			A3 << markers2->markers[0].pose.position.x,  
			markers2->markers[0].pose.position.y,   
			markers2->markers[0].pose.position.z; 
			MatrixXf A21(3,1);
			A21 = R*A3 + t;
			std::cout << A21.transpose() << std::endl;
		}

		if(markers2->markers[1].id == 60 || markers2->markers[1].id == 70){
//			ROS_INFO("id22 : %ld",markers2->markers[1].id );			
			MatrixXf A4(3,1);  
			A4 << markers2->markers[1].pose.position.x,  
			markers2->markers[1].pose.position.y,   
			markers2->markers[1].pose.position.z; 
			MatrixXf A22(3,1);
			A22 = R*A4 + t;
			std::cout << A22.transpose() << std::endl;
		}

				
	} 	

	if(markers1->markers.empty() && markers2->markers.empty()){
		ROS_INFO("None");
	}	
	
}


int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "mapping");

  ros::NodeHandle nh;

  message_filters::Subscriber<aruco_pose::MarkerArray> node1_sub(nh, "node1/aruco_detect/markers", 20);
  message_filters::Subscriber<aruco_pose::MarkerArray> node2_sub(nh, "node2/aruco_detect/markers", 20);

  typedef sync_policies::ApproximateTime<aruco_pose::MarkerArray, aruco_pose::MarkerArray> MySyncPolicy;
  // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), node1_sub, node2_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2));

  ros::spin();

  return 0;
}






