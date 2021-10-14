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

/***
    ROS_INFO("id:%ld",markers->markers[0].id );
	ROS_INFO("id:%ld",markers->markers[1].id );
	ROS_INFO("id:%ld",markers->markers[2].id );
	ROS_INFO("id:%ld",markers->markers[3].id );
	ROS_INFO("id:%ld",markers->markers[4].id );
	ROS_INFO("id:%ld",markers->markers[5].id );
	ROS_INFO("id:%ld",markers->markers[6].id );
	ROS_INFO("id:%ld",markers->markers[7].id );

    ROS_INFO("x:%f",markers->markers[0].pose.position.x );
    ROS_INFO("y:%f",markers->markers[0].pose.position.y );
    ROS_INFO("z:%f",markers->markers[0].pose.position.z );
***/
 
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

/***	
	for (int i = 0; i < 3; i++){
		for (int j = 0; j < 7; j++){
			cout << array[i][j] << ' ';
		}
		cout << endl;			
	}        
***/
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

/*** 
    R << 0.00802022, -0.46992044,  0.88267234,  
  	-0.99969089, -0.02454104, -0.00398177,  
  	0.02353281, -0.88236756, -0.46997201;  
***/
 
//    std::cout << R << std::endl;

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

/***
    t << -33.5440957,
	1496.97146057,
	457.92227559;
***/
//	std::cout << t << std::endl;
/***
	ROS_INFO("id1:%ld",markers->markers[0].id );
	ROS_INFO("id2:%ld",markers->markers[1].id );

    MatrixXf A1(3,1);  
    A1 << markers->markers[0].pose.position.x,  
    markers->markers[0].pose.position.y,   
    markers->markers[0].pose.position.z; 
	MatrixXf A11(3,1);
	A11 = R*A1 + t;

	MatrixXf A2(3,1);  
    A2 << markers->markers[1].pose.position.x,  
    markers->markers[1].pose.position.y,   
    markers->markers[1].pose.position.z; 
	MatrixXf A22(3,1);
	A22 = R*A2 + t;

	//std::cout << R*A1 + t << std::endl;
    std::cout << A11.transpose() << std::endl;
	std::cout << A22.transpose() << std::endl;
 ***/

	if(markers->markers[0].id == 2 || markers->markers[0].id == 70){
			ROS_INFO("id11 : %ld",markers->markers[0].id );			
			MatrixXf A1(3,1);  
			A1 << markers->markers[0].pose.position.x,  
			markers->markers[0].pose.position.y,   
			markers->markers[0].pose.position.z; 
			MatrixXf A11(3,1);
			A11 = R*A1 + t;
			std::cout << A11.transpose() << std::endl;
		}

		if(markers->markers[1].id == 2 || markers->markers[1].id == 70){
			ROS_INFO("id12 : %ld",markers->markers[1].id );			
			MatrixXf A2(3,1);  
			A2 << markers->markers[1].pose.position.x,  
			markers->markers[1].pose.position.y,   
			markers->markers[1].pose.position.z; 
			MatrixXf A12(3,1);
			A12 = R*A2 + t;
			std::cout << A12.transpose() << std::endl;
		}
	
  }

  else{
    ROS_INFO("None");
  }

}



int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "listener");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("aruco_detect/markers", 1000, markersCallback);

  ros::spin();

  return 0;
}

