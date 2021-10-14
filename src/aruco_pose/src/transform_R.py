#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from aruco_pose.msg import MarkerArray
import numpy as np
import rigid_transform_3D
from numpy import *
from math import sqrt


def callback(data):

  if data.markers:
        rospy.loginfo(data.markers[0].id)
	rospy.loginfo(data.markers[1].id)
	rospy.loginfo(data.markers[2].id)
	rospy.loginfo(data.markers[3].id)
	rospy.loginfo(data.markers[4].id)
	rospy.loginfo(data.markers[5].id)
	rospy.loginfo(data.markers[6].id)
	rospy.loginfo(data.markers[7].id)
      
	for i in range(8):
	  k =  data.markers[i].id
	  if k == 10:
  	    a = i
	  elif k == 20:
	    b = i
          elif k == 30:
	    c = i
	  elif k == 40:
	    d = i
          elif k == 50:
	    e = i
	  elif k == 60:
	    f = i
          elif k == 70:
	    g = i
	  elif k == 80:
	    h = i

	#print(a,b,c,d,e,f)
	
	A = np.mat([[data.markers[a].pose.position.x,data.markers[b].pose.position.x,data.markers[c].pose.position.x,data.markers[d].pose.position.x,data.markers[e].pose.position.x,data.markers[f].pose.position.x,data.markers[g].pose.position.x,data.markers[h].pose.position.x],[data.markers[a].pose.position.y,data.markers[b].pose.position.y,data.markers[c].pose.position.y,data.markers[d].pose.position.y,data.markers[e].pose.position.y,data.markers[f].pose.position.y,data.markers[g].pose.position.y,data.markers[h].pose.position.y],[data.markers[a].pose.position.z,data.markers[b].pose.position.z,data.markers[c].pose.position.z,data.markers[d].pose.position.z,data.markers[e].pose.position.z,data.markers[f].pose.position.z,data.markers[g].pose.position.z,data.markers[h].pose.position.z]])	
	
	B = np.mat([[1145,1355,1145,1355,1145,1355,1145,1355],[966.3,966.3,1263.3,1263.3,1736.7,1736.7,669.3,669.3],[-510,-510,-510,-510,-510,-510,-510,-510]])	

	print(A)
	print(B)

        R, t = rigid_transform_3D.rigid_transform_3D(A, B)
        print(R)
        print(t)

	with open('/home/ubuntu/catkin_ws/src/clever/aruco_pose/cfg/R_matrix_R','wb') as f1:
            for line in R:
        	np.savetxt(f1, line, fmt='%.8f')

	with open('/home/ubuntu/catkin_ws/src/clever/aruco_pose/cfg/t_matrix_R','wb') as f2:
            for line in t:
        	np.savetxt(f2, line, fmt='%.8f')
	
	#rospy.loginfo(data.markers[0].pose.position.x)
	#rospy.loginfo(data.markers[0].pose.position.y)
	#rospy.loginfo(data.markers[0].pose.position.z)

  else:
	rospy.loginfo("none")


    
    
def listener():

    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/cam_R/aruco_detect/markers",MarkerArray , callback)

    rospy.spin()

if __name__ == '__main__':
    listener()


