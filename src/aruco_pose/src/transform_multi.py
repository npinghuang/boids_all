#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from aruco_pose.msg import MarkerArray
import numpy as np
import rigid_transform_3D
from numpy import *
from math import sqrt


def callback1(data):

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
	
	B = np.mat([[891.5,891.5,891.5,891.5,1145,1145,1355,1355],[1815,1605,1395,1185,1648.5,1351.5,1648.5,1351.5],[0,0,0,0,0,0,0,0]])	

	print(A)
	print(B)

        R, t = rigid_transform_3D.rigid_transform_3D(A, B)
        print(R)
        print(t)

	with open('/home/ubuntu/catkin_ws/src/clever/aruco_pose/cfg/R1_matrix','wb') as fR:
            for line in R:
        	np.savetxt(fR, line, fmt='%.8f')
	
	with open('/home/ubuntu/catkin_ws/src/clever/aruco_pose/cfg/t1_matrix','wb') as ft:
            for line in R:
        	np.savetxt(ft, line, fmt='%.8f')
	
	#rospy.loginfo(data.markers[0].pose.position.x)
	#rospy.loginfo(data.markers[0].pose.position.y)
	#rospy.loginfo(data.markers[0].pose.position.z)

  else:
	rospy.loginfo("none")


def callback2(data):

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
	
	B = np.mat([[891.5,891.5,891.5,891.5,1145,1145,1355,1355],[1815,1605,1395,1185,1648.5,1351.5,1648.5,1351.5],[0,0,0,0,0,0,0,0]])	

	print(A)
	print(B)

        R, t = rigid_transform_3D.rigid_transform_3D(A, B)
        print(R)
        print(t)

	with open('/home/ubuntu/catkin_ws/src/clever/aruco_pose/cfg/R2_matrix','wb') as fR:
            for line in R:
        	np.savetxt(fR, line, fmt='%.8f')
	
	with open('/home/ubuntu/catkin_ws/src/clever/aruco_pose/cfg/t2_matrix','wb') as ft:
            for line in R:
        	np.savetxt(ft, line, fmt='%.8f')
	
	#rospy.loginfo(data.markers[0].pose.position.x)
	#rospy.loginfo(data.markers[0].pose.position.y)
	#rospy.loginfo(data.markers[0].pose.position.z)

  else:
	rospy.loginfo("none")


    
    
def listener():

    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("node1/aruco_detect/markers",MarkerArray , callback1)

    rospy.Subscriber("node2/aruco_detect/markers",MarkerArray , callback2)

    rospy.spin()

if __name__ == '__main__':
    listener()


