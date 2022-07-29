#!/usr/bin/env python
import rospy
import tf
import numpy as np
from sensor_msgs.msg import CameraInfo
from tf.transformations import quaternion_matrix
from ar_track_alvar_msgs.msg import AlvarMarkers

#t1
t1_0_tag=tf.transformations.euler_matrix(0,0,0)
t1_0_tag[0][3] = 5.702769
t1_0_tag[1][3] = 0
t1_0_tag[2][3] = -0.065271
#t2
t2_0_tag=tf.transformations.euler_matrix(0,0,0)
t2_0_tag[0][3] = 4.503994
t2_0_tag[1][3] = 6.826416
t2_0_tag[2][3] = -0.000455
#t3
t3_0_tag=tf.transformations.euler_matrix(0,0,0.86)
t3_0_tag[0][3] = 4.503994
t3_0_tag[1][3] = 6.826416
t3_0_tag[2][3] = -0.000455


def callback(data):
    
    t_base_tag = quaternion_matrix([
    data.markers[0].pose.pose.orientation.x,
    data.markers[0].pose.pose.orientation.y,
    data.markers[0].pose.pose.orientation.z,
    data.markers[0].pose.pose.orientation.w])
    t_base_tag[0][3] = data.markers[0].pose.pose.position.x
    t_base_tag[1][3] = data.markers[0].pose.pose.position.y
    t_base_tag[2][3] = data.markers[0].pose.pose.position.z

    t_tag_base=tf.transformations.inverse_matrix(t_base_tag)
    t_0_base=np.dot(t1_0_tag,t_tag_base)

    print("t_0_base")
    print(t_0_base)
    print("t_base_tag")
    print(t_base_tag)
    print("t_tag_base")
    print(t_tag_base)
    print("t1_te")
    print(t1_0_tag)

def nav4d():

  

    #t2=4.503994 6.826416 -0.000455 0 0 0   
    #t3=14.329459 -0.428800 -0.087178 0 0 0.86 
    #t4=29.215900 5.300520 -0.069001 0 0 0.53    
    #t5=12.103476 -11.714824 0.377463 0 0 1.25 
    #t6=18.821705 12.167322 0.12938 0 0 0.73 
    #t7=18.397116 3.251184 -0.032864 0 0 0.19 
    #t8=18.566999 -18.526400 0.0577 0 0 1.16 
    #t9=12.228898 7.785070 0.127643 0 0 1.47   
    #t10=20.629101 -8.191796 -0.069001 0 0 0.61   
    #t11=29.695601 -16.672436 0.078902 0 0 0.16 
    #t12=28.375999 11.429800 -0.09303 0 0 0.21 
    #t13=30.889601 -5.985371 0.008829 0 0 0.61  
    #t14=1.717059 -10.908800 -0.009376 0 0 0.73   
    #t15=5.813930 14.724000 0.114653 0 0 -0.457594 
    





    #da base_link a camera
    t_base_camera = quaternion_matrix([0, 0.10451, 0, 0.99452])
    t_base_camera[0][3] = 0.0971
    t_base_camera[1][3] = 0
    t_base_camera[2][3] = -0.0427
   # print(t_base_camera)
    #da camera a base_link
    t_camera_base=tf.transformations.inverse_matrix(t_base_camera)
   # print(t_camera_base)
    #da camera a arTag
    t_camera_tag = quaternion_matrix([0, 0, 0, 0])
    t_camera_tag[0][3] = 0
    t_camera_tag[1][3] = 0
    t_camera_tag[2][3] = 0
    #base a tag
    t_base_tag=t_base_camera*t_camera_tag
    #tag a base
    t_tag_base=tf.transformations.inverse_matrix(t_base_tag)
    #tu sai t_0_tag
    #t_0_base=t_0_tag*t_tag_base





    rospy.init_node('nav4d', anonymous=True)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.Subscriber("/ar_pose_marker", AlvarMarkers, callback)
    
    
    
    
    rospy.spin()
    print(t_base_tag)

    
    

if __name__== '__main__':
    nav4d()