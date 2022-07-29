#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from leo_erc_gazebo.msg import Setpoint
from nav_msgs.msg import Odometry
import math
from geometry_msgs.msg import Twist, Vector3

# TIMER
# CONTROL ACTION

class RoverRegulator:
    def __init__(self):
        # Attributes
        self.setPoint_x = 0
        self.setPoint_y = 0
        self.actualPosition_x = 0
        self.actualPosition_y = 0
        self.actualPosition_theta = 0
        self.l = 0.005

        # Subscribers
        self.odomSub = rospy.Subscriber("/zed2/odom", Odometry, self.odomCallback, queue_size=1)
        self.setpointSub = rospy.Subscriber("/setpoint", Setpoint, self.setpointCallback, queue_size=1)
        self.executeTimer = rospy.Timer(rospy.Duration(0.001), self.timerCallback)

        # Control Action Publisher
        self.control_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        

    def setpointCallback(self, data):
        print("debug")
        setpoint = data
        self.setPoint_x = setpoint.x
        self.setPoint_y = setpoint.y
        print("NUOVO OBIETTIVO: ",self.setPoint_x,self.setPoint_y, " ID: ",setpoint.id)
        

    def odomCallback(self, data):
        orientation = data.pose.pose.orientation
        position = data.pose.pose.position
        siny_cosp = 2*(orientation.w*orientation.z+orientation.x*orientation.y)
        cosy_cosp = 1-2*(orientation.y**2 + orientation.z**2)
        self.actualPosition_theta = math.atan2(siny_cosp,cosy_cosp)
        self.actualPosition_x = position.x + self.l*math.cos(self.actualPosition_theta)
        self.actualPosition_y = position.y + self.l*math.sin(self.actualPosition_theta)



        """
         double xE, yE, theta;
    double l=0.1;
    //calcolo di theta attraverso la conversione quaternion -> euler angle   DA ELIMINARE POI
    double siny_cosp = 2*(msg->pose.pose.orientation.w*msg->pose.pose.orientation.z
      +msg->pose.pose.orientation.x*msg->pose.pose.orientation.y);
    double cosy_cosp = 1-2*(msg->pose.pose.orientation.y*msg->pose.pose.orientation.y +
    msg->pose.pose.orientation.z*msg->pose.pose.orientation.z);
    theta = std::atan2(siny_cosp,cosy_cosp);
    xE = msg->pose.pose.position.x + l*cos(theta);
    yE = msg->pose.pose.position.y + l*sin(theta);
    """

    def timerCallback(self, e):

        if rospy.get_param('navigation_mode') == 'AUTO':
            error_x = self.actualPosition_x - self.setPoint_x
            error_y = self.actualPosition_y - self.setPoint_y

            k1 = 0.2
            k2 = 0.2

            v1 = -k1*error_x
            v2 = -k2*error_y

            u1 = math.cos(self.actualPosition_theta)*v1 + math.sin(self.actualPosition_theta)*v2
            u2 = -v1*(math.sin(self.actualPosition_theta/self.l)) + v2*(math.cos(self.actualPosition_theta)/self.l)
            u2 = 5*u2
            u1 = 5*u1
            
            # minimum linear velocity
            if abs(u1)<0.3:
                u1 = math.copysign(0.3,u1)
            """
            if abs(error_x)<0.05 and abs(error_y)<0.05:
                u1 = 0
                u2 = 0
            """
            

            control_action = Twist()
            linear_vel = Vector3(u1,0,0)
            angular_vel = Vector3(0,0,u2)
            control_action.linear = linear_vel
            control_action.angular = angular_vel

            self.control_pub.publish(control_action)

    """
    ###########
        double ex, ey;
    ex = xE - xRef;
    ey = yE - yRef;

    // k1 e k2 (usati anche per y)
    double k1=0.2, k2=0.2;

    double v1, v2, u1, u2;
      v1 = -k1*ex;
      v2 = -k2*ey;

      u1 = cos(theta)*v1 + sin(theta)*v2;
      u2 = -v1*(sin(theta)/l) + v2*(cos(theta)/l);
      #######
"""

if __name__ == '__main__':
    rospy.init_node('rover_regulator', anonymous=True)

    # Initialize Object
    regulator = RoverRegulator()

    rospy.spin()