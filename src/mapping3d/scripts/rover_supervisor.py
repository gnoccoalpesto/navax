#!/usr/bin/env python
import rospy
from leo_erc_gazebo.msg import Setpoint
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
import math
import csv



class RoverSupervisor:


    def __init__(self):

        self.setPoint_x = 0
        self.setPoint_y = 0
        self.actualPosition_x = 0
        self.actualPosition_y = 0
        self.actualPosition_theta = 0
        self.current_setpoint = 0
        self.setpoints = []
        self.l = 0.01

        self.setpoint_pub = rospy.Publisher('/setpoint', Setpoint, queue_size=1)
        rate = rospy.Rate(10) # 10hz

        self.odom_sub = rospy.Subscriber("/zed2/odom", Odometry, self.odom_callback, queue_size=1)
        self.execute_timer = rospy.Timer(rospy.Duration(0.001), self.timer_callback)

        # INSERIRE IL PATH DEL FILE CON I SETPOINT
        with open('/home/ruggero/Desktop/newPercorso.csv') as csv_file:
            csv_reader = csv.reader(csv_file, delimiter=',')
            line_count = 0
            for row in csv_reader:
                if line_count == 0:
                    line_count += 1
                else:
                    line_count += 1
                    setpoint = Setpoint()
                    setpoint.x = float(row[0])
                    setpoint.y = float(row[1])
                    setpoint.id = int(row[3])
                    self.setpoints.append(setpoint)



    
    def odom_callback(self, data):
        orientation = data.pose.pose.orientation
        position = data.pose.pose.position
        siny_cosp = 2*(orientation.w*orientation.z+orientation.x*orientation.y)
        cosy_cosp = 1-2*(orientation.y**2 + orientation.z**2)
        self.actualPosition_theta = math.atan2(siny_cosp,cosy_cosp)
        self.actualPosition_x = position.x + self.l*math.cos(self.actualPosition_theta)
        self.actualPosition_y = position.y + self.l*math.sin(self.actualPosition_theta)

    def timer_callback(self,data):
        distance_x = abs(self.setPoint_x-self.actualPosition_x)
        distance_y = abs(self.setPoint_y-self.actualPosition_y)
        if distance_x < 0.5 and distance_y < 0.5:
            print("SETPOINT ",self.current_setpoint, " RAGGIUNTO")
            self.current_setpoint += 1
            if self.current_setpoint == len(self.setpoints):
                print("PERCORSO COMPLETATO")
                # basta mandare setpoints

        self.setPoint_x = self.setpoints[self.current_setpoint].x
        self.setPoint_y = self.setpoints[self.current_setpoint].y
        print("NUOVO SETPOINT: ",self.setPoint_x,self.setPoint_y, " ID: ",self.current_setpoint)
        self.setpoint_pub.publish(self.setpoints[self.current_setpoint])




if __name__ == '__main__':
    rospy.init_node('rover_supervisor', anonymous=True)
    supervisor = RoverSupervisor()
    rospy.spin()

"""
def talker():
    pub = rospy.Publisher('/setpoint', Setpoint, queue_size=1)
    rospy.init_node('setpoint_publisher', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    # INSERIRE IL PATH DEL FILE CON I SETPOINT
    with open('/home/carlo/Desktop/Percorso.csv') as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        line_count = 0
        setpoints = []
        for row in csv_reader:
            if line_count == 0:
                line_count += 1
            else:
                print(row[0],row[1],row[3])
                line_count += 1
                setpoint = Setpoint()
                setpoint.x = float(row[0])
                setpoint.y = float(row[1])
                setpoint.id = int(row[3])
                setpoints.append(setpoint)

    #while not rospy.is_shutdown():
    for setpoint in setpoints:

        rospy.loginfo(setpoint)
        pub.publish(setpoint)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass


actualSetpoint attributo oggett supervisor
da callback -> actualPosition

if(norma(actualSetpoint - actualPosition) < 0.05)
    setypoint reached!
    send the next one
    """