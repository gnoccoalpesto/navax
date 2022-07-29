#!/usr/bin/env python

import rospy
import numpy as np

import sys
import os
import time

#----------------------------------------------------------------

global machine_state
#global operation_mode

#-----------------------------------------------------------------
'''
def enum(**enums):
    return type('Enum', (), enums)
State_Type=enum(INIT=0,RUN=1,ERROR=100)
'''

def operationManager(navigation_mode):
    if navigation_mode=="MANUAL":
        text_wrapper=""
    elif navigation_mode=="AUTO":
        text_wrapper=" !!! "
    print("___current navigation mode: "+text_wrapper+str(navigation_mode)+text_wrapper)
    op_mode_input=input("enter CHANGE to switch to other mode(AUTO,MANUAL): ")
    print(op_mode_input)
    if op_mode_input=="CHANGE":
        if navigation_mode=="AUTO":
            rospy.set_param("navigation_mode","MANUAL")
            time.sleep(1)

        elif navigation_mode=="MANUAL":
            rospy.set_param("navigation_mode","AUTO")
            time.sleep(1)
#-----------------------------------------------------------------

def stateChecker():
#    pub=rospy.Publisher('test_topic', String, queue_size=10)
    rospy.init_node('navigation_state_manager', anonymous=True)
    loop_rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        machine_state=rospy.get_param('navigation_state')
#        may add default value(if not set): get_param(PARAM,DEFAULT_VAL)
        navigation_mode=rospy.get_param('navigation_mode')

        if machine_state=='INIT':
            print('transitioning to RUNNING state')
            rospy.set_param('navigation_state',"RUN")

        elif machine_state=='RUN':
            operationManager(navigation_mode)


        loop_rate.sleep()

#---------------------------------------------------------------

if __name__ == '__main__':
    print('===STATE MANAGER===')
    print('setting up the system...')
    rospy.set_param("navigation_state","INIT")
    rospy.set_param("navigation_mode","MANUAL")
    stateChecker()