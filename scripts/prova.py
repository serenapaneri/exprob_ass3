#! /usr/bin/env python2

import rospy
from exprob_ass3.srv import Command, CommandRequest

pose_client = None

def main():

    global pose_client
    rospy.init_node('prova')

    rospy.wait_for_service('arm_pose')    
    pose_client = rospy.ServiceProxy('arm_pose', Command)
    
    print('ora parte questo')
    
    pose_client('default')
    
    print('ora dovrebbe partire l altro')

    pose_client('low_detection')
    
    rospy.spin()
    
if __name__ == '__main__':
    main()
