#! /usr/bin/env python2

import rospy
from exprob_ass3.srv import Command

comm_client = None
    

def main():

    global comm_client
    rospy.init_node('prova')

    # oracle_hint client
    rospy.wait_for_service('comm')    
    comm_client = rospy.ServiceProxy('comm', Command)

    comm_client('start')
    rospy.spin()

    
if __name__ == '__main__':
    main()
