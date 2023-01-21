#! /usr/bin/env python2

import rospy
from exprob_ass3.srv import Marker
from exprob_ass3.msg import ID

pose_client = None
ID_sub_ = None
IDs = [0]

def id_callback(data):
    
    global IDs
    IDs.append(data.current_id)
    return IDs
    
    

def main():

    global hint_client, ID_sub_, IDs
    rospy.init_node('prova')

    # oracle_hint client
    rospy.wait_for_service('/oracle_hint')    
    hint_client = rospy.ServiceProxy('/oracle_hint', Marker)
    # ID subscriber
    ID_sub_ = rospy.Subscriber('/aruco_marker_publisher/ID', ID, id_callback)
    
    rate = rospy.Rate(1)
    while True:     
        
        hints = hint_client(IDs[-1])
        print(hints.oracle_hint.ID)
        print(hints.oracle_hint.key)
        print(hints.oracle_hint.value)
        rate.sleep()

    
if __name__ == '__main__':
    main()
