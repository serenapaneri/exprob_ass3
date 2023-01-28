#! /usr/bin/env python2

## @package exprob_ass3
#
# \file hints.py
# \brief script that implements the hints reciever of the game.
#
# \author Serena Paneri
# \version 1.0
# \date 28/01/2023
# \details
#
# Subscribes to: <BR>
#     /aruco_marker_publisher/ID
#
# Publishes to: <BR>
#     None
#
# Serivces: <BR>
#     comm
#     room_ID
#
# Client Services: <BR>
#     armor_interface_srv
#     /oracle_hint
#
# Action Services: <BR>
#     None
#
# Description: <BR>
#     In this node is implemented the hints reciever of the cluedo game. In particular, when
#     the state_machine commands to start recieving hints the hints are first by selecting the
#     not malformed ones and those are then uploaded in the ontology. The malformed ones are
#     simply discarded. In the meanwhile the IDs of that hints recieved are stored and sent to 
#     the state machine.

import rospy
from armor_msgs.srv import *
from armor_msgs.msg import * 
from exprob_ass3.srv import Marker
from exprob_ass3.srv import Command
from exprob_ass3.srv import RoomID, RoomIDResponse
from exprob_ass3.msg import ID

# armor interface
armor_interface = None
# oracle_hint client
hint_client = None
# command service
comm_srv = None
# ID subscriber
ID_sub_ = None
# RoomID service
roomid_srv = None

start = False
IDs = [0]
room_IDs = []


##
# \brief Callback function of the comm service.
# \param: req, CommandRequest
# \return: start
#
# This function recieves the command from the client in the state_machine node. When the client commands to 
# start then the start variable is set to True and when it commands to stop, the start variable is set to false.
def com(req):

    global start
    if (req.command == 'start'):
        start = True
    elif (req.command == 'stop'):
        start = False
    return start
    

##
# \brief Callback function of the RoomID service.
# \param: req, RoomIDRequest
# \return: res, RoomIDResponse
#
# This function simply sends the ID found within a room to the state_machine node.    
def RoomID_handle(req):
    global room_IDs
    res = RoomIDResponse()
    res.roomid = room_IDs
    return res


##
# \brief Callback function of the subcriber to aruco_marker_publisher/ID
# \param: data
# \return: IDs
#
# This function collect the ID of the aruco marker that have been detected. 
def id_callback(data):
    
    global IDs
    IDs.append(data.current_id)
    return IDs
    

##
# \brief This function search an element in a list.
# \param: list_, element
# \return: True, False
#
# This functions checks if a specific element is present (True) or not (False) in a list.     
def search(list_, element):
    """
      This function check if an element is present or not into a list
    """
    for i in range(len(list_)):
        if list_[i] == element:
            return True
    return False
    

##
# \brief Function to upload the hints recieved in the cluedo_ontology.
# \param: ID_, key_, value_
# \return: None
#
# This function is used to upload every hints recieved from the environment in the cluedo_ontology.    
def upload_hint(ID_, key_, value_):

    req = ArmorDirectiveReq()
    req.client_name = 'hints'
    req.reference_name = 'cluedontology'

    if key_ == 'who':  
        req.command = 'ADD'
        req.primary_command_spec = 'OBJECTPROP'
        req.secondary_command_spec = 'IND'
        req.args = ['who','Hypothesis' + str(ID_), value_]
        msg = armor_interface(req)
        res = msg.armor_response 
    
    elif key_ == 'what':
        req.command = 'ADD'
        req.primary_command_spec = 'OBJECTPROP'
        req.secondary_command_spec = 'IND'
        req.args = ['what','Hypothesis' + str(ID_), value_]
        msg = armor_interface(req)
        res = armor_interface(req)
    
    elif key_ == 'where':
        req.command = 'ADD'
        req.primary_command_spec = 'OBJECTPROP'
        req.secondary_command_spec = 'IND'
        req.args = ['where','Hypothesis' + str(ID_), value_]
        msg = armor_interface(req)
        res = msg.armor_response 
    
    apply_()
    reasoner()
    print("The hint has been uploaded")


##
# \brief The reasoner of the ontology.
# \param: None
# \return: None
#
# This function implements the reasoner of the ontology that needs to be started in order to update
# the knowledge of the ontology.
def reasoner():

    req = ArmorDirectiveReq()
    req.client_name = 'hints'
    req.reference_name = 'cluedontology'
    req.command = 'REASON'
    req.primary_command_spec = ''
    req.secondary_command_spec = ''
    msg = armor_interface(req)
    res = msg.armor_response


##
# \brief It is the apply command of armor.
# \param: None
# \return: None
#
# This function apply the changes done to the ontology.  
def apply_():

    req = ArmorDirectiveReq()
    req.client_name = 'state_machine'
    req.reference_name = 'cluedontology'
    req.command = 'APPLY'
    req.primary_command_spec = ''
    req.secondary_command_spec = ''
    msg = armor_interface(req)
    res = msg.armor_response
            

##
# \brief Main function of the hints node.
# \param: None
# \return: None
#
# In the main function all the service comm and roomID are implemented as well as the clients to
# ARMOR and oracle_hint. Moreover there is also the subscriber to the aruco_marker_publisher. 
# In this node when the state_machine commands to start recieving hints the hints are first
# filtered by the fact of being malformed or not, and if they are not they are uploaded in the
# ontology. In the meanwhile the IDs of that hints recieved are stored and sent to the state 
# machine.
def main():

    global hint_client, ID_sub_, comm_srv, roomid_srv, IDs, room_IDs, armor_interface
    rospy.init_node('hints')

    # armor service
    rospy.wait_for_service('armor_interface_srv')
    armor_interface = rospy.ServiceProxy('armor_interface_srv', ArmorDirective)
    # oracle_hint client
    rospy.wait_for_service('/oracle_hint')    
    hint_client = rospy.ServiceProxy('/oracle_hint', Marker)
    # ID subscriber
    ID_sub_ = rospy.Subscriber('/aruco_marker_publisher/ID', ID, id_callback) 
    # command service
    comm_srv = rospy.Service('comm', Command, com) 
    # RoomID service
    roomid_srv = rospy.Service('roomID', RoomID, RoomID_handle)
       

    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
    
        # if the state machine command to start
        if start == True:
            # retrieving the last hint collected
            hints = hint_client(IDs[-1])
            ID_ = hints.oracle_hint.ID
            key = hints.oracle_hint.key
            value = hints.oracle_hint.value
        
            # if the hint is a malformed hint
            if key == '' or value == '' or key == 'when' or value == '-1':
                print('Malformed hint, the robot will discard this')
            # instead if the hint is not a malformed one
            else:
                print('Hint collected: {}, {}, {}'.format(ID_, key, value)) 
                # uploading the hint in the ontology
                upload_hint(ID_, key, value)
                if len(room_IDs) < 1:
                    room_IDs.append(ID_)
                else:
                    element = search(room_IDs, ID_)
                    if element == True:
                        rate.sleep()
                    else:
                        room_IDs.append(ID_)     
            rate.sleep()
        # if the start machine command to stop               
        elif start == False:
            room_IDs.clear()
            rate.sleep()

    
if __name__ == '__main__':
    main()
