#! /usr/bin/env python2

## @package exprob_ass3
#
# \file state_machine.py
# \brief script that implements the state machine of the game.
#
# \author Serena Paneri
# \version 1.0
# \date 28/01/2023
# \details
#
# Subscribes to: <BR>
#     /odom
#
# Publishes to: <BR>
#     /cmd_vel
#
# Serivces: <BR>
#     None
#
# Client Services: <BR>
#     armor_interface_srv
#     /oracle_solution
#     comm
#     arm_pose
#     room_ID
#
# Action Services: <BR>
#     move_base
#
# Description: <BR>
#     In this node there is a state machine built using the smach package.
#     There are five different states in which the robot can be:
#     - StartGame: in this state, the dafult pose of the robot is set, the ontology is loaded,
#                  the TBox is uploaded, the individuals of each class are disjoint and the
#                  order of the room is randomized. 
#     - Motion: in this state is implemented the motion of the robot within the rooms of the game
#               thanks to move_base, and also the motion to the oracle room. 
#               Here we have three outcomes meaning that three checks are perfomed.
#               One checks if the robot has visited only one or both waypoint within a room, the
#               second one checks if there are new consistent hypotheses, and in that case the 
#               robot should move to the oracle room, and finally the last is to see if the 
#               robot has already visited all the room of the game.
#     - Room: in this state the robot simulates the collection of hints, and to do that the robot
#             assumes a specific pose thanks to moveit and start rotating in order to detect the 
#             aruco markers, from which the hints are retrieved. 
#     - Complete: in this state, that is executed in each room of the game, the robot checks if there
#                 are, or not, new complete hypotheses. If there are then the robot should test their
#                 consistency, if there are not then the robot should keep searching for hints.
#     - Consistency: in this state the robot should check if there are new complete and consistent 
#                    hypotheses. If there are then the robot should go to the oracle room to try its
#                    guessing, if instead the hypotheses are inconsistent then the robot should keep 
#                    searching for hints.
#     - OralceRoom: in this final state the robot asks to the oracle if the complete and 
#                   consistent hypotheses that it has found are the correct one. If it is then
#                   the game is finished, otherwise the robot restarts the process searching for
#                   new hints.


import rospy
import random
import smach
import smach_ros
import time
import actionlib
from armor_msgs.srv import * 
from armor_msgs.msg import * 
from exprob_ass3.srv import Command
from exprob_ass3.srv import Pose
from exprob_ass3.srv import RoomID
from exprob_ass2.srv import Oracle
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf import transformations
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# lists of the individuals of the cluedo game
people = ["missScarlett", "colonelMustard", "mrsWhite", "mrGreen", "mrsPeacock", "profPlum"]
weapons = ["candlestick", "dagger", "leadPipe", "revolver", "rope", "spanner"]
places = ["conservatory", "lounge", "kitchen", "library", "hall", "study", "bathroom", "diningRoom", "billiardRoom"]

# list of rooms
room1 = [-4, -3, -1, -4]
room2 = [-4, 2, -3, 1]
room3 = [-4, 7, -1, 5]
room4 = [5, -7, 1, -8]
room5 = [5, -3, 4, -4.5]
room6 = [5, 1, 5.5, 0]
rooms = [room1, room2, room3, room4, room5, room6]

# armor client
armor_interface = None
# oracle client
oracle_client = None
# arm pose client
pose_client = None
# command client
comm_client = None
# RoomID client
roomID_client = None
# cmd_vel publisher
vel_pub = None
# move_base client
move_base_client = None

hypotheses = []
hypo_check = []
complete_hypotheses = []
consistent_hypotheses = []
inconsistent_hypotheses = []
guess = []
wrong_guess = []
right_guess = []
consistency_index = []

url = ''
elements = 0
last = 0
consistent = False
second_round = False

# robot state variables
actual_position = Point()
actual_position.x = 0
actual_position.y = 0
actual_yaw = 0 


##
# \brief Callback function of the subscriber to odom.
# \param: msg
# \return: None
#
# This is the callback function of the subscriber to the topic odom and it is used to know the actual position
# and orientation of the robot in the space.
def odom_callback(msg):
    global actual_position, actual_yaw
    
    # actual position
    actual_position = msg.pose.pose.position
    # actual yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    actual_yaw = euler[2]


##
# \brief The load function loads the cluedo_ontology.
# \param: None
# \return: None
#
# This function is used to load the cluedo_ontology.
def load():

    req = ArmorDirectiveReq()
    req.client_name = 'state_machine'
    req.reference_name = 'cluedontology'
    req.command = 'LOAD'
    req.primary_command_spec = 'FILE'
    req.secondary_command_spec = ''
    req.args = ['/root/ros_ws/src/exprob_ass3/cluedo_ontology.owl', 'http://www.emarolab.it/cluedo-ontology', 'true', 'PELLET', 'true']
    msg = armor_interface(req)
    res = msg.armor_response 


##
# \brief Function to upload all the individuals.
# \param: None
# \return: None
#
# This function is used to upload all the individuals of all classes in the ontology.
def tbox():

    global people, weapons, places
    req = ArmorDirectiveReq()
    req.client_name = 'state_machine'
    req.reference_name = 'cluedontology'
    req.command = 'ADD'
    req.primary_command_spec = 'IND'
    req.secondary_command_spec = 'CLASS'
    
    # individuals of the class suspected person
    for person in people:
    	req.args = [person, 'PERSON']
    	msg = armor_interface(req)
    	res = msg.armor_response
    print('The suspects have been uploaded in the TBox')
    
    # individuals of the class probable implements
    for weapon in weapons:
        req.args = [weapon, 'WEAPON']
        msg = armor_interface(req)
        res = msg.armor_response
    print('The implements have been uploaded in the TBox')
    
    # individuals of the class suspected scenes of murder
    for place in places:
        req.args = [place, 'PLACE']
        msg = armor_interface(req)
        res = msg.armor_response
    print('The scenes of murder have been uploaded in the TBox')
    
    
##
# \brief The disjoint_individuals function disjoints the individuals of a class.
# \param: None
# \return: None
#
# This operation needs to be done in order to disjoint the individuals belonging to each class, 
# from each other.
def disjoint_individuals():

    req = ArmorDirectiveReq()
    req.client_name = 'state_machine'
    req.reference_name = 'cluedontology'
    req.command = 'DISJOINT'
    req.primary_command_spec = 'IND'
    req.secondary_command_spec = 'CLASS'
    req.args = ['PERSON']
    msg = armor_interface(req)
    res = msg.armor_response
    print('The individuals of the class PERSON have been disjoint')
    req.args = ['WEAPON']
    msg = armor_interface(req)
    res = msg.armor_response
    print('The individuals of the class WEAPON have been disjoint')
    req.args = ['PLACE']
    msg = armor_interface(req)
    res = msg.armor_response
    print('The individuals of the class PLACE have been disjoint')


##
# \brief The reasoner of the ontology.
# \param: None
# \return: None
#
# This function implements the reasoner of the ontology that needs to be started in order to update
# the knowledge of the ontology.
def reasoner():

    req = ArmorDirectiveReq()
    req.client_name = 'state_machine'
    req.reference_name = 'cluedontology'
    req.command = 'REASON'
    req.primary_command_spec = ''
    req.secondary_command_spec = ''
    msg = armor_interface(req)
    res = msg.armor_response


##
# \brief It is the query command to retrieve an individual from a class.
# \param: None
# \return: res
#
# This functions returns, if there are any, the individuals of the class COMPLETED of the ontology.
def complete():

    req = ArmorDirectiveReq()
    req.client_name = 'state_machine'
    req.reference_name = 'cluedontology'
    req.command = 'QUERY'
    req.primary_command_spec = 'IND'
    req.secondary_command_spec = 'CLASS'
    req.args = ['COMPLETED']
    msg = armor_interface(req)
    res = msg.armor_response
    return res


##
# \brief It is the query command to retrieve an individual from a class.
# \param: None
# \return: res
#
# This functions returns, if there are any, the individuals of the class INCONSISTENT of the ontology.  
def inconsistent():

    req = ArmorDirectiveReq()
    req.client_name = 'state_machine'
    req.reference_name = 'cluedontology'
    req.command = 'QUERY'
    req.primary_command_spec = 'IND'
    req.secondary_command_spec = 'CLASS'
    req.args = ['INCONSISTENT']
    msg = armor_interface(req)
    res = msg.armor_response
    return res


##
# \brief Function that retrieves the object properties of an individual from the cluedo_ontology.
# \param: prop_name, ind_name
# \return: res
#
# This function retrives the object property of an individual from the cluedo_ontology. The ind_name stands for
# the name of the individuals whose objects propetries (prop_name) you want collect. 
def retrieve_hypo(prop_name, ind_name):

    req = ArmorDirectiveReq()
    req.client_name = 'check_hypothesis'
    req.reference_name = 'cluedontology'
    req.command = 'QUERY'
    req.primary_command_spec = 'OBJECTPROP'
    req.secondary_command_spec = 'IND'
    req.args = [prop_name, ind_name]
    msg = armor_interface(req)
    res = msg.armor_response
    return res


##
# \brief Function that "cleans" the url recieved by the cluedo ontology.
# \param: obj_query
# \return: res
#
# This function takes the string that is written as an url and remove that parl only leaving a part of
# the string that is the string of interest.    
def find_string(obj_query):

    url_ind = '<http://www.emarolab.it/cluedo-ontology#'
    obj_query = obj_query.replace(url_ind, '')
    obj_query = obj_query.replace('>', '')
    return obj_query


##
# \brief It saves the changes on a new ontology file.
# \param: None
# \return: None
#
# This functions saves the ontology in a new file, also saving the inferences.
def save():

    req = ArmorDirectiveReq()
    req.client_name = 'state_machine'
    req.reference_name = 'cluedontology'
    req.command = 'SAVE'
    req.primary_command_spec = 'INFERENCE'
    req.secondary_command_spec = ''
    req.args = ['/root/ros_ws/src/exprob_ass3/final_ontology_inferred.owl']
    msg = armor_interface(req)
    res = msg.armor_response
    print('The new ontology has been saved under the name final_ontology_inferred.owl')


##
# \brief Function that checks if elements of list2 exist in list1.
# \param: list1, list2
# \return: True, False
#
# This function is used to check if the elements in the list2 are present or not in the list1.
def search_list(list1, list2):

    result = any(item in list1 for item in list2)
    if result:
        return True
    else:
        return False


##
# \brief Function that returns the indixes in a list.
# \param: list1, list2
# \return: index, []
#
# This function is used to search the elements of a list2 in a list1 and return their indexes.  
def list_index(list1, list2):

    check = search_list(list1, list2)
    if check == True:
        index = [i for i,item in enumerate(list1) if item in list2]
        return index
    else:
        return []


##
# \brief Class StartGame of the state_machine.
# \param: None
# \return: None
#
# 
# This class should initialized all the things needed to start the investigation of the cluedo game.
# Here the robot assumes the default position, thanks to moveit, the ontology is loaded, all the 
# individuals of the game are uploaded, then disjoint from each other and feed to the reasoner. 
# Then the rooms of the game are randomized.
# Here there is only one outcome that is motion, in order to make the robot go to the various room,
# searching for hints.
class StartGame(smach.State):

    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['motion'])
        
    def execute(self, userdata):
    
        global pose_client, rooms
        print('The robot is powering on')
        # makes the robot assuming the default position
        pose_client('default') 
        print('Initializing ARMOR')
        time.sleep(1)
        print('Loading the ontology')
        # loading the ontology
        load()
        time.sleep(1)
        print('Uploading the TBox')
        # uploading all the individuals of the lists
        tbox()
        # reasoner
        reasoner()
        time.sleep(1)
        print('Disjoint the individuals of all classes')
        # disjoint the individuals of all the classes
        disjoint_individuals()
        time.sleep(1)
        # reasoners
        reasoner()
        # randomize the rooms order
        random.shuffle(rooms)
        
        print('Starting the investigation')
        return 'motion'
     

##
# \brief Class Motion of the state_machine.
# \param: None
# \return: None
#
# This class should execute the movement of the robot between the various rooms of the cluedo game.
# If there are no new complete and consistent hypotheses, then the robot should keep going searching
# new hints in other rooms, and the movement of the robot is implemented thanks to the move_base package.
# If, instead, there is actually a new complete and consistent hypothesis the robot can go to the oracle
# room, trying its guessing.
# There can be also the possibility that the robot never find the correct solution and, when there are
# no more rooms to be visited the game finish.
# So, there are three outcomes:
# - enter room, if the robot is still collecting hints
# - go_oracle, if the hypothesis formulated is complete and consistent
# - game_finished, if there are no more rooms to visit. 
class Motion(smach.State):

    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['enter_room','go_oracle', 'game_finished'])
        
    def execute(self, userdata):
    
        global consistent, rooms, move_base_client, actual_position, actual_yaw, second_round
        # move base 
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        
        if consistent == True:
            # if there is at least one consistent hypothesis the robot goes to the oracle
            goal.target_pose.pose.position.x = 0
            goal.target_pose.pose.position.y = - 1
            goal.target_pose.pose.orientation.w = 1
           
            move_base_client.wait_for_server()
            move_base_client.send_goal(goal)
            
            # do nothing until the robot reaches the goal position
            while ((actual_position.x - 0)*(actual_position.x - 0) + (actual_position.y - (-1))*(actual_position.y - (-1))) > 0.05:
                time.sleep(0.1)
            
            move_base_client.cancel_all_goals()
            print('The robot is going to the oracle room')
            return 'go_oracle'
        else:
            # if the robot still need to visit rooms
            if rooms:
                print('The robot is searching for hints')
                # going to a random room
                # visiting the first waypoint of the room
                if second_round == False:
                    print('Going to: [{}, {}]'.format(rooms[-1][0], rooms[-1][1]))
                    goal.target_pose.pose.position.x = rooms[-1][0]
                    goal.target_pose.pose.position.y = rooms[-1][1]
                    goal.target_pose.pose.orientation.w = 1
                    move_base_client.wait_for_server()
                    move_base_client.send_goal(goal)
                    
                    # do nothing until the robot reaches the goal position
                    while ((actual_position.x - rooms[-1][0])*(actual_position.x - rooms[-1][0]) + (actual_position.y - rooms[-1][1])*(actual_position.y - rooms[-1][1])) > 0.05:
                        time.sleep(0.1)
                    print('Start collecting hints')
            
                # visiting the second waypoint of the room
                if second_round == True:
                    print('Going to: [{}, {}]'.format(rooms[-1][2], rooms[-1][3]))
                    goal.target_pose.pose.position.x = rooms[-1][2]
                    goal.target_pose.pose.position.y = rooms[-1][3]
                    goal.target_pose.pose.orientation.w = 1
                    move_base_client.wait_for_server()
                    move_base_client.send_goal(goal)
                    
                    # do nothing until the robot reaches the goal position
                    while ((actual_position.x - rooms[-1][2])*(actual_position.x - rooms[-1][2]) + (actual_position.y - rooms[-1][3])*(actual_position.y - rooms[-1][3])) > 0.05:
                        time.sleep(0.1)
                    # remove the coordinates of the actual room
                    rooms.pop()
                  
                move_base_client.cancel_all_goals()
                return 'enter_room'
            # if the robot already visited all the rooms
            else:
                # save the ontology
                save()
                return 'game_finished'



##
# \brief Class Room of the state_machine.
# \param: None
# \return: None
#
# In this class the robot should collect the hints needed to form hypotheses.
# What it does is to make the robot assume the low_detection position, implemented with moveit and
# then makes the robot rotate around its yaw angle, to make a quick scan of the room and be able
# to detect the aruco markers in order to retrieve the hints.
# There are two outcomes:
# - complete, if the robot has already visited the two waypoints
# - motion, if the robot visited just the first waypoint.     
class Room(smach.State):

    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['complete', 'motion'])
        
    def execute(self, userdata):

        global comm_client, pose_client, vel_pub, second_round
        # makes the robot assuming the low_detection position
        pose_client('low_detection')
        # start detecting hints
        comm_client('start')
        
        # making the robot rotates 
        velocity = Twist()
        # setting the angular velocity
        velocity.angular.z = 0.5
        # publishing on cmd_vel 
        vel_pub.publish(velocity)
        # time needed to accomplish the task
        time.sleep(80)
        # stops the robot  
        velocity.angular.z = 0.0
        # publishing on cmd_vel 
        vel_pub.publish(velocity)
        # makes the robot assuming the default position
        pose_client('default')
        
        # if the robot is at the fist waypoint
        if second_round == False:
            second_round = True
            return 'motion'
        # if the robot is at the second waypoint
        if second_round == True:
            second_round = False
            print('Finish collecting hints')
            return 'complete'
        
        
##
# \brief Class Complete of the state_machine.
# \param: None
# \return: None
#
# This class is executed everytime the robot finish to explore the current room. In this class
# the completeness of the hypothesis collected until that moment is checked. 
# Of course, the already checked hypotheses, are no longer taken in exam and the code is structured
# to handle multiple complete hypotheses at one time, since it can happen.
# If there is at least one new complete hypothesis, then the robot should then check the consistency
# of that hypothesis, if there are not the robot goes to another room in order to collect more hints to
# try to form a complete hypothesis.
# There are two outcomes:
# - consistency, if the robot in that room find at least a complete hypothesis
# - motion, if there are not new complete hypotheses.             
class Complete(smach.State):

    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['motion', 'consistency'])
        
    def execute(self, userdata):

        global roomID_client, url, hypotheses, complete_hypotheses, elements, comm_client
        # retrieving all the IDs found in the current room
        res = roomID_client()
        room_IDs = res.roomid
        
        # stop detecting hints
        comm_client('stop')
        
        # saving all the hypotheses find in the current room
        for n in room_IDs:
            hypotheses.append('Hypothesis' + str(n))
        
        # check the completeness
        print('Checking the completeness ..')
        time.sleep(1)
        
        # checking if the hypotheses had already been checked
        indexes = list_index(hypotheses, complete_hypotheses)
        indexes.sort(reverse = True)        
        
        # removing the hypotheses that are already complete
        if indexes:
            for i in indexes:
                hypotheses.pop(i)
        
        # checking the completeness of the hypotheses not already checked  
        if hypotheses:    
            for item in hypotheses:
                url = '<http://www.emarolab.it/cluedo-ontology#{}>'.format(item)
        
                iscomplete = complete()
                time.sleep(1)

                # if the list of queried object of the class COMPLETED is empty
                if len(iscomplete.queried_objects) == 0:
                    print('The {} is uncomplete'.format(item))
                    time.sleep(1)
                # if the list of queried object of the class COMPLETED is not empty
                elif len(iscomplete.queried_objects) != 0:       
                    # checking if the current hypothesis is present or not in the list of queried objects of the class COMPLETED
                    if url not in iscomplete.queried_objects:
                        print('The {} is uncomplete'.format(item))
                        time.sleep(1)
                    elif url in iscomplete.queried_objects:
                        print('The {} is complete'.format(item))
                        time.sleep(1)
                        # save the new complete hypothesis
                        complete_hypotheses.append(item)  
        # if the current hypotheses have already been checked               
        else:
            print('No new complete hypotheses')               
        
        # checking if there are new complete hypotheses 
        if len(complete_hypotheses) > elements:
            print('A new complete hypothesis has been found')
            time.sleep(1)
            # empty hypotheses
            hypotheses.clear()
            return 'consistency'
        # if there are not
        else:
            print('No new complete hypotheses to check')
            time.sleep(1)
            # empty hypotheses
            hypotheses.clear()
            return 'motion'        


##
# \brief Class Consistency of the state_machine.
# \param: None
# \return: None
#
# This class is executed everytime a new complete hypothesis has been found. In this class
# the consistency of the hypothesis collected until that moment is checked. 
# Of course, the already checked hypotheses, are no longer taken in exam and the code is structured
# to handle multiple consistent hypotheses at one time, since it can happen.
# If there is at least one new consistent hypothesis, then the robot should go to the oracle trying its guessing.
# There is only one outcome that is motion, that allows to the robot to go to the oracle, if a new complete and
# consistent hypothesis is found, if it's not to go searching new hints.      
class Consistency(smach.State):

    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['motion'])
        
    def execute(self, userdata):
    
        global consistent, complete_hypotheses, consistent_hypotheses, inconsistent_hypotheses, elements, hypo_check, last, url, consistency_index
        # updating the threshold 
        elements = len(complete_hypotheses)
               
        print('Checking if there is one consistent hypothesis')
        time.sleep(1)
        
        # checking if the hypotheses had already been checked
        indexes_cons = list_index(complete_hypotheses, consistent_hypotheses)
        indexes_incons = list_index(complete_hypotheses, inconsistent_hypotheses)
        
        consistency_index.extend(indexes_cons)
        consistency_index.extend(indexes_incons)
        
        # create a new list containing all the elements of complete_hypotheses
        hypo_check.extend(complete_hypotheses)
        
        consistency_index.sort(reverse = True)
        
         # removing the hypotheses that have already been checked
        if consistency_index:
            for i in consistency_index:
                hypo_check.pop(i)

        consistency_index.clear()
        
        # checking the consistency of the hypotheses not already checked    
        for item in hypo_check:
            url = '<http://www.emarolab.it/cluedo-ontology#{}>'.format(item)
        
            isinconsistent = inconsistent()

            # if the list of queried object of the class COMPLETED is not empty
            if len(isinconsistent.queried_objects) != 0:
                # checking if the current hypothesis is present or not in the list of queried objects of the class INCONSISTENT
                if url in isinconsistent.queried_objects:
                    print('The {} is inconsistent'.format(item))
                    time.sleep(1)
                    inconsistent_hypotheses.append(item)
                elif url not in isinconsistent.queried_objects:
                    print('The {} is complete and consistent'.format(item))
                    time.sleep(1)
                    print('The robot is ready to go to the oracle')
                    time.sleep(1)
                    consistent_hypotheses.append(item)
            # if the list of queried object of the class INCONSISTENT is empty
            elif len(isinconsistent.queried_objects) == 0:
                print('The {} is complete and consistent'.format(item))
                time.sleep(1)
                consistent_hypotheses.append(item)
                print('The robot is ready to go to the oracle')
                time.sleep(1)
            
        # checking if there are new complete and consistent hypotheses        
        if len(consistent_hypotheses) > last:
            print('A new complete and consistent hypothesis has been found')
            time.sleep(1)
            # empty hypo_check
            hypo_check.clear()
            consistent = True
            return 'motion'
        else:
            print('No new consistent hypotheses')
            time.sleep(1)
            # empty hypo_check
            hypo_check.clear()
            consistent = False
            return 'motion'
            

##
# \brief Class Oracle of the state_machine.
# \param: None
# \return: None
#
# This class is executed when the robot has collected all the hints that forms a complete and
# consistent hypothesis, and it tries its guess.
# This class should inform the robot if the hypothesis found is the winning one or not, and this
# is done thanks to the oracle service that checks the ID of the hypothesis of the robot with
# the winning one.
# If the hypothesis is correct then the game is finished. If it is not then the robot should
# restart searching for new hints and repeat all the process.
# Of course, the already checked hypotheses, are no longer taken in exam and the code is structured
# to handle multiple guesses at one time, since it can happen.
# Here we have two outcomes:
# - game_finished, if the hypothesis found is the correct one
# - motion, if the hypothesis found is wrong.
class OracleRoom(smach.State):

    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['motion','game_finished'])
        
    def execute(self, userdata):
    
        global oracle_client, last, consistent_hypotheses, consistent, wrong_guess, guess, right_guess
        
        print('The robot is inside the oracle rooom')
        time.sleep(3)
        
        # updating the threshold
        last = len(consistent_hypotheses)
        # setting consistent to false
        consistent = False
        
        # waiting for the service that gives the ID of the winning hypothesis
        rospy.wait_for_service('/oracle_solution')
        
        print('Oracle: "Name your guess"')
        # checking if the hypotheses had already been checked
        indexes_guess = list_index(consistent_hypotheses, wrong_guess)
        
        guess.extend(consistent_hypotheses)
        
         # removing the hypotheses that have already been checked
        if indexes_guess:
            for g in indexes_guess:
                guess.pop(g)

        for item in guess:
            # retrieving the current hypotheses
            who_url = retrieve_hypo('who', item)
            what_url = retrieve_hypo('what', item)
            where_url = retrieve_hypo('where', item)
        
            who_queried = who_url.queried_objects
            what_queried = what_url.queried_objects
            where_queried = where_url.queried_objects
        
            who = find_string(who_queried[0])
            what = find_string(what_queried[0])
            where = find_string(where_queried[0])
        
            # retrieving the current ID       
            str_ = 'Hypothesis'
            cur_ID = item.replace(str_, '')
        
            # converting the string into an integer
            current_ID = int(cur_ID)
        
            print('{} with the {} in the {}'.format(who, what, where))
            time.sleep(1)
            # oracle client
            res = oracle_client()
            winning_ID = res.ID
            # if the two IDs coincides
            if current_ID == winning_ID:
                right_guess.append(item)
            # otherwise if they are not the same   
            else:
                wrong_guess.append(item)
        
        # if the current hypothesis is the winning one         
        if right_guess:
            print('Yes, you guessed right!')
            save()
            return 'game_finished'
        # if instead is not the winning one
        else:
            print('No you are wrong, maybe next time you will have better luck')
            guess.clear()
            consistent = False
            return 'motion' 
        

##
# \brief This is the main function of the node state_machine.
# \param: None
# \return: None
#
# In the main function the node state_machine is initialized and here the service are called
# as well as the odom subscriber, the cmd_vel publisher and move_base.
def main():

    global armor_interface, oracle_client, pose_client, roomID_client, comm_client, vel_pub, move_base_client
    rospy.init_node('state_machine')
    sm = smach.StateMachine(outcomes=['game_finished'])
    
    rospy.wait_for_service('armor_interface_srv')
    
    # armor client
    armor_interface = rospy.ServiceProxy('armor_interface_srv', ArmorDirective)
    # oracle client
    oracle_client = rospy.ServiceProxy('/oracle_solution', Oracle)
    # comm client
    comm_client = rospy.ServiceProxy('comm', Command)
    # arm pose client
    pose_client = rospy.ServiceProxy('arm_pose', Pose)
    # RoomID client
    roomID_client = rospy.ServiceProxy('roomID', RoomID)
    # cmd_vel publisher
    vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
    # odom subscriber
    odom_sub = rospy.Subscriber('/odom', Odometry, odom_callback)
    # move_base client
    move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)


    with sm:
        smach.StateMachine.add('StartGame', StartGame(), 
                               transitions={'motion': 'Motion'})
                               
        smach.StateMachine.add('Motion', Motion(), 
                               transitions={'enter_room':'Room', 
                                            'go_oracle':'OracleRoom',
                                            'game_finished':'game_finished'})
                                            
        smach.StateMachine.add('Room', Room(), 
                               transitions={'complete':'Complete',
                                            'motion':'Motion'})
                               
        smach.StateMachine.add('Complete', Complete(), 
                               transitions={'motion':'Motion',
                                            'consistency': 'Consistency'})
                                            
        smach.StateMachine.add('Consistency', Consistency(), 
                               transitions={'motion':'Motion'})
                               
        smach.StateMachine.add('OracleRoom', OracleRoom(), 
                               transitions={'motion':'Motion', 
                                            'game_finished':'game_finished'})


    # Create and start the introspection server for visualization
    sis = smach_ros.IntrospectionServer('state_machine', sm, '/SM_ROOT')
    sis.start()

    # Execute the state machine
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()
    
if __name__ == '__main__':
    main()

