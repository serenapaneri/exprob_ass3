#! /usr/bin/env python2

## @package exprob_ass3
#
# \file state_machine.py
# \brief script that implements the state machine of the game.
#
# \author Serena Paneri
# \version 1.0
# \date 11/11/2022
# \details
#
# Subscribes to: <BR>
#     None
#
# Publishes to: <BR>
#     None
#
# Serivces: <BR>
#     None
#
# Client Services: <BR>
#     armor_interface_srv
#
# Action Services: <BR>
#     None
#
# Description: <BR>
#     In this node there is a state machine built using the smach package.
#     There are three different states in which the robot can be:
#     - Motion: in this state there is the simulation of the motion of the robot within the
#               rooms of the game. Here there are two types of check. In the first one the
#               number of hints collected is evaluated. If it is not sufficienet the robot
#               keeps moving, instead if they are enough, there is a second check.
#               This check evaluates if the hypothesis composed by the hints found is 
#               a complete and consistent hypothesis. If it is not the robot should restart
#               again the process.
#     - Room: in this state the robot simulates the collection of hints, a new hint each
#             time it is in a new room. 
#     - Oralce: in this final state the robot asks to the oracle if the complete and 
#               consistent hypothesis that it has found is the correct one. If it is then
#               the game is finished, otherwise the robot restarts the process searching for+
#               new hints.


import rospy
import random
import smach
import smach_ros
import time
from armor_msgs.srv import * 
from armor_msgs.msg import * 
from exprob_ass3.srv import Command
from exprob_ass3.srv import Marker
from exprob_ass2.srv import Oracle

# lists of the individuals of the cluedo game
people = ["missScarlett", "colonelMustard", "mrsWhite", "mrGreen", "mrsPeacock", "profPlum"]
weapons = ["candlestick", "dagger", "leadPipe", "revolver", "rope", "spanner"]
places = ["conservatory", "lounge", "kitchen", "library", "hall", "study", "bathroom", "diningRoom", "billiardRoom"]

# list of rooms
rooms = [[-4, -3], [-4, 2], [-4, 7], [5, -7], [5, -3], [5, 1]]

# armor client
armor_interface = None
# oracle client
oracle_client = None
# hint client
hint_client = None
# arm pose client
pose_client = None

consistent = False
hint_counter = 0
hints = []
hypo = []
url = ''


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
# \brief The load function loads the cluedo_ontology.
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
# \brief This function search an element in a list.
# \param: element
# \return: hypo
#
# This function is used to store the elements found into a list in order to have them sorted,
# since the hints recieved are not.      
def classes(element):

     global people, weapons, places, hypo
     if search(people, element) == True:
         hypo.insert(0, element)
     elif search(weapons, element) == True:
         hypo.insert(1, element)
     elif search(places, element) == True:
         hypo.insert(2, element)
     return hypo
    

##
# \brief Function to upload the hints recieved in the cluedo_ontology.
# \param: ID_, key_, value_
# \return: None
#
# This function is used to upload every hints recieved from the environment in the cluedo_ontology.    
def upload_hint(ID_, key_, value_):

    req = ArmorDirectiveReq()
    req.client_name = 'state_machine'
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
# \brief Class StartGame of the state_machine.
# \param: None
# \return: None
#
# 
# The only outcomes is motion.
class StartGame(smach.State):

    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['motion'])
        
    def execute(self, userdata):
    
        global pose_client
        print('The robot is powering on')
        pose_client('start', 'default') 
        print('Initializing ARMOR')
        time.sleep(1)
        print('Loading the ontology')
        load()
        time.sleep(1)
        print('Uploading the TBox')
        tbox()
        reason()
        time.sleep(1)
        print('Disjoint the individuals of all classes')
        disjoint_individuals()
        reason()
        time.sleep(1)
        # randomize the rooms order
        random.shuffle(rooms)
        
        print('Starting the investigation')
        return 'motion'
     

##
# \brief Class Motion of the state_machine.
# \param: None
# \return: None
#
# This class should simulate the movement of the robot between the various rooms of the cluedo game.
# If the hints percieved are less than the expected number given by the subscriber, then the 
# robot should keep going searching hints in other rooms.
# If, instead, the number of hints collected is the right one the the class should check if the 
# hypothesis formed with those hints is complete and consistent.
# If it's not the robot should restart searching for collecting other hints, instead, if the 
# hypothesis is complete and consistent the robot can go to the oracle room trying its guessing.
# There are three outcomes:
# - enter room, if the robot is still collecting hints
# - go_oracle, if the hypothesis formulate is complete and consistent
# - moving, if the hypothesis formulated is uncomplete or inconsistent.  
class Motion(smach.State):

    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['enter_room','go_oracle'])
        
    def execute(self, userdata):
    
        global consistent, hint_counter    
        if consistent == True:
            # HERE IMPLEMENT MOVE_BASE TO GO (x = 0.0, y = -1.0)
            return 'go_oracle'
        else:
            if hint_count < 5:
                return 'enter_room'
            else:
                # HERE IMPLEMENT MOVE BASE TO ROOMS[-1]
                rooms.pop()
                # reset the hint counter
                hint_count = 0
                return 'enter_room'



##
# \brief Class Room of the state_machine.
# \param: None
# \return: None
#
#      
class Room(smach.State):

    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['motion, consistency'])
        
    def execute(self, userdata):

        global hint_counter
        if hint_counter < 5:
            pose_client('start', 'low_detection')
            # HERE IMPLEMENT THE SERVICE TO READ THE MARKERS
        return 'motion'
        

        
class Consistency(smach.State):

    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['motion, enter_room'])
        
    def execute(self, userdata):
    
        global consistent
        print('Checking if it is consistent ..')
        isinconsistent = inconsistent()
        if len(isinconsistent.queried_objects) != 0:
            # objects of the class INCONSISTENT
            if url in isinconsistent.queried_objects:
                print('The hypothesis is inconsistent')
                consistent = False
                return 'enter_room'
            elif url not in isinconsistent.queried_objects:
                print('The hypothesis is complete and consistent')
                print('The robot is ready to go to the oracle')
                consistent = True
                return 'motion'
        elif len(isinconsistent.queried_objects) == 0:
            print('The hypothesis is complete and consistent')
            consistent = True
            return 'motion'
            

##
# \brief Class Oracle of the state_machine.
# \param: None
# \return: None
#
# This class should simulate what happens when the robot has collected all the hints that forms
# a complete and consistent hypothesis, and it tries its guess.
# This class should inform the robot if the hypothesis found is the winning one or not, and this
# is done thanks to the oracle_service that checks the ID of the hypothesis of the robot with
# the winning one.
# If the hypothesis is correct then the game is finished. If it is not then the robot should
# restart searching for new hints and repeat all the process.
# Here we have two outcomes:
# - game_finished, if the hypothesis found is the correct one
# - motion, if the hypothesis found is wrong.     
class Oracle(smach.State):

    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['motion','game_finished'])
        
    def execute(self, userdata):
    
        global oracle_client, comm_client, hints, hypo, attempt, hint_count
        print('The robot is inside the oracle rooom')
        time.sleep(3)
        
        # waiting for the service that gives the ID of the winning hypothesis
        rospy.wait_for_service('winning_hypothesis')
        
        print('Oracle: "Name your guess"')
        time.sleep(1)
        # oracle client
        # if the two IDs coincides
        print('Yes, you guessed right!')
        save()
        return 'game_finished'
        # otherwise if they are not the same   

        print('No you are wrong, maybe next time you will have better luck')
        return 'motion' 
        

##
# \brief This is the main function of the node state_machine.
# \param: None
# \return: None
#
# In the main function the node state_machine is initialized and here the service are called
# as well as the hint subscriber.
def main():

    global armor_interface, oracle_client, hint_client, pose_client
    rospy.init_node('state_machine')
    sm = smach.StateMachine(outcomes=['game_finished'])
    
    rospy.wait_for_service('armor_interface_srv')
    print('Waiting for the armor service')
    
    # armor client
    armor_interface = rospy.ServiceProxy('armor_interface_srv', ArmorDirective)
    # oracle client
    oracle_client = rospy.ServiceProxy('/oracle_solution', Oracle)
    # command client
    hint_client = rospy.ServiceProxy('/oracle_hint', Marker)
    # arm pose client
    pose_client = rospy.ServiceProxy('arm_pose', Command)


    with sm:
        smach.StateMachine.add('StartGame', StartGame(), 
                               transitions={'motion': 'Motion'})
        smach.StateMachine.add('Motion', Motion(), 
                               transitions={'enter_room':'Room', 
                                            'go_oracle':'Oracle'})
        smach.StateMachine.add('Room', Consistency(), 
                               transitions={'motion':'Motion',
                                            'consistency': 'Consistency'})
        smach.StateMachine.add('Consistency', Room(), 
                               transitions={'motion':'Motion',
                                            'enter_room': 'Room'})
        smach.StateMachine.add('Oracle', Oracle(), 
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

