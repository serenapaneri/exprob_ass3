#!/usr/bin/env python2

## @package exprob_ass2
#
# \file menage_ontology.py
# \brief script to load the cluedo_ontology, to upload the TBox and to disjoint all individuals.
#
# \author Serena Paneri
# \version 1.0
# \date 5/1/2023
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
#    In this node are defined all the list of individuals, divided in people, weapons and places, 
#    the values of all those lists are stored in a parameter server.
#    This node contains a client for the ARMOR service, to be able to menage the cluedo_ontology.
#    In this node, in particular, the cluedo_ontology is loaded, all the individuals of the
#    game are upload in the ontology, thus forming the TBox of the ontology, the individuals of each
#    class are disjoint from each other and the resoner is started to update the knowledge.
 

import rospy
import random
from armor_msgs.srv import *
from armor_msgs.msg import * 

# lists of the individuals of the cluedo game
people = ["missScarlett", "colonelMustard", "mrsWhite", "mrGreen", "mrsPeacock", "profPlum"]
weapons = ["candlestick", "dagger", "leadPipe", "revolver", "rope", "spanner"]
places = ["conservatory", "lounge", "kitchen", "library", "hall", "study", "bathroom", "diningRoom", "billiardRoom"]

# armor client
armor_interface = None

##
# \brief The main function initializes and handles the client of the ARMOR service.
# \param: None
# \return: None
#
# This is the main function of the node where the node is initialized and handles the client of the
# ARMOR service. It also calls the other functions to load the cluedo_ontology, to upload the Tbox 
# and then it starts the reasoner menage the knowledge of the ontology.
def main():

    global armor_interface
    # initializing the menage_ontology node
    rospy.init_node('menage_ontology')
    
    # armor service
    rospy.wait_for_service('armor_interface_srv')
    print('Waiting for the armor service')
    armor_interface = rospy.ServiceProxy('armor_interface_srv', ArmorDirective)
    
    # loading the cluedo ontology
    load()
    # adding the individuals in the TBox
    tbox()
    # reason 
    reasoner()
    # disjoint the individuals of all the classes
    disjoint_individuals()
    # reason
    reasoner()
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Stopping")

##
# \brief The load function loads the cluedo_ontology.
# \param: None
# \return: None
#
# This function is used to load the cluedo_ontology.
def load():

    req = ArmorDirectiveReq()
    req.client_name = 'menage_ontology'
    req.reference_name = 'cluedontology'
    req.command = 'LOAD'
    req.primary_command_spec = 'FILE'
    req.secondary_command_spec = ''
    req.args = ['/root/ros_ws/src/exprob_ass2/cluedo_ontology.owl', 'http://www.emarolab.it/cluedo-ontology', 'true', 'PELLET', 'true']
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
    req.client_name = 'menage_ontology'
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
    req.client_name = 'menage_ontology'
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
    req.client_name = 'menage_ontology'
    req.reference_name = 'cluedontology'
    req.command = 'REASON'
    req.primary_command_spec = ''
    req.secondary_command_spec = ''
    msg = armor_interface(req)
    res = msg.armor_response

    
if __name__ == '__main__':
    main()
