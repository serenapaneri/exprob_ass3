#! /usr/bin/env python2
import rospy
from armor_msgs.srv import * 
from armor_msgs.msg import * 

armor_interface = None

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
    
def main():
    global armor_interface
    rospy.init_node('save')
    
    rospy.wait_for_service('armor_interface_srv')   
    # armor client
    armor_interface = rospy.ServiceProxy('armor_interface_srv', ArmorDirective)
    
    save()
    
    rospy.spin()
    
if __name__ == '__main__':
    main()
