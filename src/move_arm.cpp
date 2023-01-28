/** @package exprob_ass3
*
* \file move_arm.cpp
* \brief this node implements the configuration poses of the robot.
*
* \author Serena Paneri
* \version 1.0
* \date 28/1/2023
*
* \details
*
* Subscribes to: <BR>
*     None
*
* Publishes to: <BR>
*     None
*
* Serivces: <BR>
*     arm_pose
*
* Client Services: <BR>
*     None
*
* Action Client: <BR>
*     None
*
* Description: <BR>
* In this node are implemented the poses that the robot can assume during the game that
* were previously implemeted thanks to the moveit setup assistant. Also a service is
# implemented, in a way that the client could decides which pose the robot should assumed.
*/

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include<exprob_ass3/Pose.h>


/**
* \brief Callback function of the arm_pose service.
* \param req, PoseRequest
* \param res, PoseResponse
* \return true
*
* This function is the callback function of the arm_pose service in which the robot can 
* assumes two different poses that are the default one and the low_detection one.
* This poses can be selected by the client taht in this case is the state machine.
*/
bool pose_handle(exprob_ass3::Pose::Request &req, exprob_ass3::Pose::Response &res){
    
    // moveit
    moveit::planning_interface::MoveGroupInterface group("arm");
    group.setEndEffectorLink("cluedo_link");
    group.setPoseReferenceFrame("base_link");
    group.setPlannerId("RRTstar");
    group.setNumPlanningAttempts(10);
    group.setPlanningTime(10.0);
    group.allowReplanning(true);
    group.setGoalJointTolerance(0.0001);
    group.setGoalPositionTolerance(0.0001);
    group.setGoalOrientationTolerance(0.001);
    
    // default pose
    if (req.pose == "default"){
        group.setNamedTarget("default");
        group.move();
    }
    // low_detection pose
    else if (req.pose == "low_detection"){
        group.setNamedTarget("low_detection");
        group.move();
    }
    
    return true;
}


/**
* \brief Main function of the move_arm node.
* \param None
* \return 0
*
* This is the main function of the move_arm node in which the node is initialized and
* the arm_pose service is implemented.
*/
int main(int argc, char **argv){
    ros::init(argc, argv, "custom_planning");
    ros::NodeHandle nh;
    
    ros::ServiceServer pose_srv = nh.advertiseService("arm_pose", pose_handle);
    
    ros::AsyncSpinner spinner(100);
    spinner.start();
    ros::waitForShutdown();

    return 0;
}
