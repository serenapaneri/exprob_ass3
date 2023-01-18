#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include<exprob_ass3/Command.h>

bool pose_handle(exprob_ass3::Command::Request &req, exprob_ass3::Command::Response &res){
    
    if (req.command == "start"){
    
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
        
        if (req.pose == "default"){
            group.setNamedTarget("default");
            group.move();
            res.ok = true;
        }
        else if (req.pose == "low_detection"){
            group.setNamedTarget("low_detection");
            group.move();
            res.ok = true;
        }
    }
    
    else if (req.command == "stop") {
        res.ok = false;
    }
    return true;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "custom_planning");
    ros::NodeHandle nh;
    
    ros::ServiceServer pose_srv = nh.advertiseService("arm_pose", pose_handle);
    
    ros::AsyncSpinner spinner(1);
    spinner.start();
    sleep(20.0);

    return 0;
}
