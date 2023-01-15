#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char **argv){
    ros::init(argc, argv, "custom_planning");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    sleep(2.0);
    
    moveit::planning_interface::MoveGroupInterface group("arm");
    group.setEndEffectorLink("arm_link_04");
    group.setPoseReferenceFrame("base_link");
    group.setPlannerId("RRTstar");
    group.setNumPlanningAttempts(10);
    group.setPlanningTime(10.0);
    group.allowReplanning(true);
    group.setGoalJointTolerance(0.0001);
    group.setGoalPositionTolerance(0.0001);
    group.setGoalOrientationTolerance(0.001);

    group.setNamedTarget("low_detection");
    group.move(); 
    return 0;
}
