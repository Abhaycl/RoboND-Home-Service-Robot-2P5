#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Define a client for to send goal requests to the move base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


int main(int argc, char** argv) {
    // Initialize the pick_objects node
    ros::init(argc, argv, "pick_objects");

    // Tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    // Wait 5 sec for move_base action server to come up
    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move base action server to come up.");
    }

    move_base_msgs::MoveBaseGoal goal;

    // Set up the frame parameters
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    // Define a position and orientation for the turtlebot to reach
    goal.target_pose.pose.position.x = 4.62;
    goal.target_pose.pose.position.y = 2.63;
    goal.target_pose.pose.orientation.w = 1.0;

    // Send the goal position and orientation for the turtlebot to reach
    ROS_INFO("Sending goal.");
    ac.sendGoal(goal);

    // Wait an infinite time for the results
    ac.waitForResult();

    // Check if the turtlebot reached its goal
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Goal: Turtlebot reached pick-up:");
        ROS_INFO("Wait for 5 seconds.");
        ros::Duration(5.0).sleep();

        // Defining a drop-off goal for the turtlebot to go after pick-up
        goal.target_pose.pose.position.x = 3.45;
        goal.target_pose.pose.position.y = -0.21;
        goal.target_pose.pose.orientation.w = 1.0;

        ROS_INFO("Sending drop-off goal.");
        ac.sendGoal(goal);
        ac.waitForResult();

        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("Final: Turtlebot reached drop-off.");
        }
        else {
            ROS_INFO("The turtlebot failed to reach drop-off.");
        }
    }
    else {
        ROS_INFO("The turtlebot sucked this time around.");
    }

    return 0;
}