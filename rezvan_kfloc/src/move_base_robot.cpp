#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <math.h>
#define NUM_POSES 4

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "move_base_robot");
    

    MoveBaseClient ac("move_base", true);

    while (!ac.waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the move_base action server to come up...");
    }
  
    std::vector<geometry_msgs::Pose> poses;

    poses.resize(NUM_POSES);

    tf2::Quaternion quat;
    quat.setRPY(0, 0, 0);
    
    poses[0].position.x = 2;
    poses[0].position.y = 0.5;
    poses[0].orientation = tf2::toMsg(quat); 
  
    quat.setRPY(0, 0, -M_PI / 2);  
    poses[1].position.x = 2;
    poses[1].position.y = -1;
    poses[1].orientation = tf2::toMsg(quat); 

    quat.setRPY(0, 0, -M_PI);
    poses[2].position.x = -1;
    poses[2].position.y = -2;
    poses[2].orientation =  tf2::toMsg(quat);

    quat.setRPY(0, 0, M_PI/2);
    poses[3].position.x = -2;
    poses[3].position.y = 0;
    poses[3].orientation = tf2::toMsg(quat);   

    move_base_msgs::MoveBaseGoal goal_pose;
    goal_pose.target_pose.header.frame_id = "map";
    goal_pose.target_pose.header.stamp = ros::Time::now();

    ros::Rate loop_rate(10);  

    for(size_t i = 0; i < poses.size(); i++)
    {
      goal_pose.target_pose.pose.position.x = poses[i].position.x;
      goal_pose.target_pose.pose.position.y = poses[i].position.y;
      goal_pose.target_pose.pose.orientation.x = poses[i].orientation.x;
      goal_pose.target_pose.pose.orientation.y = poses[i].orientation.y;
      goal_pose.target_pose.pose.orientation.z = poses[i].orientation.z;
      goal_pose.target_pose.pose.orientation.w = poses[i].orientation.w;

      ac.sendGoal(goal_pose);
      ac.waitForResult();

      if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      {
        ROS_INFO("Robot reached the goal pose: (%f, %f)", poses[i].position.x, poses[i].position.y);
      }
      else
      {
        ROS_INFO("The base failed to move to the goal pose: (%f, %f) for some misterious reasons", poses[i].position.x, poses[i].position.y);
      }


      ros::Duration(5.0).sleep();
    }

    return 0;
}