#include <ros/ros.h>

#include <moveit_msgs/GetPositionIK.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <eigen_conversions/eigen_msg.h>
#include <moveit_msgs/ExecuteTrajectoryActionResult.h>

#include <actionlib/client/simple_action_client.h>
#include <moveit_msgs/ExecuteTrajectoryAction.h>
#include <moveit_msgs/ExecuteTrajectoryActionGoal.h>

#include <tf/transform_datatypes.h>

#include "ros_igtl_bridge/igtlpoint.h"

class ismr19_control{

private:


  ros::NodeHandle nh;
  
  geometry_msgs::Point entry;
  geometry_msgs::Point target;

  ros::Subscriber sub_result;
  ros::Subscriber sub_igtl_point;
  
  bool valid_entry;
  bool valid_target;
  
public:

  ismr19_control( ros::NodeHandle& nh ):
    nh(nh),
    valid_entry(false),
    valid_target(false){
    
    sub_igtl_point = nh.subscribe( "/IGTL_POINT_IN", 10, &ismr19_control::igtl_point_cb, this );
    
    sub_result = nh.subscribe( "/execute_trajectory/result", 10, &ismr19_control::result_cb, this );    

  }

  
  void result_cb( const moveit_msgs::ExecuteTrajectoryActionResult& result ){
    ros::AsyncSpinner spinner(1);
    spinner.start();
    std::string PLANNING_GROUP = "ur5";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    
    geometry_msgs::PoseStamped current_pose = move_group.getCurrentPose( "needle" );

    geometry_msgs::Pose desired_pose = current_pose.pose;
    desired_pose.position = target;

    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(current_pose.pose);
    waypoints.push_back(desired_pose);


    ros::Duration(4).sleep();
    
    const double jump_threshold = 0.0;
    const double eef_step = 0.005;
    moveit_msgs::RobotTrajectory trajectory;
    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

    for( int i=0; i<trajectory.joint_trajectory.points.size(); i++ ){
      for( int j=0; j<trajectory.joint_trajectory.points[i].positions.size(); j++){
	//std::cout << std::setw(13) << trajectory.joint_trajectory.points[i].positions[j];
      }
      trajectory.joint_trajectory.points[i].time_from_start *= 4.0;  // slow motion down
    }

    actionlib::SimpleActionClient<moveit_msgs::ExecuteTrajectoryAction> client("/execute_trajectory", true);
    std::cout << "Wait for server" << std::endl;
    client.waitForServer();    

    moveit_msgs::ExecuteTrajectoryGoal goal;
    goal.trajectory.joint_trajectory = trajectory.joint_trajectory;
    client.sendGoal(goal);
    bool finished_before_timeout = client.waitForResult(ros::Duration(60.0));
    if(finished_before_timeout){
      actionlib::SimpleClientGoalState state = client.getState();
      ROS_INFO("Action finished: %s",state.toString().c_str());
    }

  }

  void igtl_point_cb ( const ros_igtl_bridge::igtlpoint& point ){
    if(point.name.compare("Entry") == 0){
      valid_entry = true;
      entry.x = point.pointdata.x / 1000.0; // Convert mm to m
      entry.y = point.pointdata.y / 1000.0; // Convert mm to m
      entry.z = point.pointdata.z / 1000.0; // Convert mm to m      
      ROS_INFO("[ismr19_control] Entry point has been received: \n");
    } else if(point.name.compare("Target") == 0){
      valid_target = true;
      target.x = point.pointdata.x / 1000.0; // Convert mm to m
      target.y = point.pointdata.y / 1000.0; // Convert mm to m
      target.z = point.pointdata.z / 1000.0; // Convert mm to m      
      ROS_INFO("[ismr19_control] Target point has been received: \n");
    }
    if( valid_entry && valid_target ){
      execute();
    }
  }
  
  void execute(){
    
    ros::AsyncSpinner spinner(1);
    spinner.start();
    std::string PLANNING_GROUP = "ur5";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
    
    tf::Vector3 p1( entry.x, entry.y, entry.z ); 
    tf::Vector3 p2( target.x, target.y, target.z ); 
    tf::Vector3 a = p2-p1;
    a = a / a.length();

    tf::Vector3 o( 0.0, 1.0, 0.0 );
    tf::Vector3 n = o.cross(a);
      
    tf::Matrix3x3 R( n.getX(), o.getX(), a.getX(),
		     n.getY(), o.getY(), a.getY(),
		     n.getZ(), o.getZ(), a.getZ() );

    geometry_msgs::PoseStamped pose;
    tf::poseTFToMsg( tf::Pose( R, p1 ), pose.pose );
    
    sensor_msgs::JointState seed;
    seed.name.push_back( "shoulder_pan_joint" );
    seed.name.push_back( "shoulder_lift_joint" );
    seed.name.push_back( "elbow_joint" );
    seed.name.push_back( "wrist_1_joint" );
    seed.name.push_back( "wrist_2_joint" );
    seed.name.push_back( "wrist_3_joint" );
    seed.position.push_back( -0.4735596817074883 );
    seed.position.push_back( -1.7595926907275998 );
    seed.position.push_back( 2.6589746902651235 );
    seed.position.push_back( -0.9003782216382002 );
    seed.position.push_back( -0.506688355320275 );
    seed.position.push_back( -0.7767779676822881 );

    moveit_msgs::GetPositionIK ikmsg;
    ikmsg.request.ik_request.group_name = PLANNING_GROUP;
    ikmsg.request.ik_request.robot_state.joint_state = seed;
    ikmsg.request.ik_request.robot_state.is_diff = false;
    ikmsg.request.ik_request.avoid_collisions = true;
    ikmsg.request.ik_request.pose_stamped = pose;
    ikmsg.request.ik_request.timeout = ros::Duration(5);
    ikmsg.request.ik_request.attempts = 5;

    ros::ServiceClient client = nh.serviceClient<moveit_msgs::GetPositionIK>("compute_ik");
    if( client.call( ikmsg ) ){}
    else{ std::cout << "Call to compute_ik failed" << std::endl; }
    
    if( ikmsg.response.error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS ){
      moveit_msgs::RobotTrajectory trajectory;
      move_group.setStartState(*current_state);
      move_group.setJointValueTarget(ikmsg.response.solution.joint_state);
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      moveit::planning_interface::MoveItErrorCode code = move_group.plan( plan );
      if( (bool)code ){
	ros::Duration(1.0).sleep();
	move_group.execute(plan);
      }
    }
    else{ std::cout << "IK solution failed" << std::endl; }

  }
  
};

int main( int argc, char** argv ){

  ros::init(argc, argv, "ismr19_control");
  
  ros::NodeHandle nh;
      ros::AsyncSpinner spinner(1);
      spinner.start();

  ismr19_control control(nh);

  ros::spinOnce();
  
  getchar();
  
  return 0;
}
