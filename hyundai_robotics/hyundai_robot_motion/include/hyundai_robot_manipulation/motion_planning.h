#pragma once

// ROS
#include <ros/ros.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>
#include <std_srvs/Trigger.h>
#include <actionlib/client/simple_action_client.h> // action
#include <control_msgs/FollowJointTrajectoryAction.h> // control_msgs
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/shape_messages.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>
#include <std_msgs/String.h>
#include <geometry_msgs/WrenchStamped.h>
#include <std_msgs/Bool.h>
// #include <rosbag/bag.h>
// #include <rosbag/view.h>

// MoveIT
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/ApplyPlanningScene.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_scene/planning_scene.h> // to check collision between robots and environnment
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/trajectory_processing/iterative_spline_parameterization.h> // Time parameterization
#include <moveit/trajectory_processing/iterative_time_parameterization.h> // Time parameterization
#include <moveit/robot_trajectory/robot_trajectory.h> // Time parameterization
#include <moveit/collision_detection/collision_tools.h> // to check collision between target object and the world
#include <moveit/collision_detection_fcl/fcl_compat.h> // collision check
#include <moveit/robot_state/cartesian_interpolator.h>

// etc
#include <iostream> // to output file
#include <fstream> // to output file
#include <ros/package.h> // to get this package path
#include <algorithm> // to get max value
#include <eigen_conversions/eigen_msg.h> // to convert msgs
#include <picking_vision/ObjInfo.h> 
#include <aruco_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <tf/tf.h>

namespace motion_planning{
  class MotionPlanning{
    private:
      ros::NodeHandle& nh_;
      ros::Publisher planning_scene_diff_publisher_;
      ros::Publisher marker_array_publisher_;
      ros::Publisher trajectory_publisher_;
      ros::ServiceClient planning_scene_diff_client_;
      ros::WallDuration sleep_t_;
      ros::Subscriber picking_vision_coordinate_;
      ros::Subscriber aruco_marker_detection_;
      ros::Subscriber obb_object_pose_;

      robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
      robot_model::RobotModelPtr robot_model_;
      robot_state::RobotStatePtr robot_state_;
      robot_state::JointModelGroup* jmg_;
      planning_scene::PlanningScenePtr planning_scene_;
      planning_scene_monitor::PlanningSceneMonitorPtr psm_;
      planning_pipeline::PlanningPipelinePtr planning_pipeline_;
      moveit::core::JumpThreshold jump_threshold_; 
      // moveit::core::MaxEEFStep max_cart_step_; 
      moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;

      moveit_msgs::PlanningScene planning_scene_msgs_;
      visualization_msgs::MarkerArray collision_points_;

      std::string robot_group_name_, global_link_;
      std::vector<std::string> robot_joint_names_;
      std::vector<double> robot_joint_init_;
      std::string obj_name_;
      double obj_x_;
      double obj_y_;
      double obj_z_;

      std_msgs::Header aruco_id_;
      // geometry_msgs::PoseWithCovariance aruco_coordinate_;
      double aruco_coordinate_x_;
      double aruco_coordinate_y_;
      double aruco_coordinate_z_;

      double aruco_coordinate_rx_;
      double aruco_coordinate_ry_;
      double aruco_coordinate_rz_;
      geometry_msgs::PoseStamped aruco_coordinate_;
      geometry_msgs::Pose aruco_pose_;
      geometry_msgs::Pose obb_pose_;

      double aruco_coordinate_qx_;
      double aruco_coordinate_qy_;
      double aruco_coordinate_qz_;
      double aruco_coordinate_qw_;
    

    public:
      MotionPlanning(ros::NodeHandle& nh);
      void initEvn();

      void visionMsgCallback(const picking_vision::ObjInfo& msg);
      void arucoMsgCallback(const geometry_msgs::PoseStamped& msg);
      void obbMsgCallback(const geometry_msgs::PoseStamped& msg);
      
      // Time parameterization of trajectory
      // 1)input trajectory 2)parameterized trajectory 3)factor scaling both
      bool parameterizeTime(moveit_msgs::RobotTrajectory& input_traj, moveit_msgs::RobotTrajectory& output_traj, double factor=1.0);

      // Only visualizing robot motion on Rviz
      // 1)robot trajectory 2)false if you want to see animation of robot motion
      bool showRobotMotion(moveit_msgs::RobotTrajectory robot_traj, bool blocking=false);

      // Update planning_scene_ according to robot_state and msg
      bool updatePlanningScene();

      // move robot arm to target joint value
      // 1)target joint values 2)robot trajectory
      bool moveJoint(std::vector<double> target_joints, moveit_msgs::RobotTrajectory& derived_traj);

      // move robot arm to target pose
      // 1)target pose 2)target link 3)robot trajectory
      bool moveJoint(Eigen::Isometry3d& target_pose, std::string target_link, moveit_msgs::RobotTrajectory& derived_traj);

      // generate object
      // 1)object name 2)object pose 3)object msg 4)whether show objects in planning_scene
      void genObj(std::string object_name, Eigen::Isometry3d& object_pose, moveit_msgs::CollisionObject& collision_obj, bool print_objs=true);

      void run();
  };
}