#pragma once

#include <preparatory_manipulation/motion_planning.h>
#include <preparatory_manipulation/grasp_generation.h>
#include <preparatory_manipulation/parameters.h>
#include <filesystem>
#include <rosbag/bag.h>
#include <fiducial_msgs/FiducialTransformArray.h>
#include <preparatory_manipulation/quaternion_average.h>
#include <tf/transform_datatypes.h>
#include <assembly_robot_msgs/rotation_obj.h>

namespace preparatory_manipulation{

class Manipulation{
private:
  ros::NodeHandle& nh_;
  preparatory_manipulation::MotionPlanning& motion_;
  preparatory_manipulation::GraspGeneration& grasp_;
  preparatory_manipulation::Params params_;
  moveit::core::JumpThreshold jump_threshold_;
  std::string rob1_power_point_, rob1_precision_point_, rob2_power_point_, rob2_precision_point_;
  std::string robs_group_name_, rob1_group_name_, rob2_group_name_, grip1_group_name_, grip2_group_name_;
  std::string target_object_file_;
  std::string global_link_;
  bool save_path_bag_, load_path_bag_, save_rotation_bag_, load_rotation_bag_;
  bool aruco_, aruco_valid_;
  bool priority_regrasp_;
  double rotation_angle_step_;
  double weight_length_, weight_angle_;
  double rob1_pregrasp_trans_, rob2_pregrasp_trans_;
  double distance_axis_grasp_;
  std::vector<double> init_rob1_, init_rob2_, init_grip1_, init_grip2_, init_joints_;
  std::vector<double> rob1_grasp_rot_, rob2_grasp_rot_, rob1_grasp_trans_, rob2_grasp_trans_
                    , rob1_sup_grasp_rot_, rob2_sup_grasp_rot_, rob1_sup_grasp_trans_, rob2_sup_grasp_trans_
                    , marker_rot_, marker_trans_;

  int32_t fiducial_id_, image_seq_;
  std::string camera_id_;
  geometry_msgs::Transform aruco_transform_;

  typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> TrajClient;

public:
  Manipulation(ros::NodeHandle& nh, preparatory_manipulation::MotionPlanning& motion, preparatory_manipulation::GraspGeneration& grasp, Params& params);

  void tfArucoCallback(const fiducial_msgs::FiducialTransformArray::Ptr& msg);

  // ArUco detect function
  // 1) aruco marker pose 2) marker number
  bool detectAruco(Eigen::Isometry3d& aruco_tf, int32_t& fiducial_id_);

  // Search ArUco marker by moving robot arm
  // 1) aruco marker pose w.r.t global frame 2) z value of marker pose w.r.t global frame
  bool searchAruco(Eigen::Isometry3d& marker_pose, int32_t fiducial_id, double marker_z=1.0);

  // convert grasp pose of both robot arm w.r.t new reference frame
  // 1)TF 2)new reference frame(relative) 3)output
  void convertRefFrame(std::pair<Eigen::Isometry3d, Eigen::Isometry3d>& grasp_pose_both_origin, Eigen::Isometry3d& relative_pose
                     , std::pair<Eigen::Isometry3d, Eigen::Isometry3d>& output);

  // convert TF w.r.t new reference frame
  // 1)TF 2)new reference frame(relative) 3)output
  void convertRefFrame(Eigen::Isometry3d& transform_mat, Eigen::Isometry3d& relative_pose, Eigen::Isometry3d& output);

  // convert vector w.r.t new reference frame
  // 1)vector 2)new reference frame(relative) 3)output
  void convertRefFrame(Eigen::Vector3d& vector, Eigen::Isometry3d& relative_pose, Eigen::Vector3d& output);

  // convert vertice position of edge to new reference frame
  // 1)vertice of edges 2)new reference frame(relative) 3)output
  void convertRefFrame(std::vector<std::tuple<Eigen::Vector3d, Eigen::Vector3d>>& edges, Eigen::Isometry3d& relative_pose
                    , std::vector<std::tuple<Eigen::Vector3d, Eigen::Vector3d>>& output);

  // convert normal vector of triangle to new reference frame
  // 1)both vertice and normal vector of edges 2)new reference frame(relative) 3)output
  void convertRefFrame(std::vector<std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d>>& edges_normal, Eigen::Isometry3d& relative_pose
                    , std::vector<std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d>>& output);

  // convert both vertice of rotation axis and rotation matrix to new reference frame
  // 1)both vertice and rotation matrix of rotation axis 2)new reference frame(relative) 3)output
  void convertRefFrame(std::vector<std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d>>& rotation_axis_mat, Eigen::Isometry3d& relative_pose
                    , std::vector<std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d>>& output);

  // derive maximum rotation amount according to the rotation axis
  // 1)one rotation axis & matrix(w.r.t global) 2)object pose 3)rotation axis(both vertice) / max step of rotation 4)final rotated object pose
  bool getMaxRotAmount(std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d>& rotation_axis_mat, Eigen::Isometry3d& object_pose
                     , std::tuple<Eigen::Vector3d, Eigen::Vector3d, int>& rotation_axis_step, Eigen::Isometry3d& rotated_pose);

  // derive maximum rotation amount according to the rotation axis
  // 1)one rotation axis(w.r.t global) 2)object pose 3)rotation axis(both vertice) / max step of rotation 4)final rotated object pose
  bool getMaxRotAmount(std::tuple<Eigen::Vector3d, Eigen::Vector3d>& rotation_axis, Eigen::Isometry3d& object_pose
                     , std::tuple<Eigen::Vector3d, Eigen::Vector3d, int>& rotation_axis_step, Eigen::Isometry3d& rotated_pose);

  // prioritizing the rotation axis
  // 1)rotation axis + max rotation step 2)output
  bool priorRotAxis(std::vector<std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d, int>>& rotation_axes_step
                  , std::vector<std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d, int>>& rotation_axis_cands);

  // Load grasp pose & support grasp pose from yaml(w.r.t object frame)
  // 1)grasp pose of rob1/rob2 2) support grasp pose of rob1/rob2
  bool loadGraspPose(std::pair<Eigen::Isometry3d, Eigen::Isometry3d>& grasp_pose_both
                   , std::pair<Eigen::Isometry3d, Eigen::Isometry3d>& sup_grasp_pose_both);

  // Set pregrasp pose
  // 1)grasp pose or support grasp pose 2)output
  bool setPregraspPose(Eigen::Isometry3d& grasp_pose, Eigen::Isometry3d& pregrasp_pose);

  // Set pregrasp pose
  // 1)grasp pose or support grasp pose 2)output
  bool setPregraspPose(std::pair<Eigen::Isometry3d, Eigen::Isometry3d>& grasp_pose_both
                     , std::pair<Eigen::Isometry3d, Eigen::Isometry3d>& pregrasp_pose_both);

  // Calculate distance between rotation axis & grasp pose. And delete rotation axis candidates if distance is short
  // 1)both grasp poses 2)rotation axis candidates 3)rotation axis candidates considering grasp point
  bool getDisRotAxis(std::pair<Eigen::Isometry3d, Eigen::Isometry3d>& grasp_pose_both
                   , std::vector<std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d, int>>& rotation_axis_cands
                   , std::vector<std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d, int>>& rotation_axis_dist_cands);

  // Calculate distance between rotation axis & grasp pose. And delete rotation axis candidates if distance is short
  // 1)grasp pose 2)rotation axis candidates 3)rotation axis candidates considering grasp point
  bool getDisRotAxis(Eigen::Isometry3d grasp_pose
                   , std::vector<std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d, int>>& rotation_axis_cands
                   , std::vector<std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d, int>>& rotation_axis_dist_cands);

  // [Separaing the method to get object pose depending on the orientation of grasping pose]
  // Rotate object with dummy gripper and find object pose(angle) where the gripper does not collide
  // 1)target object 2)grasp pose w.r.t object frame 3)pregrasp pose w.r.t. object frame 4)rotation axis candidates considering grasp point 
  // 5)rotation axis candidates and object pose considering grasp point & dummy gripper
  bool getRotObjPose(moveit_msgs::CollisionObject& target_object, std::pair<Eigen::Isometry3d, Eigen::Isometry3d>& grasp_pose
                   , std::pair<Eigen::Isometry3d, Eigen::Isometry3d>& pregrasp_pose
                   , std::vector<std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d, int>>& rotation_axis_dist_cands
                   , std::vector<std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d, int, Eigen::Isometry3d>>& rotaxis_objpose_cands);

  // [Both grasping poses are downward]
  // Rotate object with dummy gripper(open/closed) and find object pose(angle) where the gripper does not collide
  // 1)target object 2)grasp pose w.r.t object frame 3)rotation axis candidates considering grasp point 4)rotation axis candidates and object pose considering grasp point & dummy gripper
  bool downGraspPose(moveit_msgs::CollisionObject& target_object, std::pair<Eigen::Isometry3d, Eigen::Isometry3d>& grasp_pose
                 , std::vector<std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d, int>>& rotation_axis_dist_cands
                 , std::vector<std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d, int, Eigen::Isometry3d>>& rotaxis_objpose_cands);

  // [Both grasping poses are upward]
  // Rotate object with dummy gripper(open/closed) and find object pose(angle) where the gripper does not collide
  // 1)target object 2)grasp pose w.r.t object frame 3)pregrasp pose w.r.t. object frame 4)rotation axis candidates considering grasp point 
  // 5)rotation axis candidates and object pose considering grasp point & dummy gripper
  bool upGraspPose(moveit_msgs::CollisionObject& target_object, std::pair<Eigen::Isometry3d, Eigen::Isometry3d>& grasp_pose
                 , std::pair<Eigen::Isometry3d, Eigen::Isometry3d>& pregrasp_pose
                 , std::vector<std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d, int>>& rotation_axis_dist_cands
                 , std::vector<std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d, int, Eigen::Isometry3d>>& rotaxis_objpose_cands);
  
  // [One grasping pose is upward, the other is downward]
  // Rotate object with dummy gripper(open/closed) and find object pose(angle) where the gripper does not collide
  // 1)target object 2)grasp pose w.r.t object frame 3)pose direction 4)pregrasp pose w.r.t. object frame 5)rotation axis candidates considering grasp point 
  // 6)rotation axis candidates and object pose considering grasp point & dummy gripper
  bool mixGraspPose(moveit_msgs::CollisionObject& target_object, std::pair<Eigen::Isometry3d, Eigen::Isometry3d>& grasp_pose
                  , std::pair<Eigen::Isometry3d, Eigen::Isometry3d>& pregrasp_pose, std::vector<std::string> pose_direction
                  , std::vector<std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d, int>>& rotation_axis_dist_cands
                  , std::vector<std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d, int, Eigen::Isometry3d>>& rotaxis_objpose_cands);

  // get IK solutions when object is rotated
  // 1)target object 2)grasp pose w.r.t object frame 3)rotation axis / rotation matrix / rotation step / rotated object TF considering dummy grippers
  // 4)rotation axis / rotation matrix / rotation step / rotated object TF / IK solutions
  bool getIKObj(moveit_msgs::CollisionObject& target_object, std::pair<Eigen::Isometry3d, Eigen::Isometry3d>& grasp_pose
              , std::vector<std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d, int, Eigen::Isometry3d>>& rotaxis_objpose_cands
              , std::vector<std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d, int, Eigen::Isometry3d, std::vector<std::vector<double>>>>& rotaxis_objpose_ik_cands);

  // get IK solutions of support grasp when object is rotated
  // 1)target object 2)grasp pose w.r.t object frame 3)support grasp pose w.r.t object frame 4)rotation axis / rotation matrix / rotation step / rotated object TF / IK solutions
  // 5)rotation axis(start/end) / rotation matrix / rotation step / rotated object TF / supportive robot arm(rob1/rob2) / IK solutions of support grasp
  bool getSupIKObj(moveit_msgs::CollisionObject& target_object, std::pair<Eigen::Isometry3d, Eigen::Isometry3d>& grasp_pose, std::pair<Eigen::Isometry3d, Eigen::Isometry3d>& sup_grasp_pose
                 , std::vector<std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d, int, Eigen::Isometry3d, std::vector<std::vector<double>>>>& rotaxis_objpose_ik_cands
                 , std::vector<std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d, int, Eigen::Isometry3d, std::string, std::vector<std::vector<double>>>>& rotaxis_objpose_sup_ik_cands);

  // derive support robot arm to rotate object (if a regrasp motion exists, the path of the master robot arm to grip the object while regrasping motion is performed is planned)
  // 1)target object 2)support grasp pose w.r.t object frame 3) grasp pose w.r.t object frame 
  // 4)rotation axis(start/end) / rotation matrix / rotation step / rotated object TF / supportive robot arm(rob1/rob2) / IK solutions of support grasp
  // 5)rotation axis(start/end) / rotation matrix / rotation step / rotated object TF / supportive robot arm(rob1/rob2) / trajectory to rotate object / trajectory index before joint space jump occurs (negative: no jump)
  // / master: initial -> pregrasp of middle pose & pregrasp of middle pose -> middle pose / object pose when joint space jump occurs
  bool getSupRobPath(moveit_msgs::CollisionObject& target_object, std::pair<Eigen::Isometry3d, Eigen::Isometry3d>& sup_grasp_pose, std::pair<Eigen::Isometry3d, Eigen::Isometry3d>&
                   , std::vector<std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d, int, Eigen::Isometry3d, std::string, std::vector<std::vector<double>>>>& rotaxis_objpose_sup_ik_cands
                   , std::vector<std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d, int, Eigen::Isometry3d, std::string
                   , std::vector<robot_state::RobotStatePtr>, int, std::vector<moveit_msgs::RobotTrajectory>, Eigen::Isometry3d>>& rotaxis_objpose_sup_traj_cands);

  // Reverse the point of path & Do time parameterization
  // 1)origin path(moveit_msgs::RobotTrajectory) 2)reversed path(moveit_msgs::RobotTrajectory)
  bool reversePathOrder(moveit_msgs::RobotTrajectory path_origin, moveit_msgs::RobotTrajectory& path_reversed);
  
  // Reverse the point of path
  // 1)origin path(robot state) 2)reversed path(robot state)
  bool reversePathOrder(std::vector<robot_state::RobotStatePtr> path_origin, std::vector<robot_state::RobotStatePtr>& path_reversed);

  // Planning integrated path
  // 1)target object 2)rotated object pose / supportive robot arm(rob1/rob2) / path to rotate object / trajectory index before joint space jump occurs (negative: no jump) 
  // / master robot path (middle) / object pose when a joint space jump occurs 3)integrated path 4)gripper state according to sequence number / gripper(rob1_group_name_,rob2_group_name_) / getGripperOpen(),getGripperClose())
  bool planPreMPL(moveit_msgs::CollisionObject& target_object
                , std::vector<std::tuple<Eigen::Isometry3d, std::string, std::vector<robot_state::RobotStatePtr>, int, std::vector<moveit_msgs::RobotTrajectory>, Eigen::Isometry3d>>& sup_traj_right
                , std::vector<std::vector<moveit_msgs::RobotTrajectory>>& integrated_paths
                , std::multimap<uint32_t, std::pair<std::string, double>>& gripper_motion);

  // show robot motion
  // 1)target object 2)integrated path 3)gripper state according to sequence number(sequence number / gripper(rob1_group_name_,rob2_group_name_) / getGripperOpen(),getGripperClose())
  // 4) path number to show(if negative number, show all robot motions)
  bool playRobotMotion(moveit_msgs::CollisionObject& target_object
                     , std::vector<std::vector<moveit_msgs::RobotTrajectory>>& integrated_paths
                     , std::multimap<uint32_t, std::pair<std::string, double>>& gripper_motion, int path_num=-1);

  // Show robot motion all at once (gripper finger motions are not considered)
  // 1)target object 2)integrated path 3)gripper state according to sequence number(sequence number / gripper(rob1_group_name_,rob2_group_name_) / getGripperOpen(),getGripperClose())
  // 4) path number to show(if negative number, show all robot motions)
  bool playRobotMotionAllatOnce(moveit_msgs::CollisionObject& target_object
                              , std::vector<std::vector<moveit_msgs::RobotTrajectory>>& integrated_paths
                              , int path_num);

  // load robot motion
  // 1)parameters 2)target object 3)integrated path 4)gripper state according to sequence number(sequence number / gripper(rob1_group_name_,rob2_group_name_) / getGripperOpen(),getGripperClose())
  bool loadRobotMotion(preparatory_manipulation::Params& params, moveit_msgs::CollisionObject& target_object
                     , std::vector<std::vector<moveit_msgs::RobotTrajectory>>& integrated_paths
                     , std::multimap<uint32_t, std::pair<std::string, double>>& gripper_motion);

  // move gripper HW
  // 1)robot name(rob1_group_name_/rob2_group_name_) 2)open(gripper_open_:0.0)/close(gripper_close_:0.8)
  bool moveGripperHW(std::string robot_name, double joint);

  // move robot arm
  // 1)trajectory of robot arm 
  void playHW(moveit_msgs::RobotTrajectory& traj);

  // move robot arm & gripper HW
  // 1)trajectory of robot arm 2)gripper motion
  void playHW(moveit_msgs::RobotTrajectory& traj, std::multimap<uint32_t, std::pair<std::string, double>> gripper_motion);

  // Arrange the target object (Only HW is connected)
  // 1)target object
  bool arrangeObj(moveit_msgs::CollisionObject& target_object);

  // Save rotation data
  // 1)target object 2)rotation related data w.r.t global frame (start point/end point/rotation matrix/maximum rotation)
  void saveRot(moveit_msgs::CollisionObject& target_object, std::vector<std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d, int>>& rotation_axis_cands);

  // Load rotation data
  // 1)target object 2)rotation related data w.r.t global frame (start point/end point/rotation matrix/maximum rotation)
  void loadRot(moveit_msgs::CollisionObject& target_object, std::vector<std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d, int>>& rotation_axis_cands);

  void alg1(moveit_msgs::CollisionObject& target_object, std::vector<std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d, int>>& rotation_axis_cands, ros::WallTime& start_wall_time);
  void alg2(moveit_msgs::CollisionObject& target_object, std::vector<std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d, int>>& rotation_axis_cands
          , std::vector<std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d, int, Eigen::Isometry3d, std::vector<std::vector<double>>>>& rotaxis_objpose_ik_cands
          , std::vector<std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d, int, Eigen::Isometry3d, std::string, std::vector<std::vector<double>>>>& rotaxis_objpose_sup_ik_cands);
  void alg3(moveit_msgs::CollisionObject& target_object
          , std::vector<std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d, int, Eigen::Isometry3d, std::vector<std::vector<double>>>>& rotaxis_objpose_ik_cands
          , std::vector<std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d, int, Eigen::Isometry3d, std::string, std::vector<std::vector<double>>>>& rotaxis_objpose_sup_ik_cands
          , std::vector<std::vector<moveit_msgs::RobotTrajectory>>& integrated_paths
          , std::multimap<uint32_t, std::pair<std::string, double>>& gripper_motion);

  void example();
  void MarkerLocationTest();

};
}

