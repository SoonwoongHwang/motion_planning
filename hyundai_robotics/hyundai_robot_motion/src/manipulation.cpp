#include <preparatory_manipulation/manipulation.h>

namespace preparatory_manipulation{

Manipulation::Manipulation(ros::NodeHandle& nh, preparatory_manipulation::MotionPlanning& motion
                         , preparatory_manipulation::GraspGeneration& grasp, Params& params) : 
    nh_(nh)
  , motion_(motion)
  , grasp_(grasp)
  , params_(params)
  , rotation_angle_step_(params.rotation_angle_step)
  , rob1_power_point_(params.rob1_power_point)
  , rob1_precision_point_(params.rob1_precision_point)
  , rob2_power_point_(params.rob2_power_point)
  , rob2_precision_point_(params.rob2_precision_point)
  , weight_length_(params.weight_length)
  , weight_angle_(params.weight_angle)
  , rob1_grasp_rot_(params.rob1_grasp_rot)
  , rob2_grasp_rot_(params.rob2_grasp_rot)
  , rob1_grasp_trans_(params.rob1_grasp_trans)
  , rob2_grasp_trans_(params.rob2_grasp_trans)
  , rob1_sup_grasp_rot_(params.rob1_sup_grasp_rot)
  , rob2_sup_grasp_rot_(params.rob2_sup_grasp_rot)
  , rob1_sup_grasp_trans_(params.rob1_sup_grasp_trans)
  , rob2_sup_grasp_trans_(params.rob2_sup_grasp_trans)
  , rob1_pregrasp_trans_(params.rob1_pregrasp_trans)
  , rob2_pregrasp_trans_(params.rob2_pregrasp_trans)
  , robs_group_name_(params.robs_group_name)
  , rob1_group_name_(params.rob1_group_name)
  , rob2_group_name_(params.rob2_group_name)
  , grip1_group_name_(params.grip1_group_name)
  , grip2_group_name_(params.grip2_group_name)
  , distance_axis_grasp_(params.distance_axis_grasp)
  , save_path_bag_(params.save_path_bag)
  , load_path_bag_(params.load_path_bag)
  , jump_threshold_(params.jt_revolute, params.jt_prismatic)
  , target_object_file_(params.target_object_file)
  , init_rob1_(params.init_rob1)
  , init_rob2_(params.init_rob2)
  , init_grip1_(params.init_grip1)
  , init_grip2_(params.init_grip2)
  , aruco_(params.aruco)
  , aruco_valid_(false)
  , global_link_(params.global_link)
  , marker_rot_(params.marker_rot)
  , marker_trans_(params.marker_trans)
  , priority_regrasp_(params.priority_regrasp)
  , save_rotation_bag_(params.save_rotation_bag)
  , load_rotation_bag_(params.load_rotation_bag)
{}

// Msg Callback function 
void Manipulation::tfArucoCallback(const fiducial_msgs::FiducialTransformArray::Ptr& msg){
  if(msg->transforms.empty())   aruco_valid_ = false;
  else{
    aruco_valid_ = true;
    fiducial_id_ = msg->transforms[0].fiducial_id;
    aruco_transform_ = msg->transforms[0].transform;
    image_seq_ = msg->image_seq;
    camera_id_ = msg->header.frame_id;
  }
}

// ArUco detect function
// 1) aruco marker pose 2) marker number
bool Manipulation::detectAruco(Eigen::Isometry3d& aruco_tf, int32_t& fiducial_id){
  ROS_INFO_STREAM("Start to detect Aruco marker");
  ros::Subscriber aruco_sub;
  aruco_sub = nh_.subscribe("/fiducial_transforms", 10, &Manipulation::tfArucoCallback, this);

  int sample_size = 10;
  int wait_count = 0;
  int32_t image_seq_bf = 0;
  bool complete_cam1 = false;
  bool complete_cam2 = false;
  Eigen::Isometry3d aruco_tf_tmp, aruco_tf_cam1, aruco_tf_cam2;
  std::vector<Eigen::Isometry3d> aruco_tfs_cam1, aruco_tfs_cam2;
  std::vector<Eigen::Vector4d> quaternions_cam1, quaternions_cam2, quaternions;

  while(true){
    if(aruco_valid_){
      aruco_valid_ = false;
      fiducial_id = fiducial_id_;
      wait_count = 0;
      
      if(abs(image_seq_ - image_seq_bf)<100){ // The times of detected markers are similar
        aruco_tf_tmp = tf2::transformToEigen(aruco_transform_);
        if(camera_id_ == "usb_cam_1" && aruco_tfs_cam1.size()<sample_size)       aruco_tfs_cam1.push_back(aruco_tf_tmp);
        else if(camera_id_ == "usb_cam_2" && aruco_tfs_cam2.size()<sample_size)  aruco_tfs_cam2.push_back(aruco_tf_tmp);
        else    ROS_ERROR_STREAM("'camera_id' is not correct in ArUco.");
        image_seq_bf = image_seq_;

        ROS_INFO_STREAM("Averaging the pose of Aruco markers(cam1) [" << aruco_tfs_cam1.size() << "/" << sample_size << "]");
        ROS_INFO_STREAM("Averaging the pose of Aruco markers(cam2) [" << aruco_tfs_cam2.size() << "/" << sample_size << "]");

        if(aruco_tfs_cam1.size()>=sample_size && aruco_tfs_cam2.size()>=sample_size){
          // For cam1
          Eigen::Vector3d sum_translation = Eigen::Vector3d(0, 0, 0);
          for(int tf_i=0; tf_i<aruco_tfs_cam1.size(); tf_i++){
            sum_translation += aruco_tfs_cam1[tf_i].translation();  // For translation
            Eigen::Quaterniond quat(aruco_tfs_cam1[tf_i].linear()); // For rotation
            quaternions_cam1.push_back(Eigen::Vector4d(quat.w(), quat.x(), quat.y(), quat.z()));
          }
          Eigen::Vector3d trans_aver = sum_translation/sample_size;
          Eigen::Vector4d quat_aver_coef = quaternionAverage(quaternions_cam1);
          Eigen::Quaterniond quat_aver(quat_aver_coef[0], quat_aver_coef[1], quat_aver_coef[2], quat_aver_coef[3]);

          aruco_tf_cam1.linear() = quat_aver.toRotationMatrix();
          aruco_tf_cam1.translation() = trans_aver;

          // For cam2
          sum_translation = Eigen::Vector3d(0, 0, 0);
          for(int tf_i=0; tf_i<aruco_tfs_cam2.size(); tf_i++){
            sum_translation += aruco_tfs_cam2[tf_i].translation();  // For translation
            Eigen::Quaterniond quat(aruco_tfs_cam2[tf_i].linear()); // For rotation
            quaternions_cam2.push_back(Eigen::Vector4d( .w(), quat.x(), quat.y(), quat.z()));
          }
          trans_aver = sum_translation/sample_size;
          quat_aver_coef = quaternionAverage(quaternions_cam2);
          quat_aver = Eigen::Quaterniond(quat_aver_coef[0], quat_aver_coef[1], quat_aver_coef[2], quat_aver_coef[3]);

          aruco_tf_cam2.linear() = quat_aver.toRotationMatrix();
          aruco_tf_cam2.translation() = trans_aver;

          // Transform 3D pose to 2D pose
          geometry_msgs::Pose aruco_pose1, aruco_pose2;
          tf::poseEigenToMsg(aruco_tf_cam1, aruco_pose1);
          tf::poseEigenToMsg(aruco_tf_cam2, aruco_pose2);
          double yaw1 = tf::getYaw(aruco_pose1.orientation);
          double yaw2 = tf::getYaw(aruco_pose2.orientation);

          aruco_tf_cam1.linear() = Eigen::Matrix3d::Identity() * Eigen::AngleAxisd(yaw1, Eigen::Vector3d::UnitZ());
          aruco_tf_cam1.translation().z() = 1.4; //1.45; // The distance from camera along z-axis is constant
          aruco_tf_cam2.linear() = Eigen::Matrix3d::Identity() * Eigen::AngleAxisd(yaw2, Eigen::Vector3d::UnitZ());
          aruco_tf_cam2.translation().z() = 1.4; //1.45; // The distance from camera along z-axis is constant

          // Combine the poses derived from cam1 and cam2
          sum_translation = aruco_tf_cam1.translation() + aruco_tf_cam2.translation();  // For translation
          Eigen::Quaterniond quat_cam1(aruco_tf_cam1.linear()); // For rotation
          Eigen::Quaterniond quat_cam2(aruco_tf_cam2.linear()); // For rotation
          quaternions.push_back(Eigen::Vector4d(quat_cam1.w(), quat_cam1.x(), quat_cam1.y(), quat_cam1.z()));
          quaternions.push_back(Eigen::Vector4d(quat_cam2.w(), quat_cam2.x(), quat_cam2.y(), quat_cam2.z()));
          trans_aver = sum_translation/2;
          quat_aver_coef = quaternionAverage(quaternions);
          quat_aver = Eigen::Quaterniond(quat_aver_coef[0], quat_aver_coef[1], quat_aver_coef[2], quat_aver_coef[3]);

          aruco_tf.linear() = quat_aver.toRotationMatrix();
          aruco_tf.translation() = trans_aver;

          ROS_INFO_STREAM("The pose of Aruco marker is returned");
          return true;
        }
      }
      else{
        ROS_WARN_STREAM("Aruco marker is detected but difference between image sequence is large");
        image_seq_bf = image_seq_;
        aruco_tfs_cam1.clear();
        aruco_tfs_cam2.clear();
      }
    }
    else{
      ROS_WARN_STREAM("Wait for detecting Aruco marker");
      wait_count++;
      ros::WallDuration(0.5).sleep();
      if(wait_count >= 10){
        ROS_ERROR_STREAM("Failed to find ArUco marker");
        return false;
      }
    }
  }
}

// Search ArUco marker by moving robot arm
// 1) aruco marker pose w.r.t global frame 2) z value of marker pose w.r.t global frame
bool Manipulation::searchAruco(Eigen::Isometry3d& marker_pose, int32_t fiducial_id, double marker_z){
  moveit_msgs::RobotTrajectory derived_traj;
  std::vector<std::vector<double>> joint_targets; // robot configuration to search marker
  joint_targets.push_back(std::vector<double>{M_PI/180.0*-90.0, M_PI/180.0*-120.0, M_PI/180.0*-60.0, 0.0, M_PI/180.0*90.0, 0});
  joint_targets.push_back(std::vector<double>{M_PI/180.0*-90.0, M_PI/180.0*-150.0, M_PI/180.0*-30.0, 0.0, M_PI/180.0*90.0, 0});
  joint_targets.push_back(std::vector<double>{M_PI/180.0*-130.0, M_PI/180.0*-120.0, M_PI/180.0*-60.0, 0.0, M_PI/180.0*130.0, 0});
  joint_targets.push_back(std::vector<double>{M_PI/180.0*-50.0, M_PI/180.0*-120.0, M_PI/180.0*-60.0, 0.0, M_PI/180.0*50.0, 0});
  
  for(int pose_i=0; pose_i<joint_targets.size(); pose_i++){
    motion_.moveJoint(joint_targets[pose_i], motion_.getJMGPtr(rob1_group_name_), derived_traj);
    motion_.showRobotMotion(derived_traj);
    motion_.getVisualTools()->prompt("move seach pose[" + std::to_string(pose_i+1) + "/" + std::to_string(joint_targets.size()) + "]");
    motion_.updatePlanningScene();

    // moveGripperHW(rob1_group_name_, motion_.getGripperClose());
    // moveGripperHW(rob2_group_name_, motion_.getGripperClose());
    playHW(derived_traj);

    Eigen::Isometry3d aruco_tf;
    bool mark_success = detectAruco(aruco_tf, fiducial_id);
    if(mark_success){
      Eigen::Isometry3d global_frame = motion_.getRobotStatePtr()->getFrameTransform(global_link_);
      Eigen::Isometry3d cam_base_rel = Eigen::Isometry3d::Identity() * Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ());
      cam_base_rel.translation().x() = 0.023;
      cam_base_rel.translation().z() = 0.062;
      marker_pose = motion_.getRobotStatePtr()->getFrameTransform("rob1_ELP_CAM") * cam_base_rel * aruco_tf;
      marker_pose.translation().z() = marker_z; //0.85 // MODIFIED
      // motion_.getVisualTools()->publishAxisLabeled(motion_.getRobotStatePtr()->getFrameTransform("rob1_ELP_CAM") * cam_base_rel, "modified frame");
      // motion_.getVisualTools()->publishAxisLabeled(motion_.getRobotStatePtr()->getFrameTransform("rob1_ELP_CAM"), "CAM");
      // motion_.getVisualTools()->publishAxisLabeled(marker_pose, "object_pose_raw");
      // motion_.getVisualTools()->trigger();

      motion_.getVisualTools()->prompt("move home configuration");
      moveit_msgs::RobotTrajectory move_home_trj;
      motion_.moveHome(move_home_trj);
      moveGripperHW(rob1_group_name_, motion_.getGripperOpen());
      moveGripperHW(rob2_group_name_, motion_.getGripperOpen());
      playHW(move_home_trj);
      
      return true;
    }
  } 
  ROS_ERROR_STREAM("'searchAruco' is failed");
  return false;
}

// convert grasp pose of both robot arm w.r.t new reference frame
// 1)TF 2)new reference frame(relative) 3)output
void Manipulation::convertRefFrame(std::pair<Eigen::Isometry3d, Eigen::Isometry3d>& grasp_pose_both_origin, Eigen::Isometry3d& relative_pose
                                 , std::pair<Eigen::Isometry3d, Eigen::Isometry3d>& output){
  Eigen::Isometry3d grasp_pose_1, grasp_pose_2;
  convertRefFrame(grasp_pose_both_origin.first, relative_pose, grasp_pose_1);
  convertRefFrame(grasp_pose_both_origin.second, relative_pose, grasp_pose_2);
  output = std::make_pair(grasp_pose_1, grasp_pose_2);
}

// convert TF w.r.t new reference frame
// 1)TF 2)new reference frame(relative) 3)output
void Manipulation::convertRefFrame(Eigen::Isometry3d& transform_mat, Eigen::Isometry3d& relative_pose, Eigen::Isometry3d& output){
  output = relative_pose * transform_mat;
}

// convert vector w.r.t new reference frame
// 1)vector 2)new reference frame(relative) 3)output
void Manipulation::convertRefFrame(Eigen::Vector3d& vector, Eigen::Isometry3d& relative_pose, Eigen::Vector3d& output){
  output = relative_pose.linear()*vector + relative_pose.translation();
}

// convert vertice position of edge to new reference frame
// 1)vertice of edges 2)new reference frame(relative) 3)output
void Manipulation::convertRefFrame(std::vector<std::tuple<Eigen::Vector3d, Eigen::Vector3d>>& edges, Eigen::Isometry3d& relative_pose
                   , std::vector<std::tuple<Eigen::Vector3d, Eigen::Vector3d>>& output){
  Eigen::Vector3d vertex_1, vertex_2;
  output.resize(edges.size());
  for(int edge_i=0; edge_i<edges.size(); edge_i++){
    vertex_1 = std::get<0>(edges[edge_i]); 
    vertex_2 = std::get<1>(edges[edge_i]);

    vertex_1 = relative_pose.linear()*vertex_1 + relative_pose.translation();
    vertex_2 = relative_pose.linear()*vertex_2 + relative_pose.translation();
    output[edge_i] = std::make_tuple(vertex_1, vertex_2);
  }
}

// convert normal vector of triangle to new reference frame
// 1)both vertice and normal vector of edges 2)new reference frame(relative) 3)output
void Manipulation::convertRefFrame(std::vector<std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d>>& edges_normal, Eigen::Isometry3d& relative_pose
                   , std::vector<std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d>>& output){
  Eigen::Vector3d vertex_1, vertex_2, normal_vec;
  output.resize(edges_normal.size());
  for(int edge_i=0; edge_i<edges_normal.size(); edge_i++){
    vertex_1 = std::get<0>(edges_normal[edge_i]); 
    vertex_2 = std::get<1>(edges_normal[edge_i]);
    normal_vec = std::get<2>(edges_normal[edge_i]);

    vertex_1 = relative_pose.linear()*vertex_1 + relative_pose.translation();
    vertex_2 = relative_pose.linear()*vertex_2 + relative_pose.translation();
    normal_vec = relative_pose.linear()*normal_vec;
    output[edge_i] = std::make_tuple(vertex_1, vertex_2, normal_vec);
  }
}

// convert both vertice of rotation axis and rotation matrix to new reference frame
// 1)both vertice and rotation matrix of rotation axis 2)new reference frame(relative) 3)output
void Manipulation::convertRefFrame(std::vector<std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d>>& rotation_axis_mat, Eigen::Isometry3d& relative_pose
                   , std::vector<std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d>>& output){
  for(int axis_i=0; axis_i<rotation_axis_mat.size(); axis_i++){
    Eigen::Vector3d point_begin = relative_pose.linear()*std::get<0>(rotation_axis_mat[axis_i]) + relative_pose.translation();
    Eigen::Vector3d point_end = relative_pose.linear()*std::get<1>(rotation_axis_mat[axis_i]) + relative_pose.translation();

    Eigen::Isometry3d object_pose_axis = Eigen::Isometry3d::Identity(); // object pose which the start point of rotation axis coincide with global frame
    object_pose_axis.translation() = -point_begin; // coincide the origin of object with the initial point of rotation axis
    Eigen::Vector3d rotation_axis = (point_end - point_begin).normalized(); // move point according to above process. And it will be vector of rotation axis
    Eigen::Vector4d quat_coef(cos(rotation_angle_step_/2), (rotation_axis*sin(rotation_angle_step_/2))[0]
                            , (rotation_axis*sin(rotation_angle_step_/2))[1], (rotation_axis*sin(rotation_angle_step_/2))[2]); // w, x, y, z
    quat_coef.normalize();

    Eigen::Quaterniond rotation_quat = Eigen::Quaterniond(quat_coef[0], quat_coef[1], quat_coef[2], quat_coef[3]); // quaternion according to the rotation axis
    output.push_back(std::make_tuple(point_begin, point_end, rotation_quat.normalized().toRotationMatrix()));
  }
}

// derive maximum rotation amount according to the rotation axis
// 1)one rotation axis & matrix(w.r.t global) 2)object pose 3)rotation axis(both vertice) / max step of rotation 4)final rotated object pose
bool Manipulation::getMaxRotAmount(std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d>& rotation_axis_mat, Eigen::Isometry3d& object_pose
                                 , std::tuple<Eigen::Vector3d, Eigen::Vector3d, int>& rotation_axis_step, Eigen::Isometry3d& rotated_pose){
  std::tuple<Eigen::Vector3d, Eigen::Vector3d> rotation_axis;
  rotation_axis = std::make_tuple(std::get<0>(rotation_axis_mat), std::get<1>(rotation_axis_mat));

  bool result = getMaxRotAmount(rotation_axis, object_pose, rotation_axis_step, rotated_pose);
  rotation_axis_mat = std::make_tuple(std::get<0>(rotation_axis), std::get<1>(rotation_axis), std::get<2>(rotation_axis_mat));
  
  return result;
}

// derive maximum rotation amount according to the rotation axis
// 1)one rotation axis(w.r.t global) 2)object pose 3)rotation axis(both vertice) / max step of rotation 4)final rotated object pose
bool Manipulation::getMaxRotAmount(std::tuple<Eigen::Vector3d, Eigen::Vector3d>& rotation_axis, Eigen::Isometry3d& object_pose
                                 , std::tuple<Eigen::Vector3d, Eigen::Vector3d, int>& rotation_axis_step, Eigen::Isometry3d& rotated_pose){
  int step_i = 0;
  int limit_angle_step = static_cast<int>(3.141592 / rotation_angle_step_); // limitation of theta in Quaternion is from 0 to pi
  moveit_msgs::CollisionObject target_object;
  std::vector<std::string> collision_objects = motion_.getPlanningScenePtr()->getWorld()->getObjectIds();

  motion_.getVisualTools()->publishArrow(tf2::toMsg(std::get<0>(rotation_axis)), tf2::toMsg(std::get<1>(rotation_axis))
                                       , rviz_visual_tools::colors::LIME_GREEN, rviz_visual_tools::scales::SMALL);
  motion_.getVisualTools()->trigger();
  
  for(step_i=0; step_i<=limit_angle_step; step_i++){
    motion_.rotateObj(rotation_axis, step_i, object_pose, rotated_pose);
    grasp_.genMeshObj(target_object_file_, rotated_pose, target_object, false);

    // check validity
    if(!motion_.checkStateValid(true)){ // true: valid
      ROS_INFO_STREAM("Maximum of rotation amount is derived based on checkStateValid()");
      break;
    }

    // check collision(between collision objects)
    bool break_for = false;
    for(int col_i=0; col_i<collision_objects.size(); col_i++){
      if(target_object.id != collision_objects[col_i]){
        if(motion_.checkCollisionObj(target_object.id, collision_objects[col_i])){ // true: collision occur
          ROS_INFO_STREAM("Maximum of rotation amount is derived based on checkCollisionObj()");
          break_for = true;
          break;
        }
      }
    }
    if(break_for) break;
  }

  rotation_axis_step = std::make_tuple(std::get<0>(rotation_axis), std::get<1>(rotation_axis), step_i-1); // step_i: step of collision occur
  
  // // show rotated object & initial object
  // if(std::get<2>(rotation_axis_step)>5){ // if step is not small
  //   motion_.rotateObj(rotation_axis, std::get<2>(rotation_axis_step), object_pose, rotated_pose);
  //   grasp_.genMeshObj(target_object_file_, rotated_pose, target_object); // last valid pose of rotated object
  //   for(int i=0; i<std::get<2>(rotation_axis_step); i++){
  //     motion_.rotateObj(rotation_axis, i, object_pose, rotated_pose);
  //     motion_.genMeshObjTrans(target_object_file_, rotated_pose, i, 0.05);
  //     motion_.getVisualTools()->publishAxis(rotated_pose);
  //   }
  //   motion_.getVisualTools()->trigger();
  //   motion_.getVisualTools()->prompt("show rotated object step"); 
  //   for(int i=0; i<std::get<2>(rotation_axis_step); i++)  motion_.clearObjWorld(target_object_file_ + "_" + std::to_string(i));
  // }
  
  motion_.getVisualTools()->deleteAllMarkers();
  motion_.getVisualTools()->trigger();

  grasp_.genMeshObj(target_object_file_, object_pose, target_object); // retrieve original object 

  return true;
}

// prioritizing the rotation axis
// 1)rotation axis + max rotation step 2)output
bool Manipulation::priorRotAxis(std::vector<std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d, int>>& rotation_axes_step
                              , std::vector<std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d, int>>& rotation_axis_cands){
  // scoring the rotation axis 
  std::multimap<double, int, std::greater<double>> rotation_axis_index; // decending order according to the score
  double rotation_axis_length, max_rotation_angle, score;
  for(int axis_i=0; axis_i<rotation_axes_step.size(); axis_i++){
    rotation_axis_length = (std::get<0>(rotation_axes_step[axis_i]) - std::get<1>(rotation_axes_step[axis_i])).norm(); // length of rotation axis
    max_rotation_angle = rotation_angle_step_ * static_cast<double>(std::get<3>(rotation_axes_step[axis_i]));
    score = weight_length_*rotation_axis_length + weight_angle_*max_rotation_angle;
    rotation_axis_index.insert(std::make_pair(score, axis_i)); // store the index of rotation axis and sort it as decending order
  }

  for(auto iter=rotation_axis_index.begin(); iter!=rotation_axis_index.end(); iter++){
    rotation_axis_cands.push_back(std::make_tuple(std::get<0>(rotation_axes_step[iter->second]), std::get<1>(rotation_axes_step[iter->second])
                                , std::get<2>(rotation_axes_step[iter->second]), std::get<3>(rotation_axes_step[iter->second]))); // iter->second: index of rotation_axes_step
  }

  return true;
}

// Load grasp pose & support grasp pose from yaml(w.r.t object frame)
// 1)grasp pose of rob1/rob2 2) support grasp pose of rob1/rob2
bool Manipulation::loadGraspPose(std::pair<Eigen::Isometry3d, Eigen::Isometry3d>& grasp_pose_both
                               , std::pair<Eigen::Isometry3d, Eigen::Isometry3d>& sup_grasp_pose_both){
  Eigen::Isometry3d rob1_grasp_pose_origin, rob2_grasp_pose_origin, rob1_sup_grasp_pose_origin, rob2_sup_grasp_pose_origin; // grasp pose w.r.t object frame
  for(int i=0; i<3; i++){
    for(int j=0; j<3; j++){
      rob1_grasp_pose_origin.linear()(i,j) = rob1_grasp_rot_[3*i+j];
      rob2_grasp_pose_origin.linear()(i,j) = rob2_grasp_rot_[3*i+j];
      rob1_sup_grasp_pose_origin.linear()(i,j) = rob1_sup_grasp_rot_[3*i+j];
      rob2_sup_grasp_pose_origin.linear()(i,j) = rob2_sup_grasp_rot_[3*i+j];
    }
  }
  rob1_grasp_pose_origin.translation() = Eigen::Vector3d(rob1_grasp_trans_[0], rob1_grasp_trans_[1], rob1_grasp_trans_[2]);
  rob2_grasp_pose_origin.translation() = Eigen::Vector3d(rob2_grasp_trans_[0], rob2_grasp_trans_[1], rob2_grasp_trans_[2]);
  rob1_sup_grasp_pose_origin.translation() = Eigen::Vector3d(rob1_sup_grasp_trans_[0], rob1_sup_grasp_trans_[1], rob1_sup_grasp_trans_[2]);
  rob2_sup_grasp_pose_origin.translation() = Eigen::Vector3d(rob2_sup_grasp_trans_[0], rob2_sup_grasp_trans_[1], rob2_sup_grasp_trans_[2]);

  grasp_pose_both = std::make_pair(rob1_grasp_pose_origin, rob2_grasp_pose_origin);
  sup_grasp_pose_both = std::make_pair(rob1_sup_grasp_pose_origin, rob2_sup_grasp_pose_origin);
  
  return true;
}

// Set pregrasp pose
// 1)grasp pose or support grasp pose 2)output
bool Manipulation::setPregraspPose(Eigen::Isometry3d& grasp_pose, Eigen::Isometry3d& pregrasp_pose){
  Eigen::Isometry3d pregrasp_pose_rel = Eigen::Isometry3d::Identity();
  pregrasp_pose_rel.translation() = Eigen::Vector3d(0, 0, rob1_pregrasp_trans_);

  pregrasp_pose = grasp_pose * pregrasp_pose_rel;
 
  return true;
}

// Set pregrasp pose
// 1)grasp pose or support grasp pose 2)output
bool Manipulation::setPregraspPose(std::pair<Eigen::Isometry3d, Eigen::Isometry3d>& grasp_pose_both
                                 , std::pair<Eigen::Isometry3d, Eigen::Isometry3d>& pregrasp_pose_both){
  Eigen::Isometry3d pregrasp_pose_rel_rob1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d pregrasp_pose_rel_rob2 = Eigen::Isometry3d::Identity();
  pregrasp_pose_rel_rob1.translation() = Eigen::Vector3d(0, 0, rob1_pregrasp_trans_);
  pregrasp_pose_rel_rob2.translation() = Eigen::Vector3d(0, 0, rob2_pregrasp_trans_);

  Eigen::Isometry3d pregrasp_pose_rob1 = grasp_pose_both.first * pregrasp_pose_rel_rob1;
  Eigen::Isometry3d pregrasp_pose_rob2 = grasp_pose_both.second * pregrasp_pose_rel_rob2;

  pregrasp_pose_both = std::make_pair(pregrasp_pose_rob1, pregrasp_pose_rob2);
  
  return true;
}

// Calculate distance between rotation axis & grasp pose. And delete rotation axis candidates if distance is short
// 1)both grasp poses 2)rotation axis candidates 3)rotation axis candidates considering grasp point
bool Manipulation::getDisRotAxis(std::pair<Eigen::Isometry3d, Eigen::Isometry3d>& grasp_pose_both
                               , std::vector<std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d, int>>& rotation_axis_cands
                               , std::vector<std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d, int>>& rotation_axis_dist_cands){
  std::vector<std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d, int>> rotation_axis_dist_cands_tmp;
  getDisRotAxis(grasp_pose_both.first, rotation_axis_cands, rotation_axis_dist_cands_tmp);
  getDisRotAxis(grasp_pose_both.second, rotation_axis_dist_cands_tmp, rotation_axis_dist_cands);
  
  return true;
}

// Calculate distance between rotation axis & grasp pose. And delete rotation axis candidates if distance is short
// 1)grasp pose 2)rotation axis candidates 3)rotation axis candidates considering grasp point
bool Manipulation::getDisRotAxis(Eigen::Isometry3d grasp_pose
                               , std::vector<std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d, int>>& rotation_axis_cands
                               , std::vector<std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d, int>>& rotation_axis_dist_cands){
  Eigen::Vector3d grasp_point, point_start, point_end;
  for(int axis_i=0; axis_i<rotation_axis_cands.size(); axis_i++){
    grasp_point = grasp_pose.translation();
    point_start = std::get<0>(rotation_axis_cands[axis_i]);
    point_end = std::get<1>(rotation_axis_cands[axis_i]);

    // coincide the reference point with 'point_end'
    grasp_point = grasp_point - point_start;
    point_end = point_end - point_start;
    point_start = Eigen::Vector3d::Zero();

    double distance = (grasp_point - grasp_point.dot(point_end.normalized())*point_end.normalized()).norm();

    // if distance between rotation aixs & grasp point is not close, add rotation axis candidates
    if(distance > distance_axis_grasp_)  rotation_axis_dist_cands.push_back(rotation_axis_cands[axis_i]);
  }
  
  // // show grasp point & rotation axis
  // for(int axis_i=0; axis_i<rotation_axis_dist_cands.size(); axis_i++){
  //   motion_.getVisualTools()->publishArrow(tf2::toMsg(std::get<0>(rotation_axis_dist_cands[axis_i])), tf2::toMsg(std::get<1>(rotation_axis_dist_cands[axis_i]))
  //                                        , rviz_visual_tools::colors::LIME_GREEN, rviz_visual_tools::scales::SMALL);
  //   motion_.getVisualTools()->publishSphere(grasp_pose, rviz_visual_tools::colors::MAGENTA, rviz_visual_tools::scales::XLARGE);
  //   // motion_.getVisualTools()->publishArrow(geometry_msgs::Point(), Eigen::toMsg(Eigen::Vector3d(grasp_pose.translation())), rviz_visual_tools::colors::CYAN, rviz_visual_tools::scales::SMALL);
  //   // motion_.getVisualTools()->publishArrow(geometry_msgs::Point(), tf2::toMsg(std::get<0>(rotation_axis_dist_cands[axis_i])), rviz_visual_tools::colors::CYAN, rviz_visual_tools::scales::SMALL);
  //   // motion_.getVisualTools()->publishArrow(geometry_msgs::Point(), tf2::toMsg(std::get<1>(rotation_axis_dist_cands[axis_i])), rviz_visual_tools::colors::CYAN, rviz_visual_tools::scales::SMALL);
  //   motion_.getVisualTools()->trigger();
  //   motion_.getVisualTools()->prompt("distance test");
  //   motion_.getVisualTools()->deleteAllMarkers();
  //   motion_.getVisualTools()->trigger();
  // }
  // motion_.getVisualTools()->prompt("distance test");
  // motion_.getVisualTools()->deleteAllMarkers();
  // motion_.getVisualTools()->trigger();

  return true;
}

// [Separaing the method to get object pose depending on the orientation of grasping pose]
// Rotate object with dummy gripper(open/closed) and find object pose(angle) where the gripper does not collide
// 1)target object 2)grasp pose w.r.t object frame 3)pregrasp pose w.r.t. object frame 4)rotation axis candidates considering grasp point 
// 5)rotation axis candidates and object pose considering grasp point & dummy gripper
bool Manipulation::getRotObjPose(moveit_msgs::CollisionObject& target_object, std::pair<Eigen::Isometry3d, Eigen::Isometry3d>& grasp_pose
                               , std::pair<Eigen::Isometry3d, Eigen::Isometry3d>& pregrasp_pose
                               , std::vector<std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d, int>>& rotation_axis_dist_cands
                               , std::vector<std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d, int, Eigen::Isometry3d>>& rotaxis_objpose_cands){
  Eigen::Isometry3d object_pose;
  tf2::fromMsg(target_object.mesh_poses.front(), object_pose);
  Eigen::Vector3d grasp_pose_z_first = (object_pose * grasp_pose.first).linear().col(2); // the z-axis of the grasping pose w.r.t global frame
  Eigen::Vector3d grasp_pose_z_second = (object_pose * grasp_pose.second).linear().col(2); // the z-axis of the grasping pose w.r.t global frame

  // Show z-axis for each grasping pose
  // geometry_msgs::Point pose_first, pose_second, pose_first_arrow, pose_second_arrow;
  // Eigen::Vector3d vec_tmp1 = grasp_pose_first.translation();
  // Eigen::Vector3d vec_tmp2 = grasp_pose_second.translation();
  // Eigen::Vector3d vec_tmp3 = grasp_pose_first.linear().col(2)+grasp_pose_first.translation();
  // Eigen::Vector3d vec_tmp4 = grasp_pose_second.linear().col(2)+grasp_pose_second.translation();
  // pose_first = Eigen::toMsg(vec_tmp1);
  // pose_second = Eigen::toMsg(vec_tmp2);
  // pose_first_arrow = Eigen::toMsg(vec_tmp3);
  // pose_second_arrow = Eigen::toMsg(vec_tmp4);
  // motion_.getVisualTools()->publishArrow(pose_first, pose_first_arrow);
  // motion_.getVisualTools()->publishArrow(pose_second, pose_second_arrow);
  // motion_.getVisualTools()->trigger();
  // ROS_WARN_STREAM("grasp_pose_first.linear().col(2): \n" << grasp_pose_first.linear().col(2)); // z-axis vector
  // ROS_WARN_STREAM("grasp_pose_second.linear().col(2): \n" << grasp_pose_second.linear().col(2)); // z-axis vector
  // motion_.getVisualTools()->prompt("z-axis test");

  Eigen::Vector3d z_axis_global = motion_.getRobotStatePtr()->getFrameTransform(global_link_).linear().col(2); // vector of z-axis of the global frame
  double theta_z_first = acos(grasp_pose_z_first.dot(z_axis_global) / (grasp_pose_z_first.norm()*z_axis_global.norm())); // angle between the z-axis of the grasping pose and that of the global frame
  double theta_z_second = acos(grasp_pose_z_second.dot(z_axis_global) / (grasp_pose_z_second.norm()*z_axis_global.norm())); // angle between the z-axis of the grasping pose and that of the global frame
  
  std::string z_axis_first, z_axis_second;
  if(theta_z_first < M_PI/2)  z_axis_first = "up";
  else                        z_axis_first = "down";

  if(theta_z_second < M_PI/2) z_axis_second = "up";
  else                        z_axis_second = "down";
  
  bool get_pose = false;
  if(z_axis_first=="down" && z_axis_second=="down"){
    ROS_INFO_STREAM("The z-axis of the graping poses: downward / downward");
    get_pose = downGraspPose(target_object, grasp_pose, rotation_axis_dist_cands, rotaxis_objpose_cands);
  }
  else if(z_axis_first=="up" && z_axis_second=="up"){
    ROS_INFO_STREAM("The z-axis of the graping poses: upward / upward");
    get_pose = upGraspPose(target_object, grasp_pose, pregrasp_pose, rotation_axis_dist_cands, rotaxis_objpose_cands);
  }
  else{ // one is up, the other is down
    ROS_INFO_STREAM("The z-axis of the graping poses: mixed");
    get_pose = mixGraspPose(target_object, grasp_pose, pregrasp_pose, {z_axis_first, z_axis_second}, rotation_axis_dist_cands, rotaxis_objpose_cands);
  }
  
  return get_pose;
}

// [Both grasping poses are downward]
// Rotate object with dummy gripper(open/closed) and find object pose(angle) where the gripper does not collide
// 1)target object 2)grasp pose w.r.t object frame 3)rotation axis candidates considering grasp point 4)rotation axis candidates and object pose considering grasp point & dummy gripper
bool Manipulation::downGraspPose(moveit_msgs::CollisionObject& target_object, std::pair<Eigen::Isometry3d, Eigen::Isometry3d>& grasp_pose
                             , std::vector<std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d, int>>& rotation_axis_dist_cands
                             , std::vector<std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d, int, Eigen::Isometry3d>>& rotaxis_objpose_cands){
  int max_rot_step, extra_rot_step;
  Eigen::Matrix3d rot_mat;
  Eigen::Vector3d point_start, point_end;
  Eigen::Isometry3d object_pose, rotated_pose;
  tf2::fromMsg(target_object.mesh_poses.front(), object_pose); // first:rob1 second:rob2
  moveit_msgs::CollisionObject dummy_gripper1, dummy_gripper2;

  // extra rotation step for safety
  if(target_object_file_ == "chair_side_part")        extra_rot_step = 3;
  else if(target_object_file_ == "shelf_side_part")   extra_rot_step = 3;
  else if(target_object_file_ == "stool_side_part")   extra_rot_step = 3;

  for(int axis_i=0; axis_i<rotation_axis_dist_cands.size(); axis_i++){
    max_rot_step = std::get<3>(rotation_axis_dist_cands[axis_i]);
    rot_mat = std::get<2>(rotation_axis_dist_cands[axis_i]);
    point_start = std::get<0>(rotation_axis_dist_cands[axis_i]);
    point_end = std::get<1>(rotation_axis_dist_cands[axis_i]);

    motion_.getVisualTools()->publishArrow(tf2::toMsg(point_start), tf2::toMsg(point_end), rviz_visual_tools::colors::LIME_GREEN, rviz_visual_tools::scales::SMALL);
    motion_.getVisualTools()->trigger();

    bool break_for_step = false;
    for(int step_i=0; step_i<=max_rot_step; step_i++){
      bool not_col_open = true; // true if collision of dummy gripper(open) is not occur
      bool not_col_closed = true; // true if collision of dummy gripper(closed) is not occur

      std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d> rotation_axis_mat;
      rotation_axis_mat = std::make_tuple(point_start, point_end, rot_mat);
      motion_.rotateObj(rotation_axis_mat, step_i, object_pose, rotated_pose);
      grasp_.genMeshObj(target_object_file_, rotated_pose, target_object, false);

      // Check collsion of open dummy gripper
      motion_.genDummyGripper(dummy_gripper1, "dummy_gripper1", "open", grasp_pose.first, rotated_pose, rob1_power_point_, false);
      motion_.genDummyGripper(dummy_gripper2, "dummy_gripper2", "open", grasp_pose.second, rotated_pose, rob2_power_point_, false);
      std::vector<std::string> collision_objects = motion_.getPlanningScenePtr()->getWorld()->getObjectIds();

      // check validity 
      if(motion_.checkStateValid(true)){ // true: valid
        // check collision(between collision objects) 
        for(int col_i=0; col_i<collision_objects.size(); col_i++){
          if(target_object_file_ != collision_objects[col_i]){
            if(!motion_.checkCollisionObj(dummy_gripper1.id, collision_objects[col_i]) // true: collision occur
               && !motion_.checkCollisionObj(dummy_gripper2.id, collision_objects[col_i])){
              not_col_open = not_col_open*true;
            }
            else{
              not_col_open = false;
              break;
            }
          }
        }
      }
      else  not_col_open = false;

      // when open dummy gripper does not collied with anything, do collision check w.r.t closed dummy gripper
      if(not_col_open){
        motion_.genDummyGripper(dummy_gripper1, "dummy_gripper1", "closed", grasp_pose.first, rotated_pose, rob1_power_point_, false);
        motion_.genDummyGripper(dummy_gripper2, "dummy_gripper2", "closed", grasp_pose.second, rotated_pose, rob2_power_point_, false);
        collision_objects.clear();
        collision_objects = motion_.getPlanningScenePtr()->getWorld()->getObjectIds();

        // check validity 
        if(motion_.checkStateValid(true)){ // true: valid
          // check collision(between collision objects) 
          for(int col_i=0; col_i<collision_objects.size(); col_i++){
            if(target_object_file_ != collision_objects[col_i]){
              if(!motion_.checkCollisionObj(dummy_gripper1.id, collision_objects[col_i]) // true: collision occur
                 && !motion_.checkCollisionObj(dummy_gripper2.id, collision_objects[col_i])){
                not_col_closed = not_col_closed*true;
              }
              else{
                not_col_closed = false;
                break;
              }
            }
          }
        }
        else  not_col_closed = false;
      }

      // when dummy grippers do not collide with anything
      if(not_col_open*not_col_closed){
        // rotate object & dummy grippers according to extra step for safety
        bool success = true; // success to get object pose(angle) considering dummy gripper
        bool break_for_extra = false;
        int extra_i;
        for(extra_i=1; extra_i<=extra_rot_step; extra_i++){
          motion_.rotateObj(rotation_axis_mat, step_i+extra_i, object_pose, rotated_pose);
          grasp_.genMeshObj(target_object_file_, rotated_pose, target_object, false);
          motion_.genDummyGripper(dummy_gripper1, "dummy_gripper1", "closed", grasp_pose.first, rotated_pose, rob1_power_point_, false);
          motion_.genDummyGripper(dummy_gripper2, "dummy_gripper2", "closed", grasp_pose.second, rotated_pose, rob2_power_point_, false);
          collision_objects.clear();
          collision_objects = motion_.getPlanningScenePtr()->getWorld()->getObjectIds();

          // check validity
          if(!motion_.checkStateValid(true)){ // true: valid
            ROS_INFO_STREAM("Invalid state is occur when rotating the object additionally");
            break_for_extra = true;
            break_for_step = true;
            success = false;
            break;
          }

          // check collision(between collision objects)
          for(int col_i=0; col_i<collision_objects.size(); col_i++){
            if(target_object_file_ != collision_objects[col_i]){
              if(motion_.checkCollisionObj(dummy_gripper1.id, collision_objects[col_i]) // true: collision occur
                 || motion_.checkCollisionObj(dummy_gripper2.id, collision_objects[col_i])){ 
                ROS_INFO_STREAM("Collsion occur when rotatin the object additionally");
                break_for_extra = true;
                break_for_step = true;
                success = false;
                break;
              }
            }
          }

          if(break_for_extra)   break;
        }

        if(success){
          ROS_INFO_STREAM("Object pose(angle) is derived considering the dummy gripper");
          std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d, int, Eigen::Isometry3d> rotaxis_objpose_cand;
          rotaxis_objpose_cand = std::make_tuple(point_start, point_end, rot_mat, step_i+extra_i-1, rotated_pose); // -1: when exiting above for statement with valid value, 'extra_i' is added by 1 from true value
          rotaxis_objpose_cands.push_back(rotaxis_objpose_cand);
          break_for_step = true;

          // // show rotated object & trail
          // motion_.clearObjWorld("dummy_gripper1", false);
          // motion_.clearObjWorld("dummy_gripper2", true);
          // for(int i=0; i<std::get<3>(rotaxis_objpose_cands.back()); i++){
          //   motion_.rotateObj(rotation_axis_mat, i, object_pose, rotated_pose);
          //   motion_.genMeshObjTrans(target_object_file_, rotated_pose, i);
          // }
          // motion_.getVisualTools()->prompt("show rotated object step"); 
          // for(int i=0; i<std::get<3>(rotaxis_objpose_cands.back()); i++)  motion_.clearObjWorld(target_object_file_ + "_" + std::to_string(i));

          // // show rotated object & dummy grippers
          // for(int i=0; i<std::get<3>(rotaxis_objpose_cands.back()); i++){
          //   motion_.rotateObj(rotation_axis_mat, i, object_pose, rotated_pose);
          //   motion_.genMeshObjTrans(target_object_file_, rotated_pose, i);
          //   motion_.genDummyGripperTrans(dummy_gripper1, "dummy_gripper1", "closed", grasp_pose.first, rotated_pose, rob1_power_point_, i);
          //   motion_.genDummyGripperTrans(dummy_gripper2, "dummy_gripper2", "closed", grasp_pose.second, rotated_pose, rob2_power_point_, i);
          // }
          // motion_.getVisualTools()->prompt("show rotated object step"); 
          // for(int i=0; i<std::get<3>(rotaxis_objpose_cands.back()); i++){
          //   motion_.clearObjWorld(target_object_file_ + "_" + std::to_string(i));
          //   motion_.clearObjWorld("dummy_gripper1_" + std::to_string(i), false);
          //   motion_.clearObjWorld("dummy_gripper2_" + std::to_string(i), true);
          // }
        }
      }
      
      if(break_for_step)  break;
    }

    motion_.getVisualTools()->deleteAllMarkers();
    motion_.getVisualTools()->trigger();
  }

  grasp_.genMeshObj(target_object_file_, object_pose, target_object, false); // retrieve original object 
  motion_.clearObjWorld("dummy_gripper1", false);
  motion_.clearObjWorld("dummy_gripper2", true);

  return true;
}

// [Both grasping poses are upward]
// Rotate object with dummy gripper(open/closed) and find object pose(angle) where the gripper does not collide
// 1)target object 2)grasp pose w.r.t object frame 3)pregrasp pose w.r.t. object frame 4)rotation axis candidates considering grasp point 
// 5)rotation axis candidates and object pose considering grasp point & dummy gripper
bool Manipulation::upGraspPose(moveit_msgs::CollisionObject& target_object, std::pair<Eigen::Isometry3d, Eigen::Isometry3d>& grasp_pose
                             , std::pair<Eigen::Isometry3d, Eigen::Isometry3d>& pregrasp_pose
                             , std::vector<std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d, int>>& rotation_axis_dist_cands
                             , std::vector<std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d, int, Eigen::Isometry3d>>& rotaxis_objpose_cands){
  int max_rot_step, extra_rot_step;
  Eigen::Matrix3d rot_mat;
  Eigen::Vector3d point_start, point_end;
  Eigen::Isometry3d object_pose, rotated_pose;
  tf2::fromMsg(target_object.mesh_poses.front(), object_pose); // first:rob1 second:rob2
  moveit_msgs::CollisionObject dummy_gripper1, dummy_gripper2;

  // extra rotation step for safety
  if(target_object_file_ == "chair_side_part")        extra_rot_step = 7;
  else if(target_object_file_ == "shelf_side_part")   extra_rot_step = 7;
  else if(target_object_file_ == "stool_side_part")   extra_rot_step = 5;

  for(int axis_i=0; axis_i<rotation_axis_dist_cands.size(); axis_i++){
    max_rot_step = std::get<3>(rotation_axis_dist_cands[axis_i]);
    rot_mat = std::get<2>(rotation_axis_dist_cands[axis_i]);
    point_start = std::get<0>(rotation_axis_dist_cands[axis_i]);
    point_end = std::get<1>(rotation_axis_dist_cands[axis_i]);

    motion_.getVisualTools()->publishArrow(tf2::toMsg(point_start), tf2::toMsg(point_end), rviz_visual_tools::colors::LIME_GREEN, rviz_visual_tools::scales::SMALL);
    motion_.getVisualTools()->trigger();

    bool break_for_step = false;
    for(int step_i=0; step_i<=max_rot_step; step_i++){
      bool not_col_open = true; // true if collision of dummy gripper(open) is not occur
      bool not_col_closed = true; // true if collision of dummy gripper(closed) is not occur
      std::vector<std::string> collision_objects;

      std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d> rotation_axis_mat;
      rotation_axis_mat = std::make_tuple(point_start, point_end, rot_mat);
      motion_.rotateObj(rotation_axis_mat, step_i, object_pose, rotated_pose);
      grasp_.genMeshObj(target_object_file_, rotated_pose, target_object, false);

      // First for grasping pose, second for pregrasp pose
      for(int dummy_i=0; dummy_i<2; dummy_i++){
        // Check collsion of open dummy gripper
        if(dummy_i==0){
          motion_.genDummyGripper(dummy_gripper1, "dummy_gripper1", "open", grasp_pose.first, rotated_pose, rob1_power_point_, false);
          motion_.genDummyGripper(dummy_gripper2, "dummy_gripper2", "open", grasp_pose.second, rotated_pose, rob2_power_point_, false);
        }
        else{
          motion_.genDummyGripper(dummy_gripper1, "dummy_gripper1", "open", pregrasp_pose.first, rotated_pose, rob1_power_point_, false);
          motion_.genDummyGripper(dummy_gripper2, "dummy_gripper2", "open", pregrasp_pose.second, rotated_pose, rob2_power_point_, false);
        }
        collision_objects.clear();
        collision_objects = motion_.getPlanningScenePtr()->getWorld()->getObjectIds();

        // check validity 
        if(motion_.checkStateValid(true)){ // true: valid
          // check collision(between collision objects) 
          for(int col_i=0; col_i<collision_objects.size(); col_i++){
            if(target_object_file_ != collision_objects[col_i]){
              if(!motion_.checkCollisionObj(dummy_gripper1.id, collision_objects[col_i]) // true: collision occur
                 && !motion_.checkCollisionObj(dummy_gripper2.id, collision_objects[col_i])){
                not_col_open = not_col_open*true;
              }
              else{
                not_col_open = false;
                break;
              }
            }
          }
        }
        else  not_col_open = false;

        // when open dummy gripper does not collied with anything, do collision check w.r.t closed dummy gripper
        if(not_col_open){
          if(dummy_i==0){
            motion_.genDummyGripper(dummy_gripper1, "dummy_gripper1", "closed", grasp_pose.first, rotated_pose, rob1_power_point_, false);
            motion_.genDummyGripper(dummy_gripper2, "dummy_gripper2", "closed", grasp_pose.second, rotated_pose, rob2_power_point_, false);
          }
          else{
            motion_.genDummyGripper(dummy_gripper1, "dummy_gripper1", "closed", pregrasp_pose.first, rotated_pose, rob1_power_point_, false);
            motion_.genDummyGripper(dummy_gripper2, "dummy_gripper2", "closed", pregrasp_pose.second, rotated_pose, rob2_power_point_, false);
          }
          collision_objects.clear();
          collision_objects = motion_.getPlanningScenePtr()->getWorld()->getObjectIds();

          // check validity 
          if(motion_.checkStateValid(true)){ // true: valid
            // check collision(between collision objects) 
            for(int col_i=0; col_i<collision_objects.size(); col_i++){
              if(target_object_file_ != collision_objects[col_i]){
                if(!motion_.checkCollisionObj(dummy_gripper1.id, collision_objects[col_i]) // true: collision occur
                   && !motion_.checkCollisionObj(dummy_gripper2.id, collision_objects[col_i])){
                  not_col_closed = not_col_closed*true;
                }
                else{
                  not_col_closed = false;
                  break;
                }
              }
            }
          }
          else  not_col_closed = false;
        }
        else  break; // open dummy gripper collides with something
      }

      // when dummy grippers do not collide with anything
      if(not_col_open*not_col_closed){
        // rotate object & dummy grippers according to extra step for safety
        bool success = true; // success to get object pose(angle) considering dummy gripper
        bool break_for_extra = false;
        bool break_for_dummy = false;
        int extra_i;
        for(extra_i=1; extra_i<=extra_rot_step; extra_i++){
          // First for grasping pose, second for pregrasp pose
          for(int dummy_i=0; dummy_i<2; dummy_i++){
            motion_.rotateObj(rotation_axis_mat, step_i+extra_i, object_pose, rotated_pose);
            grasp_.genMeshObj(target_object_file_, rotated_pose, target_object, false);
            if(dummy_i==0){
              motion_.genDummyGripper(dummy_gripper1, "dummy_gripper1", "open", grasp_pose.first, rotated_pose, rob1_power_point_, false);
              motion_.genDummyGripper(dummy_gripper2, "dummy_gripper2", "open", grasp_pose.second, rotated_pose, rob2_power_point_, false);
            }
            else{
              motion_.genDummyGripper(dummy_gripper1, "dummy_gripper1", "closed", pregrasp_pose.first, rotated_pose, rob1_power_point_, false);
              motion_.genDummyGripper(dummy_gripper2, "dummy_gripper2", "closed", pregrasp_pose.second, rotated_pose, rob2_power_point_, false);
            }
            collision_objects.clear();
            collision_objects = motion_.getPlanningScenePtr()->getWorld()->getObjectIds();

            // check validity
            if(!motion_.checkStateValid(true)){ // true: valid
              ROS_INFO_STREAM("Invalid state is occur when rotating the object additionally");
              break_for_extra = true;
              break_for_step = true;
              break_for_dummy = true;
              success = false;
              break;
            }

            // check collision(between collision objects)
            for(int col_i=0; col_i<collision_objects.size(); col_i++){
              if(target_object_file_ != collision_objects[col_i]){
                if(motion_.checkCollisionObj(dummy_gripper1.id, collision_objects[col_i]) // true: collision occur
                  || motion_.checkCollisionObj(dummy_gripper2.id, collision_objects[col_i])){ 
                  ROS_INFO_STREAM("Collsion occur when rotatin the object additionally");
                  break_for_extra = true;
                  break_for_step = true;
                  break_for_dummy = true;
                  success = false;
                  break;
                }
              }
            }

            if(break_for_dummy) break;
          }

          if(break_for_extra)   break;
        }

        if(success){
          ROS_INFO_STREAM("Object pose(angle) is derived considering the dummy gripper");
          std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d, int, Eigen::Isometry3d> rotaxis_objpose_cand;
          rotaxis_objpose_cand = std::make_tuple(point_start, point_end, rot_mat, step_i+extra_i-1, rotated_pose); // -1: when exiting above for statement with valid value, 'extra_i' is added by 1 from true value
          rotaxis_objpose_cands.push_back(rotaxis_objpose_cand);
          break_for_step = true;

          // // show rotated object & trail
          // motion_.clearObjWorld("dummy_gripper1", false);
          // motion_.clearObjWorld("dummy_gripper2", true);
          // for(int i=0; i<std::get<3>(rotaxis_objpose_cands.back()); i++){
          //   motion_.rotateObj(rotation_axis_mat, i, object_pose, rotated_pose);
          //   motion_.genMeshObjTrans(target_object_file_, rotated_pose, i);
          // }
          // motion_.getVisualTools()->prompt("show rotated object step"); 
          // for(int i=0; i<std::get<3>(rotaxis_objpose_cands.back()); i++)  motion_.clearObjWorld(target_object_file_ + "_" + std::to_string(i));

          // // show rotated object & dummy grippers
          // motion_.clearObjWorld("dummy_gripper1", false);
          // motion_.clearObjWorld("dummy_gripper2", false);
          // for(int i=0; i<=std::get<3>(rotaxis_objpose_cands.back()); i++){
          //   motion_.rotateObj(rotation_axis_mat, i, object_pose, rotated_pose);
          //   motion_.genMeshObjTrans(target_object_file_, rotated_pose, i);
          //   motion_.genDummyGripperTrans(dummy_gripper1, "dummy_gripper1", "open", grasp_pose.first, rotated_pose, rob1_power_point_, i);
          //   motion_.genDummyGripperTrans(dummy_gripper2, "dummy_gripper2", "open", grasp_pose.second, rotated_pose, rob2_power_point_, i);
          //   motion_.genDummyGripperTrans(dummy_gripper1, "dummy_gripper1_pre", "open", pregrasp_pose.first, rotated_pose, rob1_power_point_, i);
          //   motion_.genDummyGripperTrans(dummy_gripper2, "dummy_gripper2_pre", "open", pregrasp_pose.second, rotated_pose, rob2_power_point_, i);
          // }
          // motion_.getVisualTools()->prompt("show rotated object step"); 
          // for(int i=0; i<=std::get<3>(rotaxis_objpose_cands.back()); i++){
          //   motion_.clearObjWorld(target_object_file_ + "_" + std::to_string(i));
          //   motion_.clearObjWorld("dummy_gripper1_" + std::to_string(i), false);
          //   motion_.clearObjWorld("dummy_gripper2_" + std::to_string(i), false);
          //   motion_.clearObjWorld("dummy_gripper1_pre_" + std::to_string(i), false);
          //   motion_.clearObjWorld("dummy_gripper2_pre_" + std::to_string(i), true);
          // }
        }
      }
      
      if(break_for_step)  break;
    }

    motion_.getVisualTools()->deleteAllMarkers();
    motion_.getVisualTools()->trigger();
  }

  grasp_.genMeshObj(target_object_file_, object_pose, target_object, false); // retrieve original object 
  motion_.clearObjWorld("dummy_gripper1", false);
  motion_.clearObjWorld("dummy_gripper2", true);

  return true;
}

// [One grasping pose is upward, the other is downward]
// Rotate object with dummy gripper(open/closed) and find object pose(angle) where the gripper does not collide
// 1)target object 2)grasp pose w.r.t object frame 3)pose direction 4)pregrasp pose w.r.t. object frame 5)rotation axis candidates considering grasp point 
// 6)rotation axis candidates and object pose considering grasp point & dummy gripper
bool Manipulation::mixGraspPose(moveit_msgs::CollisionObject& target_object, std::pair<Eigen::Isometry3d, Eigen::Isometry3d>& grasp_pose
                             , std::pair<Eigen::Isometry3d, Eigen::Isometry3d>& pregrasp_pose, std::vector<std::string> pose_direction
                             , std::vector<std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d, int>>& rotation_axis_dist_cands
                             , std::vector<std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d, int, Eigen::Isometry3d>>& rotaxis_objpose_cands){
  int max_rot_step, extra_rot_step;
  Eigen::Matrix3d rot_mat;
  Eigen::Vector3d point_start, point_end;
  Eigen::Isometry3d object_pose, rotated_pose;
  tf2::fromMsg(target_object.mesh_poses.front(), object_pose); // first:rob1 second:rob2
  moveit_msgs::CollisionObject dummy_gripper1, dummy_gripper2;

  if(pose_direction.size()!=2){
    ROS_ERROR_STREAM("Theh number of 'pose_direction' is not correct.");
    return false;
  }

  // extra rotation step for safety
  if(target_object_file_ == "chair_side_part")        extra_rot_step = 7;
  else if(target_object_file_ == "shelf_side_part")   extra_rot_step = 7;
  else if(target_object_file_ == "stool_side_part")   extra_rot_step = 5;

  for(int axis_i=0; axis_i<rotation_axis_dist_cands.size(); axis_i++){
    max_rot_step = std::get<3>(rotation_axis_dist_cands[axis_i]);
    rot_mat = std::get<2>(rotation_axis_dist_cands[axis_i]);
    point_start = std::get<0>(rotation_axis_dist_cands[axis_i]);
    point_end = std::get<1>(rotation_axis_dist_cands[axis_i]);

    motion_.getVisualTools()->publishArrow(tf2::toMsg(point_start), tf2::toMsg(point_end), rviz_visual_tools::colors::LIME_GREEN, rviz_visual_tools::scales::SMALL);
    motion_.getVisualTools()->trigger();

    bool break_for_step = false;
    for(int step_i=0; step_i<=max_rot_step; step_i++){
      bool not_col_open = true; // true if collision of dummy gripper(open) is not occur
      bool not_col_closed = true; // true if collision of dummy gripper(closed) is not occur
      std::vector<std::string> collision_objects;

      std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d> rotation_axis_mat;
      rotation_axis_mat = std::make_tuple(point_start, point_end, rot_mat);
      motion_.rotateObj(rotation_axis_mat, step_i, object_pose, rotated_pose);
      grasp_.genMeshObj(target_object_file_, rotated_pose, target_object, false);

      // First for grasping pose, second for pregrasp pose
      for(int dummy_i=0; dummy_i<2; dummy_i++){
        // Check collsion of open dummy gripper
        if(dummy_i==0){
          motion_.genDummyGripper(dummy_gripper1, "dummy_gripper1", "open", grasp_pose.first, rotated_pose, rob1_power_point_, false);
          motion_.genDummyGripper(dummy_gripper2, "dummy_gripper2", "open", grasp_pose.second, rotated_pose, rob2_power_point_, false);
        }
        else{
          if(pose_direction[0]=="up" && pose_direction[1]=="down")
            motion_.genDummyGripper(dummy_gripper1, "dummy_gripper1", "open", pregrasp_pose.first, rotated_pose, rob1_power_point_, false);
          else if(pose_direction[0]=="down" && pose_direction[1]=="up")
            motion_.genDummyGripper(dummy_gripper2, "dummy_gripper2", "open", pregrasp_pose.second, rotated_pose, rob2_power_point_, false);
          else{
            ROS_ERROR_STREAM("'pose_direction' is not correct");
            return false;
          }
        }
        collision_objects.clear();
        collision_objects = motion_.getPlanningScenePtr()->getWorld()->getObjectIds();

        // check validity 
        if(motion_.checkStateValid(true)){ // true: valid
          // check collision(between collision objects) 
          for(int col_i=0; col_i<collision_objects.size(); col_i++){
            if(target_object_file_ != collision_objects[col_i]){
              if(!motion_.checkCollisionObj(dummy_gripper1.id, collision_objects[col_i]) // true: collision occur
                 && !motion_.checkCollisionObj(dummy_gripper2.id, collision_objects[col_i])){
                not_col_open = not_col_open*true;
              }
              else{
                not_col_open = false;
                break;
              }
            }
          }
        }
        else  not_col_open = false;

        // when open dummy gripper does not collied with anything, do collision check w.r.t closed dummy gripper
        if(not_col_open){
          if(dummy_i==0){
            motion_.genDummyGripper(dummy_gripper1, "dummy_gripper1", "closed", grasp_pose.first, rotated_pose, rob1_power_point_, false);
            motion_.genDummyGripper(dummy_gripper2, "dummy_gripper2", "closed", grasp_pose.second, rotated_pose, rob2_power_point_, false);
          }
          else{
            if(pose_direction[0]=="up" && pose_direction[1]=="down")
              motion_.genDummyGripper(dummy_gripper1, "dummy_gripper1", "closed", pregrasp_pose.first, rotated_pose, rob1_power_point_, false);
            else if(pose_direction[0]=="down" && pose_direction[1]=="up")
              motion_.genDummyGripper(dummy_gripper2, "dummy_gripper2", "closed", pregrasp_pose.second, rotated_pose, rob2_power_point_, false);
          }
          collision_objects.clear();
          collision_objects = motion_.getPlanningScenePtr()->getWorld()->getObjectIds();

          // check validity 
          if(motion_.checkStateValid(true)){ // true: valid
            // check collision(between collision objects) 
            for(int col_i=0; col_i<collision_objects.size(); col_i++){
              if(target_object_file_ != collision_objects[col_i]){
                if(!motion_.checkCollisionObj(dummy_gripper1.id, collision_objects[col_i]) // true: collision occur
                   && !motion_.checkCollisionObj(dummy_gripper2.id, collision_objects[col_i])){
                  not_col_closed = not_col_closed*true;
                }
                else{
                  not_col_closed = false;
                  break;
                }
              }
            }
          }
          else  not_col_closed = false;
        }
        else  break; // open dummy gripper collides with something
      }

      // when dummy grippers do not collide with anything
      if(not_col_open*not_col_closed){
        // rotate object & dummy grippers according to extra step for safety
        bool success = true; // success to get object pose(angle) considering dummy gripper
        bool break_for_extra = false;
        bool break_for_dummy = false;
        int extra_i;
        for(extra_i=1; extra_i<=extra_rot_step; extra_i++){
          // First for grasping pose, second for pregrasp pose
          for(int dummy_i=0; dummy_i<2; dummy_i++){
            motion_.rotateObj(rotation_axis_mat, step_i+extra_i, object_pose, rotated_pose);
            grasp_.genMeshObj(target_object_file_, rotated_pose, target_object, false);
            if(dummy_i==0){
              motion_.genDummyGripper(dummy_gripper1, "dummy_gripper1", "open", grasp_pose.first, rotated_pose, rob1_power_point_, false);
              motion_.genDummyGripper(dummy_gripper2, "dummy_gripper2", "open", grasp_pose.second, rotated_pose, rob2_power_point_, false);
            }
            else{
              if(pose_direction[0]=="up" && pose_direction[1]=="down")
                motion_.genDummyGripper(dummy_gripper1, "dummy_gripper1", "closed", pregrasp_pose.first, rotated_pose, rob1_power_point_, false);
              else if(pose_direction[0]=="down" && pose_direction[1]=="up")
                motion_.genDummyGripper(dummy_gripper2, "dummy_gripper2", "closed", pregrasp_pose.second, rotated_pose, rob2_power_point_, false);
            }
            collision_objects.clear();
            collision_objects = motion_.getPlanningScenePtr()->getWorld()->getObjectIds();

            // check validity
            if(!motion_.checkStateValid(true)){ // true: valid
              ROS_INFO_STREAM("Invalid state is occur when rotating the object additionally");
              break_for_extra = true;
              break_for_step = true;
              break_for_dummy = true;
              success = false;
              break;
            }

            // check collision(between collision objects)
            for(int col_i=0; col_i<collision_objects.size(); col_i++){
              if(target_object_file_ != collision_objects[col_i]){
                if(motion_.checkCollisionObj(dummy_gripper1.id, collision_objects[col_i]) // true: collision occur
                  || motion_.checkCollisionObj(dummy_gripper2.id, collision_objects[col_i])){ 
                  ROS_INFO_STREAM("Collsion occur when rotatin the object additionally");
                  break_for_extra = true;
                  break_for_step = true;
                  break_for_dummy = true;
                  success = false;
                  break;
                }
              }
            }

            if(break_for_dummy) break;
          }

          if(break_for_extra)   break;
        }

        if(success){
          ROS_INFO_STREAM("Object pose(angle) is derived considering the dummy gripper");
          std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d, int, Eigen::Isometry3d> rotaxis_objpose_cand;
          rotaxis_objpose_cand = std::make_tuple(point_start, point_end, rot_mat, step_i+extra_i-1, rotated_pose); // -1: when exiting above for statement with valid value, 'extra_i' is added by 1 from true value
          rotaxis_objpose_cands.push_back(rotaxis_objpose_cand);
          break_for_step = true;

          // // show rotated object & trail
          // motion_.clearObjWorld("dummy_gripper1", false);
          // motion_.clearObjWorld("dummy_gripper2", true);
          // for(int i=0; i<std::get<3>(rotaxis_objpose_cands.back()); i++){
          //   motion_.rotateObj(rotation_axis_mat, i, object_pose, rotated_pose);
          //   motion_.genMeshObjTrans(target_object_file_, rotated_pose, i);
          // }
          // motion_.getVisualTools()->prompt("show rotated object step"); 
          // for(int i=0; i<std::get<3>(rotaxis_objpose_cands.back()); i++)  motion_.clearObjWorld(target_object_file_ + "_" + std::to_string(i));

          // // show rotated object & dummy grippers
          // for(int i=0; i<std::get<3>(rotaxis_objpose_cands.back()); i++){
          //   motion_.rotateObj(rotation_axis_mat, i, object_pose, rotated_pose);
          //   motion_.genMeshObjTrans(target_object_file_, rotated_pose, i);
          //   motion_.genDummyGripperTrans(dummy_gripper1, "dummy_gripper1", "closed", grasp_pose.first, rotated_pose, rob1_power_point_, i);
          //   motion_.genDummyGripperTrans(dummy_gripper2, "dummy_gripper2", "closed", grasp_pose.second, rotated_pose, rob2_power_point_, i);
          // }
          // motion_.getVisualTools()->prompt("show rotated object step"); 
          // for(int i=0; i<std::get<3>(rotaxis_objpose_cands.back()); i++){
          //   motion_.clearObjWorld(target_object_file_ + "_" + std::to_string(i));
          //   motion_.clearObjWorld("dummy_gripper1_" + std::to_string(i), false);
          //   motion_.clearObjWorld("dummy_gripper2_" + std::to_string(i), true);
          // }
        }
      }
      
      if(break_for_step)  break;
    }

    motion_.getVisualTools()->deleteAllMarkers();
    motion_.getVisualTools()->trigger();
  }

  grasp_.genMeshObj(target_object_file_, object_pose, target_object, false); // retrieve original object 
  motion_.clearObjWorld("dummy_gripper1", false);
  motion_.clearObjWorld("dummy_gripper2", true);

  return true;
}

// get IK solutions when object is rotated
// 1)target object 2)grasp pose w.r.t object frame 3)rotation axis / rotation matrix / rotation step / rotated object TF considering dummy grippers
// 4)rotation axis / rotation matrix / rotation step / rotated object TF / IK solutions
bool Manipulation::getIKObj(moveit_msgs::CollisionObject& target_object, std::pair<Eigen::Isometry3d, Eigen::Isometry3d>& grasp_pose
                          , std::vector<std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d, int, Eigen::Isometry3d>>& rotaxis_objpose_cands
                          , std::vector<std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d, int, Eigen::Isometry3d, std::vector<std::vector<double>>>>& rotaxis_objpose_ik_cands){
  Eigen::Isometry3d object_pose, rotated_pose;
  Eigen::Vector3d point_start, point_end;
  Eigen::Matrix3d rotation_mat;
  int rotation_step;
  std::pair<Eigen::Isometry3d, Eigen::Isometry3d> grasp_pose_rotated;
  tf2::fromMsg(target_object.mesh_poses.front(), object_pose); 
  rotaxis_objpose_ik_cands.clear();
  
  // derive IK solutions when object is rotated to secure power grasp
  EigenSTL::vector_Isometry3d target_pose;
  std::vector<std::string> target_tip = {rob1_power_point_, rob2_power_point_};
  std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d, int, Eigen::Isometry3d, std::vector<std::vector<double>>> rotaxis_objpose_ik_cand;
  std::vector<std::vector<double>> multiple_joint_sols, multiple_joint_sols_tmp;
  std::vector<double> original_joints_value; 
  motion_.getRobotStatePtr()->copyJointGroupPositions(motion_.getJMGPtr(robs_group_name_), original_joints_value); // get original joints value
  std::vector<std::string> robs_joint_names_ = motion_.getJMGPtr(robs_group_name_)->getActiveJointModelNames(); // active joints of rob1 and rob2

  for(int rot_i=0; rot_i<rotaxis_objpose_cands.size(); rot_i++){
    point_start = std::get<0>(rotaxis_objpose_cands[rot_i]);
    point_end = std::get<1>(rotaxis_objpose_cands[rot_i]);
    rotation_mat = std::get<2>(rotaxis_objpose_cands[rot_i]);
    rotation_step = std::get<3>(rotaxis_objpose_cands[rot_i]);
    rotated_pose = std::get<4>(rotaxis_objpose_cands[rot_i]);

    convertRefFrame(grasp_pose, rotated_pose, grasp_pose_rotated); // rotated grasp pose according to the rotated object pose
    
    grasp_.genMeshObj(target_object_file_, rotated_pose, target_object, false);
    motion_.getVisualTools()->publishArrow(tf2::toMsg(point_start), tf2::toMsg(point_end), rviz_visual_tools::colors::LIME_GREEN, rviz_visual_tools::scales::SMALL);
    motion_.getVisualTools()->trigger();

    // // show grasp pose
    // motion_.getVisualTools()->publishAxisLabeled(grasp_pose_rotated.first, "rob1");
    // motion_.getVisualTools()->publishAxisLabeled(grasp_pose_rotated.second, "rob2");
    // motion_.getVisualTools()->trigger();
    // motion_.getVisualTools()->prompt("grasp pose");
    // motion_.getVisualTools()->deleteAllMarkers();
    // motion_.getVisualTools()->trigger();

    target_pose.clear();
    multiple_joint_sols.clear();
    multiple_joint_sols_tmp.clear(); // TEST. TODO: Have to check if it is necessary ///////////////////////////////////////////////////
    target_pose = {grasp_pose_rotated.first, grasp_pose_rotated.second};
    bool ik_success = motion_.getIKsolutions(target_pose, motion_.getJMGPtr(robs_group_name_), target_tip, multiple_joint_sols);
    if(ik_success){ // if ik solution exist
      // collision check when robot is in multiple_joint_sols and save valid ones
      for(int sol_i=0; sol_i<multiple_joint_sols.size(); sol_i++){
        motion_.getRobotStatePtr()->setVariablePositions(robs_joint_names_, multiple_joint_sols[sol_i]);
        motion_.updatePlanningScene();

        // // show robot pose
        // motion_.getVisualTools()->publishRobotState(multiple_joint_sols[sol_i], motion_.getJMGPtr(robs_group_name_), rviz_visual_tools::colors::LIME_GREEN);
        // motion_.getVisualTools()->trigger();
        // motion_.getVisualTools()->prompt("IK solution");

        if(motion_.checkStateValid(true)){ // if ik solution is valid
          multiple_joint_sols_tmp.push_back(multiple_joint_sols[sol_i]);
        }
      }

      if(multiple_joint_sols_tmp.size() != 0){ // if valid of ik solution exists
        rotaxis_objpose_ik_cand = std::make_tuple(point_start, point_end, rotation_mat, rotation_step, rotated_pose, multiple_joint_sols_tmp);
        rotaxis_objpose_ik_cands.push_back(rotaxis_objpose_ik_cand);
      }
    }

    motion_.getVisualTools()->deleteAllMarkers();
    motion_.getVisualTools()->trigger();
  }

  grasp_.genMeshObj(target_object_file_, object_pose, target_object, false); // retrieve original object 
  motion_.getRobotStatePtr()->setJointGroupPositions(motion_.getJMGPtr(robs_group_name_), original_joints_value); // retrieve robot pose
  motion_.updatePlanningScene();
  motion_.getVisualTools()->deleteAllMarkers();
  motion_.getVisualTools()->trigger();

  if(rotaxis_objpose_ik_cands.size() == 0){
    ROS_WARN_STREAM("There is no valid IK solutions (getIKObj())");
    return false;
  }

  // // show IK solution trail
  // std::vector<trajectory_msgs::JointTrajectoryPoint> ik_sols;
  // for(int i=0; i<rotaxis_objpose_ik_cands.size(); i++){
  //   for(int sol_i=0; sol_i<std::get<5>(rotaxis_objpose_ik_cands[i]).size(); sol_i++){
  //     trajectory_msgs::JointTrajectoryPoint ik_sol;
  //     ik_sol.positions = std::get<5>(rotaxis_objpose_ik_cands[i])[sol_i];
  //     ik_sols.push_back(ik_sol);
  //   }
  //   grasp_.genMeshObj(target_object_file_, std::get<4>(rotaxis_objpose_ik_cands[i]), target_object, false);
  //   motion_.getVisualTools()->prompt("IK solutions trail for power grasp [" + std::to_string(i+1) + "/" + std::to_string(rotaxis_objpose_ik_cands.size()) + "]");
  //   motion_.getVisualTools()->publishIKSolutions(ik_sols, motion_.getJMGPtr(robs_group_name_));
  //   motion_.getVisualTools()->prompt("IK solutions trail");
  //   ik_sols.clear();
  // }
  // motion_.getVisualTools()->prompt("All IK solutions trails are finished");

  return true;
}

// get IK solutions of support grasp when object is rotated
// 1)target object 2)grasp pose w.r.t object frame 3)support grasp pose w.r.t object frame 4)rotation axis / rotation matrix / rotation step / rotated object TF / IK solutions
// 5)rotation axis(start/end) / rotation matrix / rotation step / rotated object TF / supportive robot arm(rob1/rob2) / IK solutions of support grasp
bool Manipulation::getSupIKObj(moveit_msgs::CollisionObject& target_object, std::pair<Eigen::Isometry3d, Eigen::Isometry3d>& grasp_pose, std::pair<Eigen::Isometry3d, Eigen::Isometry3d>& sup_grasp_pose
                             , std::vector<std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d, int, Eigen::Isometry3d, std::vector<std::vector<double>>>>& rotaxis_objpose_ik_cands
                             , std::vector<std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d, int, Eigen::Isometry3d, std::string, std::vector<std::vector<double>>>>& rotaxis_objpose_sup_ik_cands){
  Eigen::Isometry3d object_pose, rotated_pose, target_pose;
  Eigen::Vector3d point_start, point_end, rob1_grasp_point, rob2_grasp_point;
  Eigen::Matrix3d rotation_mat;
  int rotation_step;
  double rob1_distance, rob2_distance;
  std::string supportive_robot;
  std::pair<Eigen::Isometry3d, Eigen::Isometry3d> grasp_pose_rotated, sup_grasp_pose_rotated;
  tf2::fromMsg(target_object.mesh_poses.front(), object_pose); 
  rotaxis_objpose_sup_ik_cands.clear();

  std::vector<double> original_joints_value; 
  motion_.getRobotStatePtr()->copyJointGroupPositions(motion_.getJMGPtr(robs_group_name_), original_joints_value); // get original joints value
  std::vector<std::string> rob1_joint_names = motion_.getJMGPtr(rob1_group_name_)->getActiveJointModelNames(); // active joints of rob1 
  std::vector<std::string> rob2_joint_names = motion_.getJMGPtr(rob2_group_name_)->getActiveJointModelNames(); // active joints of rob2 

  // derive ik solution about support grasp pose when object is rotated
  std::vector<std::vector<double>> multiple_joint_sols, multiple_joint_sols_tmp;
  std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d, int, Eigen::Isometry3d, std::string, std::vector<std::vector<double>>> rotaxis_objpose_sup_ik_cand;

  for(int pose_i=0; pose_i<rotaxis_objpose_ik_cands.size(); pose_i++){
    point_start = std::get<0>(rotaxis_objpose_ik_cands[pose_i]);
    point_end = std::get<1>(rotaxis_objpose_ik_cands[pose_i]);
    rotation_mat = std::get<2>(rotaxis_objpose_ik_cands[pose_i]);
    rotation_step = std::get<3>(rotaxis_objpose_ik_cands[pose_i]);
    rotated_pose = std::get<4>(rotaxis_objpose_ik_cands[pose_i]);

    convertRefFrame(grasp_pose, rotated_pose, grasp_pose_rotated); // rotated grasp pose according to the rotated object pose
    convertRefFrame(sup_grasp_pose, rotated_pose, sup_grasp_pose_rotated); // rotated support grasp pose according to the rotated object pose

    motion_.getVisualTools()->publishArrow(tf2::toMsg(point_start), tf2::toMsg(point_end), rviz_visual_tools::colors::LIME_GREEN, rviz_visual_tools::scales::SMALL);
    motion_.getVisualTools()->trigger();
    grasp_.genMeshObj(target_object_file_, rotated_pose, target_object, false); // retrieve original object 

    // // show grasp pose
    // motion_.getVisualTools()->publishAxisLabeled(sup_grasp_pose_rotated.first, "rob1_sup");
    // motion_.getVisualTools()->publishAxisLabeled(sup_grasp_pose_rotated.second, "rob2_sup");
    // motion_.getVisualTools()->trigger();
    // motion_.getVisualTools()->prompt("support grasp pose");
    // motion_.getVisualTools()->deleteAllMarkers();
    // motion_.getVisualTools()->trigger();

    // select supportive robot arm based on the distance between rotation axis & grasp(power) point
    rob1_grasp_point = grasp_pose_rotated.first.translation();
    rob2_grasp_point = grasp_pose_rotated.second.translation();
    point_start = std::get<0>(rotaxis_objpose_ik_cands[pose_i]);
    point_end = std::get<1>(rotaxis_objpose_ik_cands[pose_i]);

    // coincide the reference point with 'point_end'
    rob1_grasp_point = rob1_grasp_point - point_start;
    rob2_grasp_point = rob2_grasp_point - point_start;
    point_end = point_end - point_start;
    point_start = Eigen::Vector3d::Zero();

    rob1_distance = (rob1_grasp_point - rob1_grasp_point.dot(point_end.normalized())*point_end.normalized()).norm();
    rob2_distance = (rob2_grasp_point - rob2_grasp_point.dot(point_end.normalized())*point_end.normalized()).norm();

    // // select supportive robot arm (Origianl method to determine supportive robot arm)
    // if(rob1_distance > rob2_distance)   supportive_robot = rob2_group_name_;
    // else                                supportive_robot = rob1_group_name_;

    // derive ik solutions
    bool ik_success;
    int switched_num = 0;
    while(true){
      multiple_joint_sols.clear();
      multiple_joint_sols_tmp.clear();

      // // (Origianl method to determine supportive robot arm)
      // if(switched_num >= 2){
      //   ROS_WARN_STREAM("There is no IK solution of support grasp pose");
      //   break;
      // }

      if(switched_num == 0){
        ROS_INFO_STREAM("The case of 'supportive_robot': " << rob2_group_name_);
        supportive_robot = rob2_group_name_;
      }
      else if(switched_num == 1){
        ROS_INFO_STREAM("The case of 'supportive_robot': " << rob1_group_name_);
        supportive_robot = rob1_group_name_;
      }
      else  break;

      if(supportive_robot == rob2_group_name_){ // supportive robot arm: rob2
        ik_success = motion_.getIKsolutions(sup_grasp_pose_rotated.second, motion_.getJMGPtr(rob2_group_name_), rob2_precision_point_, multiple_joint_sols);

        // Modified algorithm to get rotating path
        if(ik_success){
          // collision check when robot is in multiple_joint_sols and save valid ones
          for(int sol_i=0; sol_i<multiple_joint_sols.size(); sol_i++){
            motion_.getRobotStatePtr()->setVariablePositions(rob2_joint_names, multiple_joint_sols[sol_i]);
            motion_.updatePlanningScene();

            if(motion_.checkStateValid(true)){ // if ik solution is valid
              multiple_joint_sols_tmp.push_back(multiple_joint_sols[sol_i]);
            }
          }
          if(multiple_joint_sols_tmp.size() != 0){
            ROS_INFO_STREAM("Valid IK solutions of support grasp pose are derived");
            rotaxis_objpose_sup_ik_cand = std::make_tuple(std::get<0>(rotaxis_objpose_ik_cands[pose_i]), std::get<1>(rotaxis_objpose_ik_cands[pose_i])
                                                        , rotation_mat, rotation_step, rotated_pose, supportive_robot, multiple_joint_sols_tmp);
            rotaxis_objpose_sup_ik_cands.push_back(rotaxis_objpose_sup_ik_cand); // including ik solution of support grasp pose of rotated object
          }
        }
        switched_num++;
      }
      else{ // supportive robot arm: rob1
        ik_success = motion_.getIKsolutions(sup_grasp_pose_rotated.first, motion_.getJMGPtr(rob1_group_name_), rob1_precision_point_, multiple_joint_sols);

        // Modified algorithm to get rotating path
        if(ik_success){
          // collision check when robot is in multiple_joint_sols and save valid ones
          for(int sol_i=0; sol_i<multiple_joint_sols.size(); sol_i++){
            motion_.getRobotStatePtr()->setVariablePositions(rob1_joint_names, multiple_joint_sols[sol_i]);
            motion_.updatePlanningScene();

            if(motion_.checkStateValid(true)){ // if ik solution is valid
              multiple_joint_sols_tmp.push_back(multiple_joint_sols[sol_i]);
            }
          }
          if(multiple_joint_sols_tmp.size() != 0){
            ROS_INFO_STREAM("Valid IK solutions of support grasp pose are derived");
            rotaxis_objpose_sup_ik_cand = std::make_tuple(std::get<0>(rotaxis_objpose_ik_cands[pose_i]), std::get<1>(rotaxis_objpose_ik_cands[pose_i])
                                                        , rotation_mat, rotation_step, rotated_pose, supportive_robot, multiple_joint_sols_tmp);
            rotaxis_objpose_sup_ik_cands.push_back(rotaxis_objpose_sup_ik_cand); // including ik solution of support grasp pose of rotated object
          }
        }
        switched_num++;
      }

      // // Show IK solutiom for the supportive robot arm
      // for(int test_i=0; test_i<multiple_joint_sols.size(); test_i++){
      //   motion_.getVisualTools()->publishRobotState(multiple_joint_sols[test_i], motion_.getJMGPtr(supportive_robot), rviz_visual_tools::colors::RED);
      //   motion_.getVisualTools()->trigger();
      //   motion_.getVisualTools()->prompt("IK TEST: " + std::to_string(test_i+1) + "/" + std::to_string(multiple_joint_sols.size()));
      // }
    }

    motion_.getRobotStatePtr()->setJointGroupPositions(motion_.getJMGPtr(robs_group_name_), original_joints_value); // retrieve robot pose
    motion_.updatePlanningScene();
    motion_.getVisualTools()->deleteAllMarkers();
    motion_.getVisualTools()->trigger();
  }

  grasp_.genMeshObj(target_object_file_, object_pose, target_object, false); // retrieve original object 
  motion_.getRobotStatePtr()->setJointGroupPositions(motion_.getJMGPtr(robs_group_name_), original_joints_value); // retrieve robot pose
  motion_.updatePlanningScene();
  motion_.getVisualTools()->deleteAllMarkers();
  motion_.getVisualTools()->trigger();

  if(rotaxis_objpose_sup_ik_cands.size() == 0){
    ROS_ERROR_STREAM("There is no IK solution of support grasp");
    return false;
  }

  // // show IK solution trail
  // std::vector<trajectory_msgs::JointTrajectoryPoint> ik_sols;
  // for(int i=0; i<rotaxis_objpose_sup_ik_cands.size(); i++){
  //   for(int sol_i=0; sol_i<std::get<6>(rotaxis_objpose_sup_ik_cands[i]).size(); sol_i++){
  //     trajectory_msgs::JointTrajectoryPoint ik_sol;
  //     ik_sol.positions = std::get<6>(rotaxis_objpose_sup_ik_cands[i])[sol_i];
  //     ik_sols.push_back(ik_sol);
  //   }
  //   grasp_.genMeshObj(target_object_file_, std::get<4>(rotaxis_objpose_sup_ik_cands[i]), target_object, false);
  //   motion_.getVisualTools()->prompt("IK solutions trail for precision grasp [" + std::to_string(i+1) + "/" + std::to_string(rotaxis_objpose_sup_ik_cands.size()) + "]");
  //   motion_.getVisualTools()->publishIKSolutions(ik_sols, motion_.getJMGPtr(std::get<5>(rotaxis_objpose_sup_ik_cands[i])));
  //   motion_.getVisualTools()->prompt("IK solutions trail");
  //   ik_sols.clear();
  // }
  // motion_.getVisualTools()->prompt("All IK solutions trails are finished");

  return true;
}

// derive support robot arm to rotate object (if a regrasp motion exists, the path of the master robot arm to grip the object while regrasping motion is performed is planned)
// 1)target object 2)support grasp pose w.r.t object frame 3) grasp pose w.r.t object frame 
// 4)rotation axis(start/end) / rotation matrix / rotation step / rotated object TF / supportive robot arm(rob1/rob2) / IK solutions of support grasp
// 5)rotation axis(start/end) / rotation matrix / rotation step / rotated object TF / supportive robot arm(rob1/rob2) / trajectory to rotate object / trajectory index before joint space jump occurs (negative: no jump)
// / master: initial -> pregrasp of middle pose & pregrasp of middle pose -> middle pose / object pose when joint space jump occurs
bool Manipulation::getSupRobPath(moveit_msgs::CollisionObject& target_object, std::pair<Eigen::Isometry3d, Eigen::Isometry3d>& sup_grasp_pose, std::pair<Eigen::Isometry3d, Eigen::Isometry3d>& grasp_pose
                               , std::vector<std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d, int, Eigen::Isometry3d, std::string, std::vector<std::vector<double>>>>& rotaxis_objpose_sup_ik_cands
                               , std::vector<std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d, int, Eigen::Isometry3d, std::string
                               , std::vector<robot_state::RobotStatePtr>, int, std::vector<moveit_msgs::RobotTrajectory>, Eigen::Isometry3d>>& rotaxis_objpose_sup_traj_cands){
  Eigen::Isometry3d object_pose, rotated_pose;
  Eigen::Vector3d point_start, point_end;
  Eigen::Matrix3d rotation_mat;
  int rotation_step;
  std::string supportive_robot, master_robot;
  tf2::fromMsg(target_object.mesh_poses.front(), object_pose); 

  std::vector<std::vector<double>> consistency_limits;
  std::tuple<Eigen::Vector3d, Eigen::Vector3d> rotation_axis;
  Eigen::Isometry3d sup_grasp_tf, sup_grasp_tf_rotated;
  std::vector<std::vector<double>> multiple_joint_sols;

  std::vector<double> original_joints_value, rob1_original_joints_value, rob2_original_joints_value; 
  motion_.getRobotStatePtr()->copyJointGroupPositions(motion_.getJMGPtr(robs_group_name_), original_joints_value); // get original joints value
  motion_.getRobotStatePtr()->copyJointGroupPositions(motion_.getJMGPtr(rob1_group_name_), rob1_original_joints_value); // get original joints value
  motion_.getRobotStatePtr()->copyJointGroupPositions(motion_.getJMGPtr(rob2_group_name_), rob2_original_joints_value); // get original joints value
  std::vector<std::string> rob1_joint_names = motion_.getJMGPtr(rob1_group_name_)->getActiveJointModelNames(); // active joints of rob1 
  std::vector<std::string> rob2_joint_names = motion_.getJMGPtr(rob2_group_name_)->getActiveJointModelNames(); // active joints of rob2 

  rotaxis_objpose_sup_traj_cands.clear();

  for(int sup_i=0; sup_i<rotaxis_objpose_sup_ik_cands.size(); sup_i++){
    multiple_joint_sols.clear();
    point_start = std::get<0>(rotaxis_objpose_sup_ik_cands[sup_i]);
    point_end = std::get<1>(rotaxis_objpose_sup_ik_cands[sup_i]);
    rotation_mat = std::get<2>(rotaxis_objpose_sup_ik_cands[sup_i]);
    rotation_step = std::get<3>(rotaxis_objpose_sup_ik_cands[sup_i]);
    rotated_pose = std::get<4>(rotaxis_objpose_sup_ik_cands[sup_i]);
    supportive_robot = std::get<5>(rotaxis_objpose_sup_ik_cands[sup_i]);
    multiple_joint_sols = std::get<6>(rotaxis_objpose_sup_ik_cands[sup_i]);

    motion_.getVisualTools()->publishArrow(tf2::toMsg(point_start), tf2::toMsg(point_end), rviz_visual_tools::colors::LIME_GREEN, rviz_visual_tools::scales::SMALL);
    motion_.getVisualTools()->trigger();
    grasp_.genMeshObj(target_object_file_, rotated_pose, target_object, false);

    motion_.getRobotStatePtr()->setJointGroupPositions(motion_.getJMGPtr(robs_group_name_), original_joints_value); // retrieve robot pose
    motion_.updatePlanningScene();

    std::vector<robot_state::RobotStatePtr> sup_traj, sup_traj_all, sup_traj_second, sup_traj_final;
    std::vector<double> multiple_joint_sol;
    std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d, int, Eigen::Isometry3d, std::string
             , std::vector<robot_state::RobotStatePtr>, int, std::vector<moveit_msgs::RobotTrajectory>, Eigen::Isometry3d> rotaxis_objpose_sup_traj_cand;
    for(int sol_i=0; sol_i<multiple_joint_sols.size(); sol_i++){ // robot configuration when robot grasp the rotated object in support grasp pose
      if(supportive_robot == rob2_group_name_){
        motion_.getRobotStatePtr()->setJointGroupPositions(motion_.getJMGPtr(rob2_group_name_), multiple_joint_sols[sol_i]);
        sup_grasp_tf = sup_grasp_pose.second;
        motion_.setJumpThreshold(motion_.getJMGPtr(rob2_group_name_), consistency_limits);
        master_robot = rob1_group_name_;
      }
      else{
        motion_.getRobotStatePtr()->setJointGroupPositions(motion_.getJMGPtr(rob1_group_name_), multiple_joint_sols[sol_i]);
        sup_grasp_tf = sup_grasp_pose.first;
        motion_.setJumpThreshold(motion_.getJMGPtr(rob1_group_name_), consistency_limits);
        master_robot = rob2_group_name_;
      }

      // moving rotated object to origianl pose and find IK solution to do this task
      sup_traj.clear();
      for(int step_i=rotation_step; step_i >= 0; step_i--){ 
        rotation_axis = std::make_tuple(point_start, point_end);
        if(step_i!=0) motion_.rotateObj(rotation_axis, step_i, object_pose, rotated_pose);
        else          rotated_pose = object_pose;
        grasp_.genMeshObj(target_object_file_, rotated_pose, target_object, false);

        convertRefFrame(sup_grasp_tf, rotated_pose, sup_grasp_tf_rotated); // rotated support grasp pose according to the rotated object pose

        // // Show robot configurations to rotate the object
        // motion_.getVisualTools()->publishAxisLabeled(sup_grasp_tf_rotated, "sup_grasp");
        // motion_.getVisualTools()->trigger();
        // motion_.getVisualTools()->prompt("sup rotation path test. mul_sols: " + std::to_string(sol_i+1) + "/" + std::to_string(multiple_joint_sols.size()) + ", step_i: " + std::to_string(step_i));
        // motion_.getVisualTools()->deleteAllMarkers();
        // motion_.getVisualTools()->trigger();
      
        if(supportive_robot == rob2_group_name_){
          bool ik_success = motion_.getSimIKsolution(sup_grasp_tf_rotated, motion_.getJMGPtr(rob2_group_name_), rob2_precision_point_, multiple_joint_sol);
          if(!ik_success){
            ROS_WARN_STREAM("The path of " << supportive_robot << " to rotate the object is not valid using ik_sol[" << sol_i+1 << "/" << multiple_joint_sols.size() << "]");
            break;
          }
          motion_.getRobotStatePtr()->setJointGroupPositions(motion_.getJMGPtr(rob2_group_name_), multiple_joint_sol);
        }
        else{
          bool ik_success = motion_.getSimIKsolution(sup_grasp_tf_rotated, motion_.getJMGPtr(rob1_group_name_), rob1_precision_point_, multiple_joint_sol);
          if(!ik_success){
            ROS_WARN_STREAM("The path of " << supportive_robot << " to rotate the object is not valid using ik_sol[" << sol_i+1 << "/" << multiple_joint_sols.size() << "]");
            break;
          }
          motion_.getRobotStatePtr()->setJointGroupPositions(motion_.getJMGPtr(rob1_group_name_), multiple_joint_sol);
        }
        motion_.updatePlanningScene();

        if(motion_.checkStateValid(true))   sup_traj.push_back(robot_state::RobotStatePtr(new robot_state::RobotState(*motion_.getRobotStatePtr().get())));
        else{
          ROS_WARN_STREAM("'sup_traj' is not valid");
          sup_traj.clear();
          break;
        }
      }
      
      // check jointspace jump 
      if(sup_traj.size()!=0){
        sup_traj_all.clear();
        sup_traj_second.clear();
        sup_traj_all = sup_traj; // copy all sup_traj. If a joint space jump occurs, sup_traj has only the trajectory before joint space jump
        double continuity = robot_state::RobotState::testJointSpaceJump(motion_.getJMGPtr(robs_group_name_), sup_traj, jump_threshold_);
        if(continuity!=1.0){
          // From now, 'sup_traj' has the trajectory which does not occur joint space jump (object is moved from upward into downward(middle))
          ROS_WARN_STREAM("'sup_traj' has joint state jump. continuity: " << continuity); 

          // Try to connect the disconnected points ///////
          int boundary_index = sup_traj.size(); // valid index of the trajectory

          // move object in the pose just before joint space jump occurs
          motion_.rotateObj(rotation_axis, sup_traj_all.size()-boundary_index-1, object_pose, rotated_pose);
          grasp_.genMeshObj(target_object_file_, rotated_pose, target_object, false); 
          Eigen::Isometry3d obj_pose_jump = rotated_pose; // object pose when a joint space jump occurs
          
          // robot_state::RobotStatePtr robot_state_original(new robot_state::RobotState(*motion_.getRobotStatePtr().get()));
          std::vector<double> joints_position;
          sup_traj.back()->copyJointGroupPositions(motion_.getJMGPtr(supportive_robot), joints_position);
          motion_.getRobotStatePtr()->setJointGroupPositions(motion_.getJMGPtr(supportive_robot), joints_position); // assign the last sup_traj to robot_state 
          motion_.updatePlanningScene();
          
          Eigen::Isometry3d graps_pose_boundary; // grasp pose to connect the trajectory between the points disconnected
          if(supportive_robot == rob1_group_name_)       graps_pose_boundary = motion_.getRobotStatePtr()->getFrameTransform(rob1_precision_point_);
          else if(supportive_robot == rob2_group_name_)  graps_pose_boundary = motion_.getRobotStatePtr()->getFrameTransform(rob2_precision_point_);
          else{
            ROS_ERROR_STREAM("The name of the supportive robot is not correct.");
            return false;
          }

          // get robot configuration just after joint space jump occurs
          joints_position.clear();
          sup_traj_all[boundary_index]->copyJointGroupPositions(motion_.getJMGPtr(supportive_robot), joints_position);
          motion_.getRobotStatePtr()->setJointGroupPositions(motion_.getJMGPtr(supportive_robot), joints_position); // assign the trajectory jumped joint space to robot_state
          motion_.updatePlanningScene();

          // Find robot configuration of 'grasp_pose_boundary' which is similar to the configuration just after the situation of the joint space jump 
          bool boundary_success = false;
          joints_position.clear();
          if(supportive_robot == rob1_group_name_)
            boundary_success = motion_.getSimIKsolution(graps_pose_boundary, motion_.getJMGPtr(supportive_robot), rob1_precision_point_, joints_position);
          else if(supportive_robot == rob2_group_name_)
            boundary_success = motion_.getSimIKsolution(graps_pose_boundary, motion_.getJMGPtr(supportive_robot), rob2_precision_point_, joints_position);

          if(!boundary_success){
            ROS_WARN_STREAM("Planning the path of " << supportive_robot << " to connect between the disconnected points is failed.");
            break;
          }
          else{
            robot_state::RobotState robot_state_tmp = *motion_.getRobotStatePtr().get(); 
            robot_state_tmp.setJointGroupPositions(motion_.getJMGPtr(supportive_robot), joints_position); // copy robot state data 
            robot_state::RobotStatePtr robot_state_ptr_tmp(robot_state::RobotStatePtr(new moveit::core::RobotState(robot_state_tmp))); // initialize std::shared_ptr
            sup_traj_second.push_back(robot_state_ptr_tmp); // assign robot_state as the first point of the second trajectory
            for(int traj_i=boundary_index; traj_i<sup_traj_all.size(); traj_i++)  sup_traj_second.push_back(sup_traj_all[traj_i]); // determine the second trajectory from middle to initial pose

            double continuity_second = robot_state::RobotState::testJointSpaceJump(motion_.getJMGPtr(robs_group_name_), sup_traj_second, jump_threshold_);
            if(continuity_second!=1.0){
              ROS_WARN_STREAM("To connect the disconnected points of the supportive robot arm is failed");
              break;
            }
            else{
              ROS_INFO_STREAM("Regrasp to connect the disconnected points of the supportive robot arm is successfully found");

              // 'sup_traj' and 'sup_traj_second' add to 'rotaxis_objpose_sup_traj_cands'. Later, this data should be check if regrasp method exist or not
              sup_traj_final.clear();
              for(auto sup_traj_one : sup_traj)         sup_traj_final.push_back(sup_traj_one);
              for(auto sup_traj_one : sup_traj_second)  sup_traj_final.push_back(sup_traj_one);

              // Find the path of the master robot arm to grasp the object to regrasp it by the supportive robot arm ///////////
              std::pair<Eigen::Isometry3d, Eigen::Isometry3d> grasp_pose_rotated; // grasp pose rotated according to the rotated object
              convertRefFrame(grasp_pose, rotated_pose, grasp_pose_rotated); // rotated support grasp pose according to the rotated object pose
              Eigen::Isometry3d master_robot_base, master_grasp_pose, master_pregrasp_pose;
              std::string target_tip_str;
              std::vector<double> master_original_joints_value;
              if(master_robot == rob1_group_name_){
                master_robot_base = motion_.getRobotStatePtr()->getFrameTransform("rob1_base_link");
                // target_tip_str = rob1_precision_point_;
                target_tip_str = rob1_power_point_; // hold the object by power grasp
                master_original_joints_value = rob1_original_joints_value;
              }
              else{
                master_robot_base = motion_.getRobotStatePtr()->getFrameTransform("rob2_base_link");
                // target_tip_str = rob2_precision_point_;
                target_tip_str = rob2_power_point_; // hold the object by power grasp
                master_original_joints_value = rob2_original_joints_value;
              }

              double dist_first = (grasp_pose_rotated.first.translation() - master_robot_base.translation()).norm(); // distance between the base of the master robot and grasp_pose.first
              double dist_second = (grasp_pose_rotated.second.translation() - master_robot_base.translation()).norm(); // distance between the base of the master robot and grasp_pose.second

              // assign the target pose of the master robot 
              if(dist_first > dist_second)  master_grasp_pose = grasp_pose_rotated.second; 
              else                          master_grasp_pose = grasp_pose_rotated.first; 

              Eigen::Isometry3d y_axis_rot = Eigen::Isometry3d::Identity() * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY()); // to rotate tf around y-axis

              // rotate the pose around the y-axis to align z-axis (because the direction of the pose is upward)
              master_grasp_pose = master_grasp_pose * y_axis_rot; //TODO: Just do it if the direction is upward
              setPregraspPose(master_grasp_pose, master_pregrasp_pose);

              // plan a path of the master robot to reach the target pose (middle)
              motion_.getVisualTools()->publishAxis(master_grasp_pose);
              motion_.getVisualTools()->trigger();
              // motion_.getVisualTools()->prompt("master_grasp_pose");
              moveit_msgs::RobotTrajectory master_origin_to_premid, master_premid_to_mid;

              // load object when a joint space jump occurs
              grasp_.genMeshObj(target_object_file_, obj_pose_jump, target_object, false);

              // path (master: initial -> pregrasp of middle pose)
              if(!motion_.moveJoint(master_pregrasp_pose, true, motion_.getJMGPtr(master_robot), target_tip_str, master_origin_to_premid)){
                ROS_WARN_STREAM("Fail to plan a path to grasp the object by the master robot arm to hold it until the supportive robot arm regrasps the object");
                break;
              }
              else{
                motion_.updatePlanningScene();
                
                // path (master: pregrasp of middle pose -> middle pose)
                if(motion_.moveCartesian(master_grasp_pose, true, motion_.getJMGPtr(master_robot), target_tip_str, master_premid_to_mid)){
                  motion_.showRobotMotion(master_premid_to_mid);
                  motion_.updatePlanningScene();

                  // move the master robot to initial configuration
                  motion_.getRobotStatePtr()->setJointGroupPositions(motion_.getJMGPtr(master_robot), master_original_joints_value);
                  motion_.updatePlanningScene();

                  ROS_INFO_STREAM("Path is sucessfully planned. (the master robot arm grasps the object to hold until the supportive robot arm regrasps it)");
                }
                else{
                  // move the master robot to initial configuration
                  motion_.getRobotStatePtr()->setJointGroupPositions(motion_.getJMGPtr(master_robot), master_original_joints_value);
                  motion_.updatePlanningScene();
                  ROS_WARN_STREAM("Fail to plan the path. (the master robot arm grasps the object to hold until the supportive robot arm regrasps it)");
                  break;
                }
              }

              // Divide the trajectory of both robot arms to that of the master robot arm
              moveit_msgs::RobotTrajectory only_master_origin_to_premid, only_master_premid_to_mid;
              if(master_robot == rob1_group_name_){
                only_master_origin_to_premid = master_origin_to_premid;
                only_master_origin_to_premid.joint_trajectory.joint_names.clear();
                only_master_origin_to_premid.joint_trajectory.points.clear();
                only_master_origin_to_premid.joint_trajectory.points.resize(master_origin_to_premid.joint_trajectory.points.size());
                for(int joint_i=0; joint_i<rob1_joint_names.size(); joint_i++)  
                  only_master_origin_to_premid.joint_trajectory.joint_names.push_back(master_origin_to_premid.joint_trajectory.joint_names[joint_i]);
                for(int point_i=0; point_i<master_origin_to_premid.joint_trajectory.points.size(); point_i++){
                  for(int joint_i=0; joint_i<rob1_joint_names.size(); joint_i++)  
                    only_master_origin_to_premid.joint_trajectory.points[point_i].positions.push_back(master_origin_to_premid.joint_trajectory.points[point_i].positions[joint_i]);
                }

                only_master_premid_to_mid = master_premid_to_mid;
                only_master_premid_to_mid.joint_trajectory.joint_names.clear();
                only_master_premid_to_mid.joint_trajectory.points.clear();
                only_master_premid_to_mid.joint_trajectory.points.resize(master_premid_to_mid.joint_trajectory.points.size());
                for(int joint_i=0; joint_i<rob1_joint_names.size(); joint_i++)  
                  only_master_premid_to_mid.joint_trajectory.joint_names.push_back(master_premid_to_mid.joint_trajectory.joint_names[joint_i]);
                for(int point_i=0; point_i<master_premid_to_mid.joint_trajectory.points.size(); point_i++){
                  for(int joint_i=0; joint_i<rob1_joint_names.size(); joint_i++)  
                    only_master_premid_to_mid.joint_trajectory.points[point_i].positions.push_back(master_premid_to_mid.joint_trajectory.points[point_i].positions[joint_i]);
                }
              }
              else{
                only_master_origin_to_premid = master_origin_to_premid;
                only_master_origin_to_premid.joint_trajectory.joint_names.clear();
                only_master_origin_to_premid.joint_trajectory.points.clear();
                only_master_origin_to_premid.joint_trajectory.points.resize(master_origin_to_premid.joint_trajectory.points.size());
                for(int joint_i=rob1_joint_names.size(); joint_i<rob1_joint_names.size()+rob2_joint_names.size(); joint_i++)  
                  only_master_origin_to_premid.joint_trajectory.joint_names.push_back(master_origin_to_premid.joint_trajectory.joint_names[joint_i]);
                for(int point_i=0; point_i<master_origin_to_premid.joint_trajectory.points.size(); point_i++){
                  for(int joint_i=rob1_joint_names.size(); joint_i<rob1_joint_names.size()+rob2_joint_names.size(); joint_i++)  
                    only_master_origin_to_premid.joint_trajectory.points[point_i].positions.push_back(master_origin_to_premid.joint_trajectory.points[point_i].positions[joint_i]);
                }
                only_master_premid_to_mid = master_premid_to_mid;
                only_master_premid_to_mid.joint_trajectory.joint_names.clear();
                only_master_premid_to_mid.joint_trajectory.points.clear();
                only_master_premid_to_mid.joint_trajectory.points.resize(master_premid_to_mid.joint_trajectory.points.size());
                for(int joint_i=rob1_joint_names.size(); joint_i<rob1_joint_names.size()+rob2_joint_names.size(); joint_i++)  
                  only_master_premid_to_mid.joint_trajectory.joint_names.push_back(master_premid_to_mid.joint_trajectory.joint_names[joint_i]);
                for(int point_i=0; point_i<master_premid_to_mid.joint_trajectory.points.size(); point_i++){
                  for(int joint_i=rob1_joint_names.size(); joint_i<rob1_joint_names.size()+rob2_joint_names.size(); joint_i++)  
                    only_master_premid_to_mid.joint_trajectory.points[point_i].positions.push_back(master_premid_to_mid.joint_trajectory.points[point_i].positions[joint_i]);
                }
              }

              std::vector<moveit_msgs::RobotTrajectory> master_middle; // 1)(master: initial -> pregrasp of middle pose) 2)(master: pregrasp of middle pose -> middle pose)
              master_middle.push_back(only_master_origin_to_premid);
              master_middle.push_back(only_master_premid_to_mid);

              rotaxis_objpose_sup_traj_cand = std::make_tuple(point_start, point_end, rotation_mat, rotation_step, std::get<4>(rotaxis_objpose_sup_ik_cands[sup_i])
                                                            , supportive_robot, sup_traj_final, boundary_index, master_middle, obj_pose_jump);
              rotaxis_objpose_sup_traj_cands.push_back(rotaxis_objpose_sup_traj_cand);

              // // Show result of regrasp related path 
              // grasp_.genMeshObj(target_object_file_, obj_pose_jump, target_object, false);
              // motion_.getVisualTools()->publishTrajectoryPath(only_master_origin_to_premid, motion_.getRobotStatePtr());
              // motion_.getVisualTools()->prompt("regrasp(master)- hold the object");
              // // std::vector<robot_state::RobotStatePtr> sup_traj_jump;
              // // for(int i=boundary_index-1; i<boundary_index+2; i++)  sup_traj_jump.push_back(sup_traj_final[i]);
              // // motion_.getVisualTools()->publishTrajectoryPath(sup_traj_jump, motion_.getJMGPtr(supportive_robot));
              // motion_.getVisualTools()->publishTrajectoryPath(sup_traj_final, motion_.getJMGPtr(supportive_robot));
              // motion_.getVisualTools()->prompt("regrasp(supportive)- rotate the object with internal singularity");

              // //Save path data
              // std::vector<double> joint_position_tmp;
              // moveit_msgs::RobotTrajectory regrasp_sup_data;
              // regrasp_sup_data.joint_trajectory.points.resize(sup_traj_final.size());
              // for(int sup_traj_final_i=0; sup_traj_final_i<sup_traj_final.size(); sup_traj_final_i++){
              //   sup_traj_final[sup_traj_final_i]->copyJointGroupPositions(supportive_robot, joint_position_tmp);
              //   regrasp_sup_data.joint_trajectory.points[sup_traj_final_i].positions = joint_position_tmp;
              // }
              // regrasp_sup_data.joint_trajectory.header.seq = sol_i;
              // const std::string package_name(params_.package_name);
              // const std::string package_path(ros::package::getPath(package_name));
              // std::string outfile_string = package_path + "/save_data/regrasp_sup_msg_" + std::to_string(regrasp_sup_data.joint_trajectory.header.seq) + ".txt";
              // std::ofstream outfile(outfile_string);
              // outfile << regrasp_sup_data.joint_trajectory;
              // outfile.close();
              // motion_.getVisualTools()->prompt("regrasp_sup_data test");

            }
            // // Show sup_traj and sup_traj_second (if exist)
            // for(int sup_traj_final_i=0; sup_traj_final_i<sup_traj_final.size(); sup_traj_final_i++){
            //   motion_.getVisualTools()->publishRobotState(*sup_traj_final[sup_traj_final_i].get(), rviz_visual_tools::colors::LIME_GREEN);
            //   motion_.getVisualTools()->prompt("sup_traj_final [" + std::to_string(sup_traj_final_i+1) + "/" + std::to_string(sup_traj_final.size()) + "], boundary: " + std::to_string(boundary_index));
            // }
          }

          sup_traj.clear();

          // // show trail when joint space jump occurs (nearby joint jump)
          // std::vector<robot_state::RobotStatePtr> joint_jump_tmp;
          // for(int jump_i=boundary_index-1; jump_i<boundary_index+1; jump_i++) joint_jump_tmp.push_back(sup_traj_all[jump_i]);
          
          // ROS_WARN_STREAM("sup_traj_all.size(): " << sup_traj_all.size());
          // motion_.getVisualTools()->publishTrajectoryPath(joint_jump_tmp, motion_.getJMGPtr(supportive_robot));
          // //Save path data
          // std::vector<double> joint_position_tmp;
          // moveit_msgs::RobotTrajectory joint_jump_data;
          // joint_jump_data.joint_trajectory.points.resize(sup_traj_all.size());
          // for(int sup_traj_all_i=0; sup_traj_all_i<sup_traj_all.size(); sup_traj_all_i++){
          //   sup_traj_all[sup_traj_all_i]->copyJointGroupPositions(supportive_robot, joint_position_tmp);
          //   joint_jump_data.joint_trajectory.points[sup_traj_all_i].positions = joint_position_tmp;
          // }
          // joint_jump_data.joint_trajectory.header.seq = sol_i;
          // const std::string package_name(params_.package_name);
          // const std::string package_path(ros::package::getPath(package_name));
          // std::string outfile_string = package_path + "/save_data/joint_jump_msg_" + std::to_string(joint_jump_data.joint_trajectory.header.seq) + ".txt";
          // std::ofstream outfile(outfile_string);
          // outfile << joint_jump_data.joint_trajectory;
          // outfile.close();
          // motion_.getVisualTools()->prompt("joint jump test");
          
        }
        else{ // no joint space jump
          ROS_INFO_STREAM("'sup_traj' is valid trajectory");
          rotaxis_objpose_sup_traj_cand = std::make_tuple(point_start, point_end, rotation_mat, rotation_step, std::get<4>(rotaxis_objpose_sup_ik_cands[sup_i])
                                                        , supportive_robot, sup_traj, -1, std::vector<moveit_msgs::RobotTrajectory>(), Eigen::Isometry3d());
          rotaxis_objpose_sup_traj_cands.push_back(rotaxis_objpose_sup_traj_cand);
        }
        // // Show sup_traj_all
        // for(int sup_traj_all_i=0; sup_traj_all_i<sup_traj_all.size(); sup_traj_all_i++){
        //   motion_.getVisualTools()->publishRobotState(*sup_traj_all[sup_traj_all_i].get(), rviz_visual_tools::colors::LIME_GREEN);
        //   motion_.getVisualTools()->prompt("sup_traj_all [" + std::to_string(sup_traj_all_i) + "/" + std::to_string(sup_traj_all.size()) + "]");
        // }

      }
    }
    
    motion_.getVisualTools()->deleteAllMarkers();
    motion_.getVisualTools()->trigger();
  }

  grasp_.genMeshObj(target_object_file_, object_pose, target_object, false); // retrieve original object 
  motion_.getRobotStatePtr()->setJointGroupPositions(motion_.getJMGPtr(robs_group_name_), original_joints_value); // retrieve robot pose
  motion_.updatePlanningScene();
  motion_.getVisualTools()->deleteAllMarkers();
  motion_.getVisualTools()->trigger();

  if(rotaxis_objpose_sup_traj_cands.size() == 0){
    ROS_WARN_STREAM("There is no valid trajectory of supportive robot arm to rotate the object");
    return false;
  }

  return true;
}

// Reverse the point of path
// 1)origin path(robot state) 2)reversed path(robot state)
bool Manipulation::reversePathOrder(std::vector<robot_state::RobotStatePtr> path_origin, std::vector<robot_state::RobotStatePtr>& path_reversed){
  path_reversed = path_origin;
  std::reverse(path_reversed.begin(), path_reversed.end());

  return true;
}

// Reverse the point of path & Do time parameterization
// 1)origin path(moveit_msgs::RobotTrajectory) 2)reversed path(moveit_msgs::RobotTrajectory)
bool Manipulation::reversePathOrder(moveit_msgs::RobotTrajectory path_origin, moveit_msgs::RobotTrajectory& path_reversed){
  path_reversed = path_origin;
  std::reverse(path_reversed.joint_trajectory.points.begin(), path_reversed.joint_trajectory.points.end());
  motion_.parameterizeTime(path_reversed, path_reversed); // Do again time parameterization because the order of point in path is different

  return true;
}

// Planning integrated path
// 1)target object 2)rotated object pose / supportive robot arm(rob1/rob2) / path to rotate object / trajectory index before joint space jump occurs (negative: no jump) 
// / master robot path (middle) / object pose when a joint space jump occurs 3)integrated path 4)gripper state according to sequence number / gripper(rob1_group_name_,rob2_group_name_) / getGripperOpen(),getGripperClose())
bool Manipulation::planPreMPL(moveit_msgs::CollisionObject& target_object
                            , std::vector<std::tuple<Eigen::Isometry3d, std::string, std::vector<robot_state::RobotStatePtr>, int
                                                   , std::vector<moveit_msgs::RobotTrajectory>, Eigen::Isometry3d>>& sup_traj_right
                            , std::vector<std::vector<moveit_msgs::RobotTrajectory>>& integrated_paths
                            , std::multimap<uint32_t, std::pair<std::string, double>>& gripper_motion){
  // Load & set grasp pose
  std::pair<Eigen::Isometry3d, Eigen::Isometry3d> grasp_pose_both_origin, sup_grasp_pose_both_origin, grasp_pose_both, sup_grasp_pose_both
                                                , pregrasp_pose_both_origin, sup_pregrasp_pose_both_origin, pregrasp_pose_both, sup_pregrasp_pose_both; 
  loadGraspPose(grasp_pose_both_origin, sup_grasp_pose_both_origin);
  setPregraspPose(grasp_pose_both_origin, pregrasp_pose_both_origin);
  setPregraspPose(sup_grasp_pose_both_origin, sup_pregrasp_pose_both_origin);
  
  Eigen::Isometry3d object_pose, rotated_pose, sup_presup_pose, master_pregrasp_pose, sup_pregrasp_pose, master_grasp_pose, sup_grasp_pose;
  std::string master_robot, supportive_robot, master_grasp_point, sup_precision_point, sup_grasp_point;
  tf2::fromMsg(target_object.mesh_poses.front(), object_pose); 

  std::vector<std::string> rob1_joint_names = motion_.getJMGPtr(rob1_group_name_)->getActiveJointModelNames(); // active joints of rob1 
  std::vector<std::string> rob2_joint_names = motion_.getJMGPtr(rob2_group_name_)->getActiveJointModelNames(); // active joints of rob2 

  // Save original robot state
  std::vector<double> robs_joints_original;
  motion_.getRobotStatePtr()->copyJointGroupPositions(motion_.getJMGPtr(robs_group_name_), robs_joints_original);

  std::vector<moveit_msgs::RobotTrajectory> integrated_path;
  for(int sup_i=0; sup_i<sup_traj_right.size(); sup_i++){ // one of the supportive robot arm configuration(rotating object)
    int boundary_index = std::get<3>(sup_traj_right[sup_i]);
    if(boundary_index < 0)  ROS_INFO_STREAM("No regrasp motion [" << sup_i+1 << "/" << sup_traj_right.size() << "]");
    else                    ROS_INFO_STREAM("regrasp motion exists [" << sup_i+1 << "/" << sup_traj_right.size() << "]");

    motion_.getRobotStatePtr()->setJointGroupPositions(motion_.getJMGPtr(robs_group_name_), robs_joints_original); // retrieve the robots to original configuration
    motion_.updatePlanningScene(); // TEST
    // motion_.resetSeqNum(); // reset sequence number

    robot_state::RobotStatePtr sup_traj_first(new robot_state::RobotState(*std::get<2>(sup_traj_right[sup_i]).front().get()));
    robot_state::RobotStatePtr sup_traj_last(new robot_state::RobotState(*std::get<2>(sup_traj_right[sup_i]).back().get()));
    supportive_robot = std::get<1>(sup_traj_right[sup_i]);
    rotated_pose = std::get<0>(sup_traj_right[sup_i]);
    convertRefFrame(sup_pregrasp_pose_both_origin, object_pose, sup_pregrasp_pose_both);
    convertRefFrame(pregrasp_pose_both_origin, rotated_pose, pregrasp_pose_both);
    convertRefFrame(grasp_pose_both_origin, rotated_pose, grasp_pose_both);
    grasp_.genMeshObj(target_object_file_, object_pose, target_object, false); // rotating the object

    // plan path initial robot configuration -> pregrasp pose of support grasp -> support grasp pose
    std::vector<double> sup_joints_init, sup_joints_original, sup_joints_rotated, mas_joints_rotated;
    moveit_msgs::RobotTrajectory sup_sup_to_presup, sup_presup_to_original, sup_rotating_obj, mas_origin_to_pregrasp, mas_pregrasp_to_grasp, sup_sup_to_presup_rotated
                               , sup_presup_to_pregrasp, sup_pregrasp_to_grasp
                               , sup_rotating_obj_upper, sup_rotating_obj_lower, mas_origin_to_premid, mas_premid_to_origin, mas_premid_to_mid, mas_mid_to_premid
                               , sup_re1_to_prere1, sup_prere1_to_prere2, sup_prere2_to_re2;
    Eigen::Isometry3d obj_pose_jump; 
    if(supportive_robot == rob2_group_name_){ // supportive robot arm: rob2
      sup_presup_pose = sup_pregrasp_pose_both.second;
      master_pregrasp_pose = pregrasp_pose_both.first;
      master_grasp_pose = grasp_pose_both.first;
      sup_pregrasp_pose = pregrasp_pose_both.second;
      sup_grasp_pose = grasp_pose_both.second;
      master_robot = rob1_group_name_;
      sup_precision_point = rob2_precision_point_;
      sup_grasp_point = rob2_power_point_;
      master_grasp_point = rob1_power_point_;
    }
    else{ // supportive robot arm: rob1
      sup_presup_pose = sup_pregrasp_pose_both.first;
      master_pregrasp_pose = pregrasp_pose_both.second;
      master_grasp_pose = grasp_pose_both.second;
      sup_pregrasp_pose = pregrasp_pose_both.first;
      sup_grasp_pose = grasp_pose_both.first;
      master_robot = rob2_group_name_;
      sup_precision_point = rob1_precision_point_;
      sup_grasp_point = rob1_power_point_;
      master_grasp_point = rob2_power_point_;
    }

    motion_.getRobotStatePtr()->copyJointGroupPositions(motion_.getJMGPtr(supportive_robot), sup_joints_original); // save original supportive robot arm configuration

    sup_traj_first->copyJointGroupPositions(motion_.getJMGPtr(supportive_robot), sup_joints_init);
    motion_.getRobotStatePtr()->setJointGroupPositions(motion_.getJMGPtr(supportive_robot), sup_joints_init); // set robot_state_ as the initial pose of support grasp case
    motion_.updatePlanningScene(); // TEST

    // path (supportive: support grasp -> pregrasp of support grasp)
    if(motion_.moveCartesian(sup_presup_pose, true, motion_.getJMGPtr(supportive_robot), sup_precision_point, sup_sup_to_presup)){ 
      reversePathOrder(sup_sup_to_presup, sup_sup_to_presup); // path (supportive: pregrasp of support grasp -> support grasp)
      gripper_motion.insert(std::make_pair(sup_sup_to_presup.joint_trajectory.header.seq, std::make_pair(supportive_robot, motion_.getGripperOpen())));
      motion_.showRobotMotion(sup_sup_to_presup);
      motion_.updatePlanningScene();

      // path (supportive: pregrasp of support grasp -> original configuration)
      if(motion_.moveJoint(sup_joints_original, motion_.getJMGPtr(supportive_robot), sup_presup_to_original)){ 
        reversePathOrder(sup_presup_to_original, sup_presup_to_original); // path (supportive: original configuration -> pregrasp of support grasp)
        gripper_motion.insert(std::make_pair(sup_presup_to_original.joint_trajectory.header.seq, std::make_pair(supportive_robot, motion_.getGripperOpen())));
        motion_.showRobotMotion(sup_presup_to_original);
        motion_.updatePlanningScene();

        // path (supportive: rotating the object)
        if(boundary_index<0){ // no regrasp motion
          sup_rotating_obj = sup_presup_to_original;
          sup_rotating_obj.joint_trajectory.points.clear();
          sup_rotating_obj.joint_trajectory.points.resize(std::get<2>(sup_traj_right[sup_i]).size());
          for(int point_i=0; point_i<std::get<2>(sup_traj_right[sup_i]).size(); point_i++){
            sup_joints_rotated.clear();
            std::get<2>(sup_traj_right[sup_i])[point_i]->copyJointGroupPositions(motion_.getJMGPtr(supportive_robot), sup_joints_rotated);

            if(supportive_robot == rob2_group_name_){ // supportive robot arm: rob2
              mas_joints_rotated.clear();
              std::get<2>(sup_traj_right[sup_i])[point_i]->copyJointGroupPositions(motion_.getJMGPtr(rob1_group_name_), mas_joints_rotated);
              for(auto mas_joint : mas_joints_rotated)  sup_rotating_obj.joint_trajectory.points[point_i].positions.push_back(mas_joint);
              for(auto sup_joint : sup_joints_rotated)  sup_rotating_obj.joint_trajectory.points[point_i].positions.push_back(sup_joint);
            }
            else{ // supportive robot arm: rob1
              mas_joints_rotated.clear();
              std::get<2>(sup_traj_right[sup_i])[point_i]->copyJointGroupPositions(motion_.getJMGPtr(rob2_group_name_), mas_joints_rotated);
              for(auto sup_joint : sup_joints_rotated)  sup_rotating_obj.joint_trajectory.points[point_i].positions.push_back(sup_joint);
              for(auto mas_joint : mas_joints_rotated)  sup_rotating_obj.joint_trajectory.points[point_i].positions.push_back(mas_joint);
            }
          }
          motion_.parameterizeTime(sup_rotating_obj, sup_rotating_obj);
          motion_.setSeqNum(sup_rotating_obj); // set sequence number
          gripper_motion.insert(std::make_pair(sup_rotating_obj.joint_trajectory.header.seq, std::make_pair(supportive_robot, motion_.getGripperClose())));
          grasp_.genMeshObj(target_object_file_, rotated_pose, target_object, false); // rotating the object
          motion_.getRobotStatePtr()->setJointGroupPositions(motion_.getJMGPtr(robs_group_name_), sup_rotating_obj.joint_trajectory.points.back().positions); // set robot configuration as last one when object is rotated
          motion_.showRobotMotion(sup_rotating_obj);
          motion_.updatePlanningScene();
        }
        else{ // regrasp motion
          // the path of the master robot arm 
          mas_origin_to_premid = std::get<4>(sup_traj_right[sup_i]).front(); // origin to pregrasp of middle
          mas_premid_to_mid = std::get<4>(sup_traj_right[sup_i]).back(); // pregrasp of middle to middle grasp 
          obj_pose_jump = std::get<5>(sup_traj_right[sup_i]); // object pose when a joint space jump occurs

          // If there is no the path of the master robot arm to regrasp the object, skip this step
          if(mas_origin_to_premid.joint_trajectory.points.size() == 0
             || mas_premid_to_mid.joint_trajectory.points.size() == 0){
            ROS_WARN_STREAM("No the path of the master robot arm to perform regrasp");
            continue;
          }

          // initilize trajectory using existing one
          sup_rotating_obj_lower = sup_presup_to_original;
          sup_rotating_obj_lower.joint_trajectory.points.clear();
          sup_rotating_obj_lower.joint_trajectory.points.resize(boundary_index);
          sup_rotating_obj_upper = sup_presup_to_original;
          sup_rotating_obj_upper.joint_trajectory.points.clear();
          sup_rotating_obj_upper.joint_trajectory.points.resize(std::get<2>(sup_traj_right[sup_i]).size() - boundary_index);

          // combine the path of the master robot arm and that of the supportive robot arm
          for(int point_i=0; point_i<boundary_index; point_i++){ // from initial object pose to middle object pose (just befor joint space jump)
            sup_joints_rotated.clear();
            std::get<2>(sup_traj_right[sup_i])[point_i]->copyJointGroupPositions(motion_.getJMGPtr(supportive_robot), sup_joints_rotated);
            if(supportive_robot == rob2_group_name_){ // supportive robot arm: rob2
              mas_joints_rotated.clear();
              std::get<2>(sup_traj_right[sup_i])[point_i]->copyJointGroupPositions(motion_.getJMGPtr(rob1_group_name_), mas_joints_rotated);
              for(auto mas_joint : mas_joints_rotated)  sup_rotating_obj_lower.joint_trajectory.points[point_i].positions.push_back(mas_joint);
              for(auto sup_joint : sup_joints_rotated)  sup_rotating_obj_lower.joint_trajectory.points[point_i].positions.push_back(sup_joint);
            }
            else{ // supportive robot arm: rob1
              mas_joints_rotated.clear();
              std::get<2>(sup_traj_right[sup_i])[point_i]->copyJointGroupPositions(motion_.getJMGPtr(rob2_group_name_), mas_joints_rotated);
              for(auto sup_joint : sup_joints_rotated)  sup_rotating_obj_lower.joint_trajectory.points[point_i].positions.push_back(sup_joint);
              for(auto mas_joint : mas_joints_rotated)  sup_rotating_obj_lower.joint_trajectory.points[point_i].positions.push_back(mas_joint);
            }
          }
          for(int point_i=0; point_i<std::get<2>(sup_traj_right[sup_i]).size()-boundary_index; point_i++){ // from middle object pose to rotated object pose
            sup_joints_rotated.clear();
            std::get<2>(sup_traj_right[sup_i])[boundary_index+point_i]->copyJointGroupPositions(motion_.getJMGPtr(supportive_robot), sup_joints_rotated);
            if(supportive_robot == rob2_group_name_){ // supportive robot arm: rob2
              mas_joints_rotated.clear();
              std::get<2>(sup_traj_right[sup_i])[boundary_index+point_i]->copyJointGroupPositions(motion_.getJMGPtr(rob1_group_name_), mas_joints_rotated);
              for(auto mas_joint : mas_joints_rotated)  sup_rotating_obj_upper.joint_trajectory.points[point_i].positions.push_back(mas_joint);
              for(auto sup_joint : sup_joints_rotated)  sup_rotating_obj_upper.joint_trajectory.points[point_i].positions.push_back(sup_joint);
            }
            else{ // supportive robot arm: rob1
              mas_joints_rotated.clear();
              std::get<2>(sup_traj_right[sup_i])[boundary_index+point_i]->copyJointGroupPositions(motion_.getJMGPtr(rob2_group_name_), mas_joints_rotated);
              for(auto sup_joint : sup_joints_rotated)  sup_rotating_obj_upper.joint_trajectory.points[point_i].positions.push_back(sup_joint);
              for(auto mas_joint : mas_joints_rotated)  sup_rotating_obj_upper.joint_trajectory.points[point_i].positions.push_back(mas_joint);
            }
          }

          // Derive the path of the supportive robot arm to regrasp the object
          // first, set robot state just before joint space jump occurs + master robot: holding the object
          std::vector<double> master_hold_joints = mas_premid_to_mid.joint_trajectory.points.back().positions;
          std::vector<double> supportive_before_jump_joints = sup_rotating_obj_lower.joint_trajectory.points.back().positions;
          std::vector<double> both_hold_jump_joints;
          if(supportive_robot == rob2_group_name_){ // supportive robot arm: rob2
            for(int joint_i=0; joint_i<master_hold_joints.size(); joint_i++)  
              both_hold_jump_joints.push_back(master_hold_joints[joint_i]);
            for(int joint_i=rob1_joint_names.size(); joint_i<supportive_before_jump_joints.size(); joint_i++)  
              both_hold_jump_joints.push_back(supportive_before_jump_joints[joint_i]);
          }
          else{ // supportive robot arm: rob1
            for(int joint_i=0; joint_i<rob1_joint_names.size(); joint_i++)  
              both_hold_jump_joints.push_back(supportive_before_jump_joints[joint_i]);
            for(int joint_i=0; joint_i<master_hold_joints.size(); joint_i++)  
              both_hold_jump_joints.push_back(master_hold_joints[joint_i]);            
          }
          
          motion_.getRobotStatePtr()->setJointGroupPositions(motion_.getJMGPtr(robs_group_name_), both_hold_jump_joints);
          motion_.updatePlanningScene(); 
          
          Eigen::Isometry3d regrasp_pose1, pre_regrasp_pose1;
          regrasp_pose1 = motion_.getRobotStatePtr()->getFrameTransform(sup_precision_point);
          setPregraspPose(regrasp_pose1, pre_regrasp_pose1);

          // (down to middle) move back the robot arm 
          Eigen::Isometry3d z_axis_trans_tf = Eigen::Isometry3d::Identity();
          z_axis_trans_tf.translation() = Eigen::Vector3d(0, 0, -0.05);
          Eigen::Isometry3d pre_regrasp_pose1_small;
          pre_regrasp_pose1_small = regrasp_pose1 * z_axis_trans_tf;

          if(!motion_.moveCartesian(pre_regrasp_pose1_small, true, motion_.getJMGPtr(supportive_robot), sup_precision_point, sup_re1_to_prere1)){
            ROS_WARN_STREAM("No Cartesian path to move back the supportive robot arm");
            continue;
          }
          
          std::vector<double> mas_hold_obj; // the configuration of the master robot arm holding the object
          std::vector<double> sup_rotate_obj; // the configuration of the supportive robot arm rotating the object
          std::vector<double> both_rotate_obj; // the configuration of the both robot arms rotating the object
          double pregrasp_dist;

          // second
          // (middle to up) find pregrasp pose before regrasp the object using the path of rotation. (pregrasp->grasp)
          sup_prere2_to_re2 = sup_rotating_obj_upper; 
          sup_prere2_to_re2.joint_trajectory.points.clear();
          motion_.getRobotStatePtr()->setJointGroupPositions(motion_.getJMGPtr(robs_group_name_), sup_rotating_obj_upper.joint_trajectory.points.front().positions);
          motion_.updatePlanningScene(); // supportive: configuration just after joint space jump (up)
          std::vector<double> last_both_joints = sup_re1_to_prere1.joint_trajectory.points.back().positions;
          Eigen::Isometry3d sup_prere2_pose = motion_.getRobotStatePtr()->getFrameTransform(sup_precision_point);
          Eigen::Isometry3d sup_prere2_pose_now;
          for(int point_i=0; point_i<sup_rotating_obj_upper.joint_trajectory.points.size(); point_i++){
            both_rotate_obj.clear();
            both_rotate_obj = sup_rotating_obj_upper.joint_trajectory.points[point_i].positions;
            sup_prere2_to_re2.joint_trajectory.points.resize(point_i+1);
            
            if(supportive_robot == rob2_group_name_){ // supportive robot arm: rob2
              for(int joint_i=0; joint_i<rob1_joint_names.size(); joint_i++)
                sup_prere2_to_re2.joint_trajectory.points[point_i].positions.push_back(last_both_joints[joint_i]);
              for(int joint_i=rob1_joint_names.size(); joint_i<rob1_joint_names.size()+rob2_joint_names.size(); joint_i++)
                sup_prere2_to_re2.joint_trajectory.points[point_i].positions.push_back(both_rotate_obj[joint_i]);
            }
            else{ // supportive robot arm: rob1
              for(int joint_i=0; joint_i<rob1_joint_names.size(); joint_i++)
                sup_prere2_to_re2.joint_trajectory.points[point_i].positions.push_back(both_rotate_obj[joint_i]);
              for(int joint_i=rob1_joint_names.size(); joint_i<rob1_joint_names.size()+rob2_joint_names.size(); joint_i++)
                sup_prere2_to_re2.joint_trajectory.points[point_i].positions.push_back(last_both_joints[joint_i]);
            }

            // move back until the distance from the middle pose to now is more than 'pregrasp_trans'
            motion_.getRobotStatePtr()->setJointGroupPositions(motion_.getJMGPtr(robs_group_name_), sup_prere2_to_re2.joint_trajectory.points[point_i].positions);
            motion_.updatePlanningScene(); 

            sup_prere2_pose_now = motion_.getRobotStatePtr()->getFrameTransform(sup_precision_point);
            double pre_distance = (sup_prere2_pose.translation() - sup_prere2_pose_now.translation()).norm();

            pregrasp_dist = abs(rob2_pregrasp_trans_);
            if(pre_distance > pregrasp_dist){ // success to get pregrasp motion
              break;
            }
          }
          reversePathOrder(sup_prere2_to_re2, sup_prere2_to_re2); // convert the order of the path 
          motion_.showRobotMotion(sup_prere2_to_re2);
          motion_.updatePlanningScene();

          // move the supportive robot to different configuration (to pregrasp pose) (regrasp main motion)
          motion_.getRobotStatePtr()->setJointGroupPositions(motion_.getJMGPtr(robs_group_name_), sup_prere2_to_re2.joint_trajectory.points.front().positions);
          motion_.updatePlanningScene(); 

          std::vector<double> sup_target_joints; // target joints of the supportive robot arm
          std::vector<double> both_joints = sup_re1_to_prere1.joint_trajectory.points.back().positions;
          if(supportive_robot == rob2_group_name_){ // supportive robot arm: rob2
            for(int sup_arm_i=rob1_joint_names.size(); sup_arm_i<rob1_joint_names.size()+rob2_joint_names.size(); sup_arm_i++)
              sup_target_joints.push_back(both_joints[sup_arm_i]);
          }
          else{ // supportive robot arm: rob1
            for(int sup_arm_i=0; sup_arm_i<rob1_joint_names.size(); sup_arm_i++)
              sup_target_joints.push_back(both_joints[sup_arm_i]);
          }

          // load object where a joint space jump occurs
          grasp_.genMeshObj(target_object_file_, obj_pose_jump, target_object, false);
          
          if(!motion_.moveJoint(sup_target_joints, motion_.getJMGPtr(supportive_robot), sup_prere1_to_prere2)){
            ROS_WARN_STREAM("Fail to plan a regrasp motion");
            continue;
          }
          reversePathOrder(sup_prere1_to_prere2,sup_prere1_to_prere2); // convert to right order

          // //Save path data
          // const std::string package_name(params_.package_name);
          // const std::string package_path(ros::package::getPath(package_name));
          // std::string outfile_string = package_path + "/save_data/regrasp_main_msg_" + std::to_string(sup_i) + ".txt";
          // std::ofstream outfile1(outfile_string);
          // outfile1 << sup_prere1_to_prere2.joint_trajectory;
          // outfile1.close();
          // outfile_string = package_path + "/save_data/regrasp_main_preup_msg_" + std::to_string(sup_i) + ".txt";
          // std::ofstream outfile2(outfile_string);
          // outfile2 << sup_prere2_to_re2.joint_trajectory;
          // outfile2.close();
          // outfile_string = package_path + "/save_data/regrasp_main_predown_msg_" + std::to_string(sup_i) + ".txt";
          // std::ofstream outfile3(outfile_string);
          // outfile3 << sup_re1_to_prere1.joint_trajectory;
          // outfile3.close();
          // motion_.getVisualTools()->prompt("regrasp_main_data test");

          // // Show robot motion of regrasp motion including pregrasp pose approach
          // moveit_msgs::RobotTrajectory regrasp_traj_tmp;
          // regrasp_traj_tmp = sup_re1_to_prere1;
          // for(int i=0; i<sup_prere1_to_prere2.joint_trajectory.points.size(); i++)  regrasp_traj_tmp.joint_trajectory.points.push_back(sup_prere1_to_prere2.joint_trajectory.points[i]);
          // for(int i=0; i<sup_prere2_to_re2.joint_trajectory.points.size(); i++)     regrasp_traj_tmp.joint_trajectory.points.push_back(sup_prere2_to_re2.joint_trajectory.points[i]);
          // motion_.getVisualTools()->deleteAllMarkers();
          // moveit::core::LinkModel* link_right = motion_.getRobotModelPtr()->getLinkModel(rob2_precision_point_);
          // motion_.getVisualTools()->publishTrajectoryLine(regrasp_traj_tmp, link_right, motion_.getJMGPtr(robs_group_name_));
          // motion_.getVisualTools()->publishTrajectoryPath(regrasp_traj_tmp, motion_.getRobotStatePtr());
          // motion_.getVisualTools()->trigger();
          // motion_.getVisualTools()->prompt("regrasp main motion test");

          motion_.clearObjWorld(target_object_file_);

          mas_premid_to_origin = mas_origin_to_premid;
          std::reverse(mas_premid_to_origin.joint_trajectory.points.begin(), mas_premid_to_origin.joint_trajectory.points.end()); // pregrasp of middle to origin
          mas_mid_to_premid = mas_premid_to_mid; 
          std::reverse(mas_mid_to_premid.joint_trajectory.points.begin(), mas_mid_to_premid.joint_trajectory.points.end()); // middle grasp to pregrasp of middle

          // From now, the positions related 'mas_~~' are for both robot arms (master: moving / supportive: stop)
          // 'mas_origin_to_premid'
          std::vector<double> both_joints_bf = sup_rotating_obj_lower.joint_trajectory.points.back().positions; // both robots
          mas_origin_to_premid.joint_trajectory.joint_names.clear();
          mas_origin_to_premid.joint_trajectory.joint_names = sup_rotating_obj_lower.joint_trajectory.joint_names;
          for(int point_i=0; point_i<mas_origin_to_premid.joint_trajectory.points.size(); point_i++){
            std::vector<double> mas_joints = mas_origin_to_premid.joint_trajectory.points[point_i].positions; // only master robot
            mas_origin_to_premid.joint_trajectory.points[point_i].positions.clear();
            if(supportive_robot == rob2_group_name_){ // supportive robot arm: rob2
              for(int joint_i=0; joint_i<rob1_joint_names.size(); joint_i++)
                mas_origin_to_premid.joint_trajectory.points[point_i].positions.push_back(mas_joints[joint_i]);
              for(int joint_i=rob1_joint_names.size(); joint_i<rob1_joint_names.size()+rob2_joint_names.size(); joint_i++)
                mas_origin_to_premid.joint_trajectory.points[point_i].positions.push_back(both_joints_bf[joint_i]);
            }
            else{ // supportive robot arm: rob1
              for(int joint_i=0; joint_i<rob1_joint_names.size(); joint_i++)
                mas_origin_to_premid.joint_trajectory.points[point_i].positions.push_back(both_joints_bf[joint_i]);
              for(int joint_i=0; joint_i<rob2_joint_names.size(); joint_i++)
                mas_origin_to_premid.joint_trajectory.points[point_i].positions.push_back(mas_joints[joint_i]);
            }
          }

          // 'mas_mid_to_premid'
          both_joints_bf.clear();
          both_joints_bf = sup_prere2_to_re2.joint_trajectory.points.back().positions; // both robots
          mas_mid_to_premid.joint_trajectory.joint_names.clear();
          mas_mid_to_premid.joint_trajectory.joint_names = sup_prere2_to_re2.joint_trajectory.joint_names;
          for(int point_i=0; point_i<mas_mid_to_premid.joint_trajectory.points.size(); point_i++){
            std::vector<double> mas_joints = mas_mid_to_premid.joint_trajectory.points[point_i].positions; // only master robot
            mas_mid_to_premid.joint_trajectory.points[point_i].positions.clear();
            if(supportive_robot == rob2_group_name_){ // supportive robot arm: rob2
              for(int joint_i=0; joint_i<rob1_joint_names.size(); joint_i++)
                mas_mid_to_premid.joint_trajectory.points[point_i].positions.push_back(mas_joints[joint_i]);
              for(int joint_i=rob1_joint_names.size(); joint_i<rob1_joint_names.size()+rob2_joint_names.size(); joint_i++)
                mas_mid_to_premid.joint_trajectory.points[point_i].positions.push_back(both_joints_bf[joint_i]);
            }
            else{ // supportive robot arm: rob1
              for(int joint_i=0; joint_i<rob1_joint_names.size(); joint_i++)
                mas_mid_to_premid.joint_trajectory.points[point_i].positions.push_back(both_joints_bf[joint_i]);
              for(int joint_i=0; joint_i<rob2_joint_names.size(); joint_i++)
                mas_mid_to_premid.joint_trajectory.points[point_i].positions.push_back(mas_joints[joint_i]);
            }
          }

          // 'mas_premid_to_origin'
          both_joints_bf.clear();
          both_joints_bf = mas_mid_to_premid.joint_trajectory.points.back().positions; // both robots
          mas_premid_to_origin.joint_trajectory.joint_names.clear();
          mas_premid_to_origin.joint_trajectory.joint_names = mas_mid_to_premid.joint_trajectory.joint_names;
          for(int point_i=0; point_i<mas_premid_to_origin.joint_trajectory.points.size(); point_i++){
            std::vector<double> mas_joints = mas_premid_to_origin.joint_trajectory.points[point_i].positions; // only master robot
            mas_premid_to_origin.joint_trajectory.points[point_i].positions.clear();
            if(supportive_robot == rob2_group_name_){ // supportive robot arm: rob2
              for(int joint_i=0; joint_i<rob1_joint_names.size(); joint_i++)
                mas_premid_to_origin.joint_trajectory.points[point_i].positions.push_back(mas_joints[joint_i]);
              for(int joint_i=rob1_joint_names.size(); joint_i<rob1_joint_names.size()+rob2_joint_names.size(); joint_i++)
                mas_premid_to_origin.joint_trajectory.points[point_i].positions.push_back(both_joints_bf[joint_i]);
            }
            else{ // supportive robot arm: rob1
              for(int joint_i=0; joint_i<rob1_joint_names.size(); joint_i++)
                mas_premid_to_origin.joint_trajectory.points[point_i].positions.push_back(both_joints_bf[joint_i]);
              for(int joint_i=0; joint_i<rob2_joint_names.size(); joint_i++)
                mas_premid_to_origin.joint_trajectory.points[point_i].positions.push_back(mas_joints[joint_i]);
            }
          }

          // 'mas_premid_to_mid'
          both_joints_bf.clear();
          both_joints_bf = mas_origin_to_premid.joint_trajectory.points.back().positions; // both robots
          mas_premid_to_mid.joint_trajectory.joint_names.clear();
          mas_premid_to_mid.joint_trajectory.joint_names = mas_origin_to_premid.joint_trajectory.joint_names;
          for(int point_i=0; point_i<mas_premid_to_mid.joint_trajectory.points.size(); point_i++){
            std::vector<double> mas_joints = mas_premid_to_mid.joint_trajectory.points[point_i].positions; // only master robot
            mas_premid_to_mid.joint_trajectory.points[point_i].positions.clear();
            if(supportive_robot == rob2_group_name_){ // supportive robot arm: rob2
              for(int joint_i=0; joint_i<rob1_joint_names.size(); joint_i++)
                mas_premid_to_mid.joint_trajectory.points[point_i].positions.push_back(mas_joints[joint_i]);
              for(int joint_i=rob1_joint_names.size(); joint_i<rob1_joint_names.size()+rob2_joint_names.size(); joint_i++)
                mas_premid_to_mid.joint_trajectory.points[point_i].positions.push_back(both_joints_bf[joint_i]);
            }
            else{ // supportive robot arm: rob1
              for(int joint_i=0; joint_i<rob1_joint_names.size(); joint_i++)
                mas_premid_to_mid.joint_trajectory.points[point_i].positions.push_back(both_joints_bf[joint_i]);
              for(int joint_i=0; joint_i<rob2_joint_names.size(); joint_i++)
                mas_premid_to_mid.joint_trajectory.points[point_i].positions.push_back(mas_joints[joint_i]);
            }
          }

          // supportive: rotate the object from initial pose to middle pose
          motion_.parameterizeTime(sup_rotating_obj_lower, sup_rotating_obj_lower);
          motion_.setSeqNum(sup_rotating_obj_lower); // set sequence number
          gripper_motion.insert(std::make_pair(sup_rotating_obj_lower.joint_trajectory.header.seq, std::make_pair(supportive_robot, motion_.getGripperClose())));
          grasp_.genMeshObj(target_object_file_, rotated_pose, target_object, false); // rotating the object
          motion_.getRobotStatePtr()->setJointGroupPositions(motion_.getJMGPtr(robs_group_name_), sup_rotating_obj_lower.joint_trajectory.points.back().positions);
          motion_.showRobotMotion(sup_rotating_obj_lower);
          motion_.updatePlanningScene();

          // master: initial to pregrasp of middle
          motion_.parameterizeTime(mas_origin_to_premid, mas_origin_to_premid); // TODO!!!!!!!!!!!!!!!!
          motion_.setSeqNum(mas_origin_to_premid); // set sequence number
          gripper_motion.insert(std::make_pair(mas_origin_to_premid.joint_trajectory.header.seq, std::make_pair(master_robot, motion_.getGripperOpen())));
          grasp_.genMeshObj(target_object_file_, rotated_pose, target_object, false); // rotating the object
          motion_.getRobotStatePtr()->setJointGroupPositions(motion_.getJMGPtr(robs_group_name_), mas_origin_to_premid.joint_trajectory.points.back().positions);
          motion_.showRobotMotion(mas_origin_to_premid);
          motion_.updatePlanningScene();

          // master: pregrasp of middle to hold pose
          motion_.parameterizeTime(mas_premid_to_mid, mas_premid_to_mid);
          motion_.setSeqNum(mas_premid_to_mid); // set sequence number
          gripper_motion.insert(std::make_pair(mas_premid_to_mid.joint_trajectory.header.seq, std::make_pair(master_robot, motion_.getGripperOpen())));
          grasp_.genMeshObj(target_object_file_, rotated_pose, target_object, false); // rotating the object
          motion_.getRobotStatePtr()->setJointGroupPositions(motion_.getJMGPtr(robs_group_name_), mas_premid_to_mid.joint_trajectory.points.back().positions);
          motion_.showRobotMotion(mas_premid_to_mid);
          motion_.updatePlanningScene();

          // supportive: move to pregrasp
          motion_.parameterizeTime(sup_re1_to_prere1, sup_re1_to_prere1);
          motion_.setSeqNum(sup_re1_to_prere1); // set sequence number
          gripper_motion.insert(std::make_pair(sup_re1_to_prere1.joint_trajectory.header.seq, std::make_pair(master_robot, motion_.getGripperClose())));
          gripper_motion.insert(std::make_pair(sup_re1_to_prere1.joint_trajectory.header.seq, std::make_pair(supportive_robot, motion_.getGripperOpen())));
          grasp_.genMeshObj(target_object_file_, rotated_pose, target_object, false); // rotating the object
          motion_.getRobotStatePtr()->setJointGroupPositions(motion_.getJMGPtr(robs_group_name_), sup_re1_to_prere1.joint_trajectory.points.back().positions);
          motion_.showRobotMotion(sup_re1_to_prere1);
          motion_.updatePlanningScene();

          // supportive: pregrasp to pregrasp (regrasp main motion)
          motion_.parameterizeTime(sup_prere1_to_prere2, sup_prere1_to_prere2);
          motion_.setSeqNum(sup_prere1_to_prere2); // set sequence number
          gripper_motion.insert(std::make_pair(sup_prere1_to_prere2.joint_trajectory.header.seq, std::make_pair(master_robot, motion_.getGripperClose())));
          gripper_motion.insert(std::make_pair(sup_prere1_to_prere2.joint_trajectory.header.seq, std::make_pair(supportive_robot, motion_.getGripperOpen())));
          grasp_.genMeshObj(target_object_file_, rotated_pose, target_object, false); // rotating the object
          motion_.getRobotStatePtr()->setJointGroupPositions(motion_.getJMGPtr(robs_group_name_), sup_prere1_to_prere2.joint_trajectory.points.back().positions);
          motion_.showRobotMotion(sup_prere1_to_prere2);
          motion_.updatePlanningScene();

          // supportive: pregrasp to grasp
          motion_.parameterizeTime(sup_prere2_to_re2, sup_prere2_to_re2);
          motion_.setSeqNum(sup_prere2_to_re2); // set sequence number
          gripper_motion.insert(std::make_pair(sup_prere2_to_re2.joint_trajectory.header.seq, std::make_pair(master_robot, motion_.getGripperClose())));
          gripper_motion.insert(std::make_pair(sup_prere2_to_re2.joint_trajectory.header.seq, std::make_pair(supportive_robot, motion_.getGripperOpen())));
          grasp_.genMeshObj(target_object_file_, rotated_pose, target_object, false); // rotating the object
          motion_.getRobotStatePtr()->setJointGroupPositions(motion_.getJMGPtr(robs_group_name_), sup_prere2_to_re2.joint_trajectory.points.back().positions);
          motion_.showRobotMotion(sup_prere2_to_re2);
          motion_.updatePlanningScene();

          // master: grasp to pregrasp
          motion_.parameterizeTime(mas_mid_to_premid, mas_mid_to_premid);
          motion_.setSeqNum(mas_mid_to_premid); // set sequence number
          gripper_motion.insert(std::make_pair(mas_mid_to_premid.joint_trajectory.header.seq, std::make_pair(master_robot, motion_.getGripperOpen())));
          gripper_motion.insert(std::make_pair(mas_mid_to_premid.joint_trajectory.header.seq, std::make_pair(supportive_robot, motion_.getGripperClose())));
          grasp_.genMeshObj(target_object_file_, rotated_pose, target_object, false); // rotating the object
          motion_.getRobotStatePtr()->setJointGroupPositions(motion_.getJMGPtr(robs_group_name_), mas_mid_to_premid.joint_trajectory.points.back().positions);
          motion_.showRobotMotion(mas_mid_to_premid);
          motion_.updatePlanningScene();

          // master: pregrasp to initial configuration
          motion_.parameterizeTime(mas_premid_to_origin, mas_premid_to_origin);
          motion_.setSeqNum(mas_premid_to_origin); // set sequence number
          gripper_motion.insert(std::make_pair(mas_premid_to_origin.joint_trajectory.header.seq, std::make_pair(master_robot, motion_.getGripperOpen())));
          gripper_motion.insert(std::make_pair(mas_premid_to_origin.joint_trajectory.header.seq, std::make_pair(supportive_robot, motion_.getGripperClose())));
          grasp_.genMeshObj(target_object_file_, rotated_pose, target_object, false); // rotating the object
          motion_.getRobotStatePtr()->setJointGroupPositions(motion_.getJMGPtr(robs_group_name_), mas_premid_to_origin.joint_trajectory.points.back().positions);
          motion_.showRobotMotion(mas_premid_to_origin);
          motion_.updatePlanningScene();

          // supportive: rotate the object from middle pose to rotated pose
          motion_.parameterizeTime(sup_rotating_obj_upper, sup_rotating_obj_upper);
          motion_.setSeqNum(sup_rotating_obj_upper); // set sequence number
          gripper_motion.insert(std::make_pair(sup_rotating_obj_upper.joint_trajectory.header.seq, std::make_pair(supportive_robot, motion_.getGripperClose())));
          grasp_.genMeshObj(target_object_file_, rotated_pose, target_object, false); // rotating the object
          motion_.getRobotStatePtr()->setJointGroupPositions(motion_.getJMGPtr(robs_group_name_), sup_rotating_obj_upper.joint_trajectory.points.back().positions);
          motion_.showRobotMotion(sup_rotating_obj_upper);
          motion_.updatePlanningScene();
        }
        
        // path (master: original configuration -> pregrasp of grasp pose)
        if(motion_.moveJoint(master_pregrasp_pose, true, motion_.getJMGPtr(master_robot), master_grasp_point, mas_origin_to_pregrasp)){
          gripper_motion.insert(std::make_pair(mas_origin_to_pregrasp.joint_trajectory.header.seq, std::make_pair(master_robot, motion_.getGripperOpen())));
          motion_.showRobotMotion(mas_origin_to_pregrasp);
          motion_.updatePlanningScene();
          
          // path (master: pregrasp of grasp pose -> grasp pose)
          if(motion_.moveCartesian(master_grasp_pose, true, motion_.getJMGPtr(master_robot), master_grasp_point, mas_pregrasp_to_grasp)){
            gripper_motion.insert(std::make_pair(mas_pregrasp_to_grasp.joint_trajectory.header.seq, std::make_pair(master_robot, motion_.getGripperOpen())));
            motion_.showRobotMotion(mas_pregrasp_to_grasp);
            motion_.updatePlanningScene();
            
            // path (supportive: support grasp -> pregrasp of support pose when object is rotated)
            Eigen::Isometry3d move_back = Eigen::Isometry3d::Identity();
            if(supportive_robot == rob2_group_name_)  move_back.translation().z() = rob2_pregrasp_trans_;
            else                                      move_back.translation().z() = rob1_pregrasp_trans_;
            if(motion_.moveCartesian(move_back, false, motion_.getJMGPtr(supportive_robot), sup_grasp_point, sup_sup_to_presup_rotated)){
              gripper_motion.insert(std::make_pair(sup_sup_to_presup_rotated.joint_trajectory.header.seq, std::make_pair(master_robot, motion_.getGripperClose())));
              gripper_motion.insert(std::make_pair(sup_sup_to_presup_rotated.joint_trajectory.header.seq, std::make_pair(supportive_robot, motion_.getGripperOpen())));
              motion_.showRobotMotion(sup_sup_to_presup_rotated);
              motion_.updatePlanningScene();
              
              // path (supportive: pregrasp of support pose -> pregrasp of grasp pose when object is rotated)
              if(motion_.moveJoint(sup_pregrasp_pose, true, motion_.getJMGPtr(supportive_robot), sup_grasp_point, sup_presup_to_pregrasp)){
                gripper_motion.insert(std::make_pair(sup_presup_to_pregrasp.joint_trajectory.header.seq, std::make_pair(supportive_robot, motion_.getGripperOpen())));
                motion_.showRobotMotion(sup_presup_to_pregrasp);
                motion_.updatePlanningScene();
                
                // path (supportive: pregrasp of grasp pose to grasp pose when object is rotated)
                if(motion_.moveCartesian(sup_grasp_pose, true, motion_.getJMGPtr(supportive_robot), sup_grasp_point, sup_pregrasp_to_grasp)){
                  gripper_motion.insert(std::make_pair(sup_pregrasp_to_grasp.joint_trajectory.header.seq, std::make_pair(supportive_robot, motion_.getGripperOpen())));
                  motion_.showRobotMotion(sup_pregrasp_to_grasp);
                  motion_.updatePlanningScene();

                  // save integrated path 
                  integrated_path.clear();
                  if(boundary_index<0){ // no regrasp motion
                    integrated_path.push_back(sup_presup_to_original);
                    integrated_path.push_back(sup_sup_to_presup);
                    integrated_path.push_back(sup_rotating_obj);
                    integrated_path.push_back(mas_origin_to_pregrasp);
                    integrated_path.push_back(mas_pregrasp_to_grasp);
                    integrated_path.push_back(sup_sup_to_presup_rotated);
                    integrated_path.push_back(sup_presup_to_pregrasp);
                    integrated_path.push_back(sup_pregrasp_to_grasp);
                  }
                  else{ // regrasp motion
                    integrated_path.push_back(sup_presup_to_original);
                    integrated_path.push_back(sup_sup_to_presup);
                    integrated_path.push_back(sup_rotating_obj_lower);
                    integrated_path.push_back(mas_origin_to_premid);
                    integrated_path.push_back(mas_premid_to_mid);
                    integrated_path.push_back(sup_re1_to_prere1);
                    integrated_path.push_back(sup_prere1_to_prere2);
                    integrated_path.push_back(sup_prere2_to_re2);
                    integrated_path.push_back(mas_mid_to_premid);
                    integrated_path.push_back(mas_premid_to_origin);
                    integrated_path.push_back(sup_rotating_obj_upper);
                    integrated_path.push_back(mas_origin_to_pregrasp);
                    integrated_path.push_back(mas_pregrasp_to_grasp);
                    integrated_path.push_back(sup_sup_to_presup_rotated);
                    integrated_path.push_back(sup_presup_to_pregrasp);
                    integrated_path.push_back(sup_pregrasp_to_grasp);
                  }
                  integrated_paths.push_back(integrated_path);
                }
              }
            }
          }
        }
      }
    }
  }

  // Save path & gripper motion as bag file
  if(save_path_bag_){
    preparatory_manipulation::SaveData savedata(params_);

    for(int sol_i=0; sol_i<integrated_paths.size(); sol_i++){
      for(int point_i=0; point_i<integrated_paths[sol_i].size(); point_i++)
        savedata.saveBag(integrated_paths[sol_i][point_i], sol_i, point_i);
    }

    for(auto iter=gripper_motion.begin(); iter!=gripper_motion.end(); iter++){ // it can find values for duplicate keys
      assembly_robot_msgs::gripper_motion gripper_msg;
      gripper_msg.seq_num = iter->first;
      gripper_msg.robot_str = iter->second.first;
      gripper_msg.gripper_joint = iter->second.second;
      
      savedata.saveBag(gripper_msg);
    }
  }
  

  // // show path 
  // for(int i=0; i<integrated_paths.size(); i++){
  //   moveit_msgs::RobotTrajectory traj_tmp;
  //   traj_tmp = std::get<0>(integrated_paths[i]);
  //   motion_.showRobotMotion(traj_tmp);
  //   ROS_WARN_STREAM("traj_tmp.joint_trajectory.header.seq: " << traj_tmp.joint_trajectory.header.seq);
  //   motion_.getVisualTools()->prompt("original -> pregrasp of support grasp");

  //   traj_tmp = std::get<1>(integrated_paths[i]);
  //   motion_.showRobotMotion(traj_tmp);
  //   ROS_WARN_STREAM("traj_tmp.joint_trajectory.header.seq: " << traj_tmp.joint_trajectory.header.seq);
  //   motion_.getVisualTools()->prompt("pregrasp of support grasp -> support grasp");

  //   traj_tmp = std::get<2>(integrated_paths[i]);
  //   motion_.showRobotMotion(traj_tmp);
  //   ROS_WARN_STREAM("traj_tmp.joint_trajectory.header.seq: " << traj_tmp.joint_trajectory.header.seq);
  //   motion_.getVisualTools()->prompt("rotating object");

  //   traj_tmp = std::get<3>(integrated_paths[i]);
  //   motion_.showRobotMotion(traj_tmp);
  //   ROS_WARN_STREAM("traj_tmp.joint_trajectory.header.seq: " << traj_tmp.joint_trajectory.header.seq);
  //   motion_.getVisualTools()->prompt("master: origin -> pregrasp");

  //   traj_tmp = std::get<4>(integrated_paths[i]);
  //   motion_.showRobotMotion(traj_tmp);
  //   ROS_WARN_STREAM("traj_tmp.joint_trajectory.header.seq: " << traj_tmp.joint_trajectory.header.seq);
  //   motion_.getVisualTools()->prompt("master: pregrasp of grasp pose -> grasp pose");
    
  //   traj_tmp = std::get<5>(integrated_paths[i]);
  //   motion_.showRobotMotion(traj_tmp);
  //   ROS_WARN_STREAM("traj_tmp.joint_trajectory.header.seq: " << traj_tmp.joint_trajectory.header.seq);
  //   motion_.getVisualTools()->prompt("supportive: support grasp -> pregrasp of support pose when object is rotated");
    
  //   traj_tmp = std::get<6>(integrated_paths[i]);
  //   motion_.showRobotMotion(traj_tmp);
  //   ROS_WARN_STREAM("traj_tmp.joint_trajectory.header.seq: " << traj_tmp.joint_trajectory.header.seq);
  //   motion_.getVisualTools()->prompt("supportive: pregrasp of support pose -> pregrasp of grasp pose when object is rotated");

  //   traj_tmp = std::get<7>(integrated_paths[i]);
  //   motion_.showRobotMotion(traj_tmp);
  //   ROS_WARN_STREAM("traj_tmp.joint_trajectory.header.seq: " << traj_tmp.joint_trajectory.header.seq);
  //   motion_.getVisualTools()->prompt("supportive: pregrasp of grasp pose to grasp pose when object is rotated");
  // }
  
  return true;
}

// show robot motion
// 1)target object 2)integrated path 3)gripper state according to sequence number(sequence number / gripper(rob1_group_name_,rob2_group_name_) / getGripperOpen(),getGripperClose())
// 4) path number to show(if negative number, show all robot motions)
bool Manipulation::playRobotMotion(moveit_msgs::CollisionObject& target_object
                                 , std::vector<std::vector<moveit_msgs::RobotTrajectory>>& integrated_paths
                                 , std::multimap<uint32_t, std::pair<std::string, double>>& gripper_motion, int path_num){
  // show path 
  moveit_msgs::RobotTrajectory traj_tmp;
  uint32_t seq_num;

  bool for_break = false;
  int ith_path;
  if(path_num < 0){
    ROS_INFO_STREAM("Show all robot motions in simulation");
    ith_path = 0;
  }
  else{
    ROS_INFO_STREAM("Show " << path_num << "th robot motion in simulation");
    ith_path = path_num;
    for_break = true;
  }

  for(int sol_i=ith_path; sol_i<integrated_paths.size(); sol_i++){ // the path of the ingegrated paths
    motion_.moveGripperFinger(robs_group_name_, motion_.getGripperOpen());
    motion_.updatePlanningScene();

    for(int point_i=0; point_i<integrated_paths[sol_i].size(); point_i++){ // the trajectory of the one ingegrated path
      traj_tmp = integrated_paths[sol_i][point_i];
      seq_num = traj_tmp.joint_trajectory.header.seq;
      for(auto iter=gripper_motion.lower_bound(seq_num); iter!=gripper_motion.upper_bound(seq_num); iter++)
        motion_.moveGripperFinger(iter->second.first, iter->second.second);
      motion_.updateGripperPlanningScene();
      motion_.showRobotMotion(traj_tmp);
      ROS_WARN_STREAM("traj_tmp.joint_trajectory.header.seq: " << traj_tmp.joint_trajectory.header.seq);
      motion_.getVisualTools()->prompt("sol_i: [" + std::to_string(sol_i+1) + "/" + std::to_string(integrated_paths.size()) 
                                       + "], point_i: [" + std::to_string(point_i+1) + "/" + std::to_string(integrated_paths[sol_i].size()) + "]");
    }
      motion_.moveGripperFinger(robs_group_name_, motion_.getGripperClose());
      motion_.updateGripperPlanningScene();
      motion_.getVisualTools()->prompt("finish");

    if(for_break) break;
  }
  return true;
}

// Show robot motion all at once (gripper finger motions are not considered)
// 1)target object 2)integrated path 3)gripper state according to sequence number(sequence number / gripper(rob1_group_name_,rob2_group_name_) / getGripperOpen(),getGripperClose())
// 4) path number to show(if negative number, show all robot motions)
bool Manipulation::playRobotMotionAllatOnce(moveit_msgs::CollisionObject& target_object
                                          , std::vector<std::vector<moveit_msgs::RobotTrajectory>>& integrated_paths
                                          , int path_num){
  // show path 
  moveit_msgs::RobotTrajectory traj_tmp;
  uint32_t seq_num;

  bool for_break = false;
  int ith_path;
  if(path_num < 0){
    ROS_INFO_STREAM("Show all robot motions all at once in simulation");
    ith_path = 0;
  }
  else{
    ROS_INFO_STREAM("Show " << path_num << "th robot motion all at once in simulation");
    ith_path = path_num;
    for_break = true;
  }

  for(int sol_i=ith_path; sol_i<integrated_paths.size(); sol_i++){ // the path of the ingegrated paths
    motion_.moveGripperFinger(robs_group_name_, motion_.getGripperOpen());
    motion_.updatePlanningScene();

    traj_tmp = integrated_paths[sol_i].front();
    for(int point_i=1; point_i<integrated_paths[sol_i].size(); point_i++){ // the trajectory of the one ingegrated path
      for(auto traj_point : integrated_paths[sol_i][point_i].joint_trajectory.points)
        traj_tmp.joint_trajectory.points.push_back(traj_point);
    }
    motion_.showRobotMotion(traj_tmp);
    motion_.getVisualTools()->prompt("robot motion all at once");

    if(for_break) break;
  }

  return true;
}

// load robot motion
// 1)parameters 2)target object 3)integrated path 4)gripper state according to sequence number(sequence number / gripper(rob1_group_name_,rob2_group_name_) / getGripperOpen(),getGripperClose())
bool Manipulation::loadRobotMotion(preparatory_manipulation::Params& params, moveit_msgs::CollisionObject& target_object
                                 , std::vector<std::vector<moveit_msgs::RobotTrajectory>>& integrated_paths
                                 , std::multimap<uint32_t, std::pair<std::string, double>>& gripper_motion){
  ROS_INFO_STREAM("Loading robot motion");
  preparatory_manipulation::SaveData savedata(params);
  std::string package_path(ros::package::getPath(params.package_name));

  namespace fs = std::filesystem;
  std::filesystem::path directory_path(package_path + "/save_data/integrated_robot_motion");

  std::string file_common_name = "integrated_path";
  int ahead_num = file_common_name.length();

  // load files of integrated path in directory
  int sol_i_max = 0;
  int traj_i_max = 0;
  std::map<std::pair<int, int>, moveit_msgs::RobotTrajectory> load_traj;
  for(auto& file : fs::recursive_directory_iterator(directory_path)){
    std::string file_name = file.path().filename().string();

    if(file_name.find(file_common_name) != std::string::npos){ // if file name has 'integrated_path'
      std::string::size_type pos_1 = file_name.find('_', ahead_num);
      std::string::size_type pos_2 = file_name.find('_', ahead_num+pos_1);
      int sol_i = std::stoi(file_name.substr(pos_1+1,pos_2-pos_1-1));
      
      std::string::size_type pos_3 = file_name.find_last_of('_');
      std::string::size_type pos_4 = file_name.find_last_of('.');
      int traj_i = std::stoi(file_name.substr(pos_3+1,pos_4-pos_3-1));
      
      if(sol_i > sol_i_max)     sol_i_max = sol_i;
      if(traj_i > traj_i_max)   traj_i_max = traj_i;
      rosbag::Bag bag;
      bag.open(file.path(), rosbag::bagmode::Read);

      for(rosbag::MessageInstance const m : rosbag::View(bag)){
        moveit_msgs::RobotTrajectory::ConstPtr traj = m.instantiate<moveit_msgs::RobotTrajectory>();
        load_traj.insert(std::make_pair(std::make_pair(sol_i, traj_i), *traj.get()));
      }

      bag.close();
    }
  }
  integrated_paths.resize(sol_i_max+1); // find maximum value of sol_i & traj_i. And resize vector

  // set integrated path
  std::pair<int, int> path_index;
  std::vector<moveit_msgs::RobotTrajectory> integrated_path;
  for(int sol_i=0; sol_i<=sol_i_max; sol_i++){
    for(int traj_i=0; traj_i<=traj_i_max; traj_i++){
      path_index = std::make_pair(sol_i, traj_i);
      integrated_path.push_back(load_traj.at(path_index));
    }
  }

  // load files of gripper motion in directory. And set gripper motion
  file_common_name = "gripper_motion";
  std::pair<std::string, double> gripper_str_joint;
  for(auto& file : fs::recursive_directory_iterator(directory_path)){
    std::string file_name = file.path().filename().string();

    if(file_name.find(file_common_name) != std::string::npos){
      rosbag::Bag bag;
      bag.open(file.path(), rosbag::bagmode::Read);

      for(rosbag::MessageInstance const m : rosbag::View(bag)){
        assembly_robot_msgs::gripper_motion::ConstPtr gripper_move = m.instantiate<assembly_robot_msgs::gripper_motion>();
        gripper_str_joint = std::make_pair(gripper_move->robot_str, gripper_move->gripper_joint);
        gripper_motion.insert(std::make_pair(gripper_move->seq_num, gripper_str_joint));
      }
      bag.close();
    }
  }

  return true;
}

// move robot arm & gripper HW
// 1)trajectory of robot arm
void Manipulation::playHW(moveit_msgs::RobotTrajectory& traj){
  playHW(traj, std::multimap<uint32_t, std::pair<std::string, double>>());
}

// move robot arm & gripper HW
// 1)trajectory of robot arm 2)gripper motion
void Manipulation::playHW(moveit_msgs::RobotTrajectory& traj, std::multimap<uint32_t, std::pair<std::string, double>> gripper_motion){
  ROS_INFO_STREAM("=== Execute HW ===");
  // find gripper motion according to sequence number of trajectory
  uint32_t seq_num = traj.joint_trajectory.header.seq;
  std::vector<std::pair<std::string, double>> gripper_motion_tmp;
  for(auto iter=gripper_motion.begin(); iter!=gripper_motion.end(); iter++){ // it can find values for duplicate keys
    if(seq_num == iter->first){ // if this trajectory has gripper motion
      gripper_motion_tmp.push_back(std::make_pair(iter->second.first, iter->second.second));
    }
  }

  // Do closing finger if this sequence has closing & opening finger motion
  if(gripper_motion_tmp.size()==2){
    if(gripper_motion_tmp.front().second > gripper_motion_tmp.back().second){ // the value of closing is bigger than that of open
      moveGripperHW(gripper_motion_tmp.front().first, gripper_motion_tmp.front().second);
      moveGripperHW(gripper_motion_tmp.back().first, gripper_motion_tmp.back().second);
    }
    else{
      moveGripperHW(gripper_motion_tmp.back().first, gripper_motion_tmp.back().second);
      moveGripperHW(gripper_motion_tmp.front().first, gripper_motion_tmp.front().second);
    }
  }
  else if(gripper_motion_tmp.size()==1)   moveGripperHW(gripper_motion_tmp.front().first, gripper_motion_tmp.front().second);
  else if(gripper_motion_tmp.size()>2)    ROS_ERROR_STREAM("'gripper_motion' is not correct so that gripper HW is not move");

  // Split trajectories into ur5e rob1 and rob2
  control_msgs::FollowJointTrajectoryGoal traj_act_goal_rob1;
  control_msgs::FollowJointTrajectoryGoal traj_act_goal_rob2;
  
  control_msgs::JointTolerance path_tolerance, goal_tolerance;
  path_tolerance.name = "path_tolerance";
  path_tolerance.position = 0.005; 
  path_tolerance.velocity = 0.005; 
  path_tolerance.acceleration = 0.001; 
  goal_tolerance.name = "goal_tolerance";
  goal_tolerance.position = 0.005;
  goal_tolerance.velocity = 0.005;
  goal_tolerance.acceleration = 0.001;

  traj_act_goal_rob1.path_tolerance.push_back(path_tolerance);
  traj_act_goal_rob1.goal_tolerance.push_back(goal_tolerance);
  traj_act_goal_rob2.path_tolerance.push_back(path_tolerance);
  traj_act_goal_rob2.goal_tolerance.push_back(goal_tolerance);
  
  traj_act_goal_rob1.trajectory.header = traj_act_goal_rob2.trajectory.header = traj.joint_trajectory.header;
  
  for(int i=0; i<6; i++)
    traj_act_goal_rob1.trajectory.joint_names.push_back(traj.joint_trajectory.joint_names[i]);
  for(int i=6; i<12; i++)
    traj_act_goal_rob2.trajectory.joint_names.push_back(traj.joint_trajectory.joint_names[i]);
  
  int points_size = traj.joint_trajectory.points.size();
  traj_act_goal_rob1.trajectory.points.resize(points_size);
  traj_act_goal_rob2.trajectory.points.resize(points_size);
  for(int index=0; index<points_size; index++){  
    for(int i=0; i<6; i++){
      traj_act_goal_rob1.trajectory.points[index].positions.push_back(traj.joint_trajectory.points[index].positions[i]);
      traj_act_goal_rob1.trajectory.points[index].velocities.push_back(traj.joint_trajectory.points[index].velocities[i]);
      traj_act_goal_rob1.trajectory.points[index].accelerations.push_back(traj.joint_trajectory.points[index].accelerations[i]);
      traj_act_goal_rob1.trajectory.points[index].time_from_start = traj.joint_trajectory.points[index].time_from_start;
    }
    for(int i=6; i<12; i++){
      traj_act_goal_rob2.trajectory.points[index].positions.push_back(traj.joint_trajectory.points[index].positions[i]);
      traj_act_goal_rob2.trajectory.points[index].velocities.push_back(traj.joint_trajectory.points[index].velocities[i]);
      traj_act_goal_rob2.trajectory.points[index].accelerations.push_back(traj.joint_trajectory.points[index].accelerations[i]);
      traj_act_goal_rob2.trajectory.points[index].time_from_start = traj.joint_trajectory.points[index].time_from_start;
    }
  }

  static TrajClient trajectory_act_client_rob1("/rob1/scaled_pos_joint_traj_controller/follow_joint_trajectory", true);
  static TrajClient trajectory_act_client_rob2("/rob2/scaled_pos_joint_traj_controller/follow_joint_trajectory", true);
  ROS_INFO_STREAM("Wait for action server");
  trajectory_act_client_rob1.waitForServer();
  trajectory_act_client_rob2.waitForServer();
  ROS_INFO_STREAM("Action server are connected");

  // Trigger remote mode of UR HW
  std_srvs::Trigger remote_trigger; // Start to connect HW
  ROS_INFO_STREAM("Wait to switch to remote mode of UR5e");
  motion_.getConnectClientRob1().call(remote_trigger);
  motion_.getConnectClientRob2().call(remote_trigger);

  ros::Time begin_trigger = ros::Time::now();
  while(!motion_.URRunningRob1() || !motion_.URRunningRob2()){
    if((ros::Time::now()-begin_trigger).toSec() > 10.0){
      ROS_WARN_STREAM("Time out to switch to remote mode of UR5e. Try agian.");
      motion_.getConnectClientRob1().call(remote_trigger);
      motion_.getConnectClientRob2().call(remote_trigger);
      begin_trigger = ros::Time::now();
    }
    ros::WallDuration(0.5).sleep();
  }
  ROS_INFO_STREAM("UR5e(rob1/rob2) are remote mode");

  trajectory_act_client_rob1.sendGoal(traj_act_goal_rob1);
  trajectory_act_client_rob2.sendGoal(traj_act_goal_rob2);
  ros::Duration time_from_start = traj_act_goal_rob1.trajectory.points.back().time_from_start * 2; // 2: for safety
  double traj_duration = time_from_start.toSec(); 
  
  ROS_INFO_STREAM("Wait for action results until " << traj_duration << "second");
  bool finished_before_timeout_left = trajectory_act_client_rob1.waitForResult(time_from_start);
  bool finished_before_timeout_right = trajectory_act_client_rob2.waitForResult(time_from_start);
  if(finished_before_timeout_left && finished_before_timeout_right){ // success
    ROS_INFO_STREAM("Action client finished");
  }
  else{ // fail
    std::string error_string_left = trajectory_act_client_rob1.getResult()->error_string;
    std::string error_string_right = trajectory_act_client_rob2.getResult()->error_string;
    ROS_WARN_STREAM("Action clients didn't finish before the time out(rob1/rob2): "
                  << error_string_left << " / " << error_string_right);
  }
}

// move gripper HW
// 1)robot name(rob1_group_name_/rob2_group_name_) 2)open(gripper_open_:0.0)/close(gripper_close_:0.8)
bool Manipulation::moveGripperHW(std::string robot_name, double joint){
  if(robot_name!=rob1_group_name_ && robot_name!=rob2_group_name_){
    ROS_ERROR_STREAM("'robot_name' is not correct in moveGripperHW function");
    return false;
  }
  if(joint > motion_.getGripperClose() || joint < motion_.getGripperOpen()){
    ROS_ERROR_STREAM("'joint' value is not correct in moveGripperHW function");
    return false;
  }

  robotiq_2f_gripper_control::Robotiq2FGripper_robot_output gripper_output_msg;
  gripper_output_msg.rACT = 1;
  gripper_output_msg.rGTO = 1;
  gripper_output_msg.rATR = 0;
  gripper_output_msg.rSP = 255; //127;
  gripper_output_msg.rFR = 255; //68;

  int value = static_cast<int>(255.0/motion_.getGripperClose()*joint);
  if(value > 255)     value = 255;
  else if(value < 0)  value = 0;
  gripper_output_msg.rPR = value;
    
  if(robot_name == rob1_group_name_)  motion_.getGrip1Publisher().publish(gripper_output_msg);
  else                                motion_.getGrip2Publisher().publish(gripper_output_msg);
  ros::WallDuration(1.5).sleep();

  return true;
}

// Arrange the target object (Only HW is connected)
// 1)target object
bool Manipulation::arrangeObj(moveit_msgs::CollisionObject& target_object){
  // Load mesh object model(MoveIt)
  Eigen::Isometry3d object_pose = Eigen::Isometry3d::Identity();
  
  // WARN: Object pose should be same as that of 'alg1' ////////////////
  // Check target object
  if(target_object_file_ == "chair_side_part"){ // For chair(STEFAN) part 
    object_pose = Eigen::AngleAxisd(M_PI-0.083, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(M_PI-0.05, Eigen::Vector3d::UnitZ())
                  * Eigen::AngleAxisd(0.005, Eigen::Vector3d::UnitX());
    object_pose.translation() = Eigen::Vector3d(-0.35, 0.52, 0.825);
  }
  else if(target_object_file_ == "shelf_side_part"){ // For shelf part 
    object_pose = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ());
    object_pose.translation() = Eigen::Vector3d(0.2, -0.38, 0.825);
  }
  else if(target_object_file_ == "stool_side_part"){ // For stool part
    object_pose = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ());
    object_pose.translation() = Eigen::Vector3d(0.18, -0.3, 0.835);
  }

  std::vector<std::tuple<EigenSTL::vector_Vector3d, Eigen::Vector3d, unsigned int>> triangle_info; //three vertice and normal vector of the same triangle / triangle number
  shapes::Mesh* mesh_obj = grasp_.genMeshObj(target_object_file_, object_pose, target_object, triangle_info); // in MoveIt
  motion_.getVisualTools()->publishAxisLabeled(object_pose, "object pose");
  motion_.getVisualTools()->trigger();

  // Derive arrange pose
  Eigen::Isometry3d arrange_pose = object_pose;
  Eigen::Isometry3d arrange_prepose, intermediate_pose;
  intermediate_pose = Eigen::Isometry3d::Identity();
  std::string robot_arm_name, precision_point;
  
  // Check target object
  if(target_object_file_ == "chair_side_part"){ // For chair(STEFAN) part 
    arrange_pose *= Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(-0.13, Eigen::Vector3d::UnitX());
  }
  else if(target_object_file_ == "shelf_side_part"){ // For shelf part 
    intermediate_pose = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitZ());
    intermediate_pose.translation().x() += 0.03;
    intermediate_pose.translation().y() -= 0.8;
    intermediate_pose.translation().z() -= 0.01;
    arrange_pose = arrange_pose * intermediate_pose;
  }
  else if(target_object_file_ == "stool_side_part"){ // For stool part
    intermediate_pose = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(M_PI/2+0.08, Eigen::Vector3d::UnitZ());
    intermediate_pose.translation().x() -= 0.025;
    intermediate_pose.translation().y() -= 0.573; 
    intermediate_pose.translation().z() -= 0.015;
    arrange_pose = arrange_pose * intermediate_pose;
  }
  setPregraspPose(arrange_pose, arrange_prepose);
  motion_.getVisualTools()->publishAxisLabeled(arrange_pose, "arrange pose");
  motion_.getVisualTools()->trigger();

  moveit_msgs::RobotTrajectory arrange_traj_1, arrange_traj_2;
  if(!motion_.moveJoint(arrange_pose, true, motion_.getJMGPtr(rob1_group_name_), rob1_precision_point_, arrange_traj_1)){
    if(!motion_.moveJoint(arrange_pose, true, motion_.getJMGPtr(rob2_group_name_), rob2_precision_point_, arrange_traj_1)){
      ROS_WARN_STREAM("Arrange the target object is not possible");
      return false;
    }
    else{ // rob2
      robot_arm_name = rob2_group_name_;
      precision_point = rob2_precision_point_;
    }
  }
  else{ // rob1
    robot_arm_name = rob1_group_name_;
    precision_point = rob1_precision_point_;
  }
  
  motion_.showRobotMotion(arrange_traj_1);
  motion_.updatePlanningScene();
  motion_.getVisualTools()->prompt("before move the robot amr");
  playHW(arrange_traj_1);
  motion_.getVisualTools()->prompt("before arrange the object");
  moveGripperHW(robot_arm_name, motion_.getGripperClose());
  ros::WallDuration(2.0).sleep();
  moveGripperHW(robot_arm_name, motion_.getGripperOpen());
  while(!motion_.moveCartesian(arrange_prepose, true, motion_.getJMGPtr(robot_arm_name), precision_point, arrange_traj_2)){
    ROS_WARN_STREAM("Cartesian path is not valid. Try to derive it again");
  }
  playHW(arrange_traj_2);
  motion_.getVisualTools()->prompt("finish arrange the object");

  return true;
}

// Save rotation data
// 1)target object 2)rotation related data w.r.t global frame (start point/end point/rotation matrix/maximum rotation)
void Manipulation::saveRot(moveit_msgs::CollisionObject& target_object, std::vector<std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d, int>>& rotation_axis_cands){
  ROS_INFO_STREAM("Save rotation data");
  Eigen::Isometry3d object_pose_inv;
  tf2::fromMsg(target_object.mesh_poses.front(), object_pose_inv);
  object_pose_inv = object_pose_inv.inverse();

  // convert the reference frame of the rotation axis to global frame
  std::vector<std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d>> rotation_axis_mat_bf, rotation_axis_mat_af;
  for(int rot_i=0; rot_i<rotation_axis_cands.size(); rot_i++){
    rotation_axis_mat_bf.push_back(std::make_tuple(std::get<0>(rotation_axis_cands[rot_i])
                                                 , std::get<1>(rotation_axis_cands[rot_i]), std::get<2>(rotation_axis_cands[rot_i])));
  }
  Eigen::Isometry3d identity_tf = Eigen::Isometry3d::Identity();
  convertRefFrame(rotation_axis_mat_bf, object_pose_inv, rotation_axis_mat_af);

  // save data
  for(int rot_i=0; rot_i<rotation_axis_mat_af.size(); rot_i++){
    Eigen::Vector3d start_point = std::get<0>(rotation_axis_mat_af[rot_i]);
    Eigen::Vector3d end_point = std::get<1>(rotation_axis_mat_af[rot_i]);
    Eigen::Quaterniond quat_rot(std::get<2>(rotation_axis_mat_af[rot_i]));
    geometry_msgs::Quaternion quat = Eigen::toMsg(quat_rot);
    
    assembly_robot_msgs::rotation_obj rotation_obj;
    rotation_obj.object_name = target_object.id;
    rotation_obj.object_pose = target_object.mesh_poses.front();
    rotation_obj.start_axis = Eigen::toMsg(start_point);
    rotation_obj.end_axis = Eigen::toMsg(end_point);
    rotation_obj.quat_rotation = quat;
    rotation_obj.max_rot_num = std::get<3>(rotation_axis_cands[rot_i]);

    rosbag::Bag bag;
    std::string package_path(ros::package::getPath(params_.package_name));
    std::string outfile_string = package_path + "/save_data/" + target_object_file_ + "_rotation_" + std::to_string(rot_i) + ".bag";
    bag.open(outfile_string, rosbag::bagmode::Write);

    bag.write("rotation_" + std::to_string(rot_i), ros::Time::now(), rotation_obj);
    bag.close();
  }
}

// Load rotation data
// 1)target object 2)rotation related data w.r.t global frame (start point/end point/rotation matrix/maximum rotation)
void Manipulation::loadRot(moveit_msgs::CollisionObject& target_object, std::vector<std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d, int>>& rotation_axis_cands){
  ROS_INFO_STREAM("Load rotation data");
  rosbag::Bag bag;
  std::string package_path(ros::package::getPath(params_.package_name));
  
  namespace fs = std::filesystem;
  std::filesystem::path directory_path(package_path + "/save_data/rotation_" + target_object_file_);

  // load files of rotation data in directory
  std::vector<std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d, int>> rotation_axis_cands_tmp;
  std::vector<std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d>> rotation_axis_mat_bf, rotation_axis_mat_af;
  for(auto& file : fs::recursive_directory_iterator(directory_path)){
    rosbag::Bag bag;
    bag.open(file.path(), rosbag::bagmode::Read);

    for(rosbag::MessageInstance const m : rosbag::View(bag)){
      assembly_robot_msgs::rotation_obj::ConstPtr rotation_obj_constptr = m.instantiate<assembly_robot_msgs::rotation_obj>();
      
      Eigen::Vector3d start_point, end_point;
      Eigen::Quaterniond quat_rot;
      Eigen::fromMsg(rotation_obj_constptr->quat_rotation, quat_rot);
      Eigen::Matrix3d rotation_mat = quat_rot.toRotationMatrix();
      Eigen::fromMsg(rotation_obj_constptr->start_axis, start_point);
      Eigen::fromMsg(rotation_obj_constptr->end_axis, end_point);

      rotation_axis_cands_tmp.push_back(std::make_tuple(start_point, end_point, rotation_mat, rotation_obj_constptr->max_rot_num));
    }

    bag.close();
  }

  for(auto& cands_tmp : rotation_axis_cands_tmp)  
    rotation_axis_mat_bf.push_back(std::make_tuple(std::get<0>(cands_tmp), std::get<1>(cands_tmp), std::get<2>(cands_tmp)));

  // convert the reference frame
  Eigen::Isometry3d object_pose;
  tf2::fromMsg(target_object.mesh_poses.front(), object_pose);
  convertRefFrame(rotation_axis_mat_bf, object_pose, rotation_axis_mat_af);

  for(int rot_i=0; rot_i<rotation_axis_mat_af.size(); rot_i++){
    rotation_axis_cands.push_back(std::make_tuple(std::get<0>(rotation_axis_mat_af[rot_i]), std::get<1>(rotation_axis_mat_af[rot_i])
                                                , std::get<2>(rotation_axis_mat_af[rot_i]), std::get<3>(rotation_axis_cands_tmp[rot_i])));
  }
}

void Manipulation::alg1(moveit_msgs::CollisionObject& target_object, std::vector<std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d, int>>& rotation_axis_cands
                      , ros::WallTime& start_wall_time){
  // Load mesh object model(MoveIt)
  Eigen::Isometry3d object_pose = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d marker_pose; // For ArUco marker pose
  
  // Check target object
  if(!aruco_){ // if 'aruco' is not used
    if(target_object_file_ == "chair_side_part"){ // For chair(STEFAN) part 
      object_pose = Eigen::AngleAxisd(M_PI-0.083, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(M_PI-0.05, Eigen::Vector3d::UnitZ())
                    * Eigen::AngleAxisd(0.005, Eigen::Vector3d::UnitX());
      // object_pose.translation() = Eigen::Vector3d(-0.35, 0.52, 0.825);
      object_pose.translation() = Eigen::Vector3d(-0.25, 0.52, 0.825);
    }
    else if(target_object_file_ == "shelf_side_part"){ // For shelf part 
      object_pose = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ());
      // object_pose.translation() = Eigen::Vector3d(0.2, -0.38, 0.825);
      object_pose.translation() = Eigen::Vector3d(0.3, -0.38, 0.825);
    }
    else if(target_object_file_ == "stool_side_part"){ // For stool part
      object_pose = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ());
      object_pose.translation() = Eigen::Vector3d(0.18, -0.1, 0.835);
    }
  }
  else{ // if 'aruco' is used, that is, vision is included 
    // Move robot to home configuration at first
    motion_.getVisualTools()->prompt("move home configuration");
    moveit_msgs::RobotTrajectory move_home_trj;
    motion_.moveHome(move_home_trj);
    moveGripperHW(rob1_group_name_, motion_.getGripperOpen());
    moveGripperHW(rob2_group_name_, motion_.getGripperOpen());
    playHW(move_home_trj);

    ros::WallDuration(1.0).sleep();
    int32_t fiducial_id;
    searchAruco(marker_pose, fiducial_id);

    // Load(from yaml) the pose of Aruco marker w.r.t object frame
    Eigen::Isometry3d marker_pose_rel_obj; // marker pose w.r.t object frame
    for(int i=0; i<3; i++){
      for(int j=0; j<3; j++){
        marker_pose_rel_obj.linear()(i,j) = marker_rot_[3*i+j];
      }
    }
    marker_pose_rel_obj.translation() = Eigen::Vector3d(marker_trans_[0], marker_trans_[1], marker_trans_[2]);
    marker_pose_rel_obj = marker_pose_rel_obj.inverse();

    // Set object pose
    if(fiducial_id_ == 1){
      target_object_file_ = "chair_side_part";
      object_pose = marker_pose * marker_pose_rel_obj * Eigen::AngleAxisd(0.07, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(0.03, Eigen::Vector3d::UnitX());
      // object_pose = marker_pose * marker_pose_rel_obj;
      object_pose.translation().z() = 0.83;
    } 
    else if(fiducial_id_ == 2){
      target_object_file_ = "shelf_side_part";
      object_pose = marker_pose * marker_pose_rel_obj;
      object_pose.translation().z() = 0.825;
    }  
    else if(fiducial_id_ == 3){
      target_object_file_ = "stool_side_part";
      object_pose = marker_pose * marker_pose_rel_obj;
      object_pose.translation().z() = 0.835;
    }  
    else{
      ROS_ERROR_STREAM("ArUco marker is not correct!");
      target_object_file_ = std::string();
      object_pose = Eigen::Isometry3d::Identity();
    }
  }

  std::vector<std::tuple<EigenSTL::vector_Vector3d, Eigen::Vector3d, unsigned int>> triangle_info; //three vertice and normal vector of the same triangle / triangle number
  shapes::Mesh* mesh_obj = grasp_.genMeshObj(target_object_file_, object_pose, target_object, triangle_info); // in MoveIt

  // motion_.getVisualTools()->publishAxisLabeled(object_pose, "object_pose");
  // motion_.getVisualTools()->publishAxisLabeled(marker_pose, "marker_pose");
  // motion_.getVisualTools()->trigger();
  // motion_.getVisualTools()->prompt("mesh pose");

  start_wall_time = ros::WallTime::now();

  // load rotation data if this mode is on
  if(params_.load_rotation_bag)  loadRot(target_object, rotation_axis_cands);
  else{ // generate rotation data
    // Load mesh object model(Open3D)
    std::vector<std::tuple<Eigen::Vector3d, Eigen::Vector3d>> convex_edges_origin, convex_edges; // both vertice of convex hull edges(origin/convert)
    Eigen::Vector3d center_mesh_model_origin, center_mesh_model; 
    std::shared_ptr<open3d::geometry::TriangleMesh> mesh_model = std::make_shared<open3d::geometry::TriangleMesh>();
    grasp_.genMeshObj(target_object_file_, mesh_model, center_mesh_model_origin); // in Open3D
    convertRefFrame(center_mesh_model_origin, object_pose, center_mesh_model);

    // Derive edges of convex hull
    grasp_.getConvexEdges(mesh_model, convex_edges_origin);
    convertRefFrame(convex_edges_origin, object_pose, convex_edges);

    // // Show edges of convex hull
    // motion_.getVisualTools()->deleteAllMarkers();
    // for(int edge_i=0; edge_i<convex_edges.size(); edge_i++)
    //   motion_.getVisualTools()->publishCylinder(std::get<0>(convex_edges[edge_i]), std::get<1>(convex_edges[edge_i]), rviz_visual_tools::colors::BLUE, 0.005);
    // motion_.getVisualTools()->trigger();
    // motion_.getVisualTools()->prompt("convex edge: " + std::to_string(convex_edges.size()));
    // motion_.getVisualTools()->deleteAllMarkers();

    // // Show edges and its vertices of convex hull including position value
    // for(int edge_i=0; edge_i<convex_edges.size(); edge_i++){
    //   motion_.getVisualTools()->publishCylinder(std::get<0>(convex_edges[edge_i]), std::get<1>(convex_edges[edge_i]), rviz_visual_tools::colors::BLUE, 0.002);
    //   motion_.getVisualTools()->publishSphere(std::get<0>(convex_edges[edge_i]), rviz_visual_tools::colors::LIME_GREEN, rviz_visual_tools::scales::SMALL);
    //   motion_.getVisualTools()->publishSphere(std::get<1>(convex_edges[edge_i]), rviz_visual_tools::colors::LIME_GREEN, rviz_visual_tools::scales::SMALL);
    //   Eigen::Isometry3d text_pose_1 = Eigen::Isometry3d::Identity();
    //   Eigen::Isometry3d text_pose_2 = Eigen::Isometry3d::Identity();
    //   text_pose_1.translation() = std::get<0>(convex_edges[edge_i]) + Eigen::Vector3d(0,0,0.01);
    //   text_pose_2.translation() = std::get<1>(convex_edges[edge_i]) + Eigen::Vector3d(0,0,0.01);

    //   // position w.r.t object frame
    //   motion_.getVisualTools()->publishText(text_pose_1, std::to_string(std::get<0>(convex_edges_origin[edge_i])[0]) + "," 
    //                                         + std::to_string(std::get<0>(convex_edges_origin[edge_i])[1]) + "," +  std::to_string(std::get<0>(convex_edges_origin[edge_i])[2])
    //                                         , rviz_visual_tools::colors::WHITE, rviz_visual_tools::scales::SMALL, false);
    //   motion_.getVisualTools()->publishText(text_pose_2, std::to_string(std::get<1>(convex_edges_origin[edge_i])[0]) + "," 
    //                                         + std::to_string(std::get<1>(convex_edges_origin[edge_i])[1]) + "," +  std::to_string(std::get<1>(convex_edges_origin[edge_i])[2])
    //                                         , rviz_visual_tools::colors::WHITE, rviz_visual_tools::scales::SMALL, false);
      
    //   // // position w.r.t world frame
    //   // motion_.getVisualTools()->publishText(text_pose_1, std::to_string(std::get<0>(convex_edges[edge_i])[0]) + "," 
    //   //                                       + std::to_string(std::get<0>(convex_edges[edge_i])[1]) + "," +  std::to_string(std::get<0>(convex_edges[edge_i])[2])
    //   //                                       , rviz_visual_tools::colors::WHITE, rviz_visual_tools::scales::SMALL, false);
    //   // motion_.getVisualTools()->publishText(text_pose_2, std::to_string(std::get<1>(convex_edges[edge_i])[0]) + "," 
    //   //                                       + std::to_string(std::get<1>(convex_edges[edge_i])[1]) + "," +  std::to_string(std::get<1>(convex_edges[edge_i])[2])
    //   //                                       , rviz_visual_tools::colors::WHITE, rviz_visual_tools::scales::SMALL, false);
    // }
    // motion_.getVisualTools()->trigger();
    // motion_.getVisualTools()->prompt("Show edges, its vertices and position. convex edge: " + std::to_string(convex_edges.size()));
    // motion_.getVisualTools()->deleteAllMarkers();

    // Get rotation axis candidates
    std::vector<std::tuple<Eigen::Vector3d, Eigen::Vector3d>> rotation_axis;
    std::vector<std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d>> rotation_axis_mat; // rotation axis / rotation matrix
    motion_.getRotAxis(convex_edges, rotation_axis);
    motion_.getRotMatObj(rotation_axis, rotation_axis_mat, object_pose); 

    // // Show rotation_axis candidates
    // for(int edge_i=0; edge_i<rotation_axis.size(); edge_i++)
    //   motion_.getVisualTools()->publishCylinder(std::get<0>(rotation_axis[edge_i]), std::get<1>(rotation_axis[edge_i]), rviz_visual_tools::colors::BLUE, 0.005);
    // motion_.getVisualTools()->trigger();
    // motion_.getVisualTools()->prompt("rotation axis candidates: " + std::to_string(rotation_axis.size()));
    // motion_.getVisualTools()->deleteAllMarkers();

    // Derive maximum rotation amount for each rotation axis
    Eigen::Isometry3d rotated_pose;
    std::vector<std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d, int>> rotation_axes_step; // both vertice of rotation axis / rotation matrix / max rotation step
    std::tuple<Eigen::Vector3d, Eigen::Vector3d, int> rotation_axis_step;
    for(int axis_i=0; axis_i<rotation_axis_mat.size(); axis_i++){
      getMaxRotAmount(rotation_axis_mat[axis_i], object_pose, rotation_axis_step, rotated_pose);
      rotation_axes_step.push_back(std::make_tuple(std::get<0>(rotation_axis_step), std::get<1>(rotation_axis_step)
                                , std::get<2>(rotation_axis_mat[axis_i]), std::get<2>(rotation_axis_step)));
    }

    // Ranking the rotatin axis candidates
    priorRotAxis(rotation_axes_step, rotation_axis_cands);
  }

  // save the rotation data if it is needed
  if(params_.save_rotation_bag)  saveRot(target_object, rotation_axis_cands);
}

void Manipulation::alg2(moveit_msgs::CollisionObject& target_object, std::vector<std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d, int>>& rotation_axis_cands
                      , std::vector<std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d, int, Eigen::Isometry3d, std::vector<std::vector<double>>>>& rotaxis_objpose_ik_cands
                      , std::vector<std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d, int, Eigen::Isometry3d, std::string, std::vector<std::vector<double>>>>& rotaxis_objpose_sup_ik_cands){
  // Set grasp pose
  Eigen::Isometry3d object_pose;
  std::pair<Eigen::Isometry3d, Eigen::Isometry3d> grasp_pose_both_origin, sup_grasp_pose_both_origin, grasp_pose_both, sup_grasp_pose_both
                                                , pregrasp_pose_both_origin, sup_pregrasp_pose_both_origin, pregrasp_pose_both, sup_pregrasp_pose_both; 
  loadGraspPose(grasp_pose_both_origin, sup_grasp_pose_both_origin);
  setPregraspPose(grasp_pose_both_origin, pregrasp_pose_both_origin);
  setPregraspPose(sup_grasp_pose_both_origin, sup_pregrasp_pose_both_origin);

  tf2::fromMsg(target_object.mesh_poses.front(), object_pose); 

  // show grasp pose
  convertRefFrame(grasp_pose_both_origin, object_pose, grasp_pose_both);
  convertRefFrame(sup_grasp_pose_both_origin, object_pose, sup_grasp_pose_both);
  convertRefFrame(pregrasp_pose_both_origin, object_pose, pregrasp_pose_both);
  convertRefFrame(sup_pregrasp_pose_both_origin, object_pose, sup_pregrasp_pose_both);
  // motion_.getVisualTools()->publishAxisLabeled(grasp_pose_both.first, "rob1_grasp");
  // motion_.getVisualTools()->publishAxisLabeled(grasp_pose_both.second, "rob2_grasp");
  // // motion_.getVisualTools()->publishAxisLabeled(sup_grasp_pose_both.first, "rob1_sup");
  // // motion_.getVisualTools()->publishAxisLabeled(sup_grasp_pose_both.second, "rob2_sup");
  // // motion_.getVisualTools()->publishAxisLabeled(pregrasp_pose_both.first, "rob1_pregrasp");
  // // motion_.getVisualTools()->publishAxisLabeled(pregrasp_pose_both.second, "rob2_pregrasp");
  // // motion_.getVisualTools()->publishAxisLabeled(sup_pregrasp_pose_both.first, "rob1_sup_pre");
  // // motion_.getVisualTools()->publishAxisLabeled(sup_pregrasp_pose_both.second, "rob2_sup_pre");
  // // motion_.getVisualTools()->publishAxisLabeled(object_pose, "object pose");
  // motion_.getVisualTools()->trigger();
  // motion_.getVisualTools()->prompt("grasp pose");
  // motion_.getVisualTools()->deleteAllMarkers();
  // motion_.getVisualTools()->trigger();

  std::vector<std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d, int>> rotation_axis_dist_cands_tmp, rotation_axis_dist_cands; // rotation axis considering distance from grasp point
  getDisRotAxis(grasp_pose_both, rotation_axis_cands, rotation_axis_dist_cands_tmp); // delete rotation axis candidates which are closed to grasp pose

  // // Load the pose of Aruco marker w.r.t object frame
  // Eigen::Isometry3d marker_pose_origin, marker_pose; // marker pose w.r.t object frame & that of w.r.t global link
  // for(int i=0; i<3; i++){
  //   for(int j=0; j<3; j++){
  //     marker_pose_origin.linear()(i,j) = marker_rot_[3*i+j];
  //   }
  // }
  // marker_pose_origin.translation() = Eigen::Vector3d(marker_trans_[0], marker_trans_[1], marker_trans_[2]);
  // marker_pose_origin = marker_pose_origin.inverse();
  // convertRefFrame(marker_pose_origin, object_pose, marker_pose);
  // getDisRotAxis(marker_pose, rotation_axis_dist_cands_tmp, rotation_axis_dist_cands); // delete rotation axis candidates which are closed to support grasp pose

  getDisRotAxis(sup_grasp_pose_both, rotation_axis_dist_cands_tmp, rotation_axis_dist_cands); // delete rotation axis candidates which are closed to support grasp pose

  // // Show rotation axis candidates which are not close to grasp pose
  // for(int axis_i=0; axis_i<rotation_axis_dist_cands.size(); axis_i++){
  //   motion_.getVisualTools()->publishArrow(tf2::toMsg(std::get<0>(rotation_axis_dist_cands[axis_i])), tf2::toMsg(std::get<1>(rotation_axis_dist_cands[axis_i]))
  //                                        , rviz_visual_tools::colors::LIME_GREEN, rviz_visual_tools::scales::SMALL);
  //   motion_.getVisualTools()->publishSphere(grasp_pose_both.first, rviz_visual_tools::colors::GREEN, rviz_visual_tools::scales::LARGE);
  //   motion_.getVisualTools()->publishSphere(grasp_pose_both.second, rviz_visual_tools::colors::GREEN, rviz_visual_tools::scales::LARGE);
  //   motion_.getVisualTools()->trigger();
  //   motion_.getVisualTools()->prompt("distance test");
  //   motion_.getVisualTools()->deleteAllMarkers();
  //   motion_.getVisualTools()->trigger();
  // }

  std::vector<std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d, int, Eigen::Isometry3d>> rotaxis_objpose_cands;
  getRotObjPose(target_object, grasp_pose_both_origin, pregrasp_pose_both_origin, rotation_axis_dist_cands, rotaxis_objpose_cands);

  // // show object pose and dummy gripper
  // moveit_msgs::CollisionObject dummy_gripper1, dummy_gripper2;
  // ROS_WARN_STREAM("rotaxis_objpose_cands.size(): " << rotaxis_objpose_cands.size());
  // for(int axis_i=0; axis_i<rotaxis_objpose_cands.size(); axis_i++){
  //   Eigen::Vector3d point_start = std::get<0>(rotaxis_objpose_cands[axis_i]);
  //   Eigen::Vector3d point_end = std::get<1>(rotaxis_objpose_cands[axis_i]);
  //   Eigen::Isometry3d rotated_pose = std::get<4>(rotaxis_objpose_cands[axis_i]);
  //   grasp_.genMeshObj(target_object_file_, rotated_pose, target_object, false);
  //   motion_.genDummyGripper(dummy_gripper1, "dummy_gripper1", "closed", grasp_pose_both_origin.first, rotated_pose, rob1_power_point_, false);
  //   motion_.genDummyGripper(dummy_gripper2, "dummy_gripper2", "closed", grasp_pose_both_origin.second, rotated_pose, rob2_power_point_, false);
  //   motion_.getVisualTools()->publishArrow(tf2::toMsg(point_start), tf2::toMsg(point_end), rviz_visual_tools::colors::LIME_GREEN, rviz_visual_tools::scales::SMALL);
  //   motion_.getVisualTools()->trigger();
  //   motion_.getVisualTools()->prompt("derived object pose: " + std::to_string(axis_i));
  //   motion_.getVisualTools()->deleteAllMarkers();
  //   motion_.getVisualTools()->trigger();
  //   motion_.clearObjWorld("dummy_gripper1", false);
  //   motion_.clearObjWorld("dummy_gripper2", false);
  // }

  // // show rotated object & frames of power grasp pose and precision grasp pose
  // motion_.clearObjWorld("dummy_gripper1", false);
  // motion_.clearObjWorld("dummy_gripper2", false);
  // for(int axis_i=0; axis_i<rotaxis_objpose_cands.size(); axis_i++){
  //   Eigen::Isometry3d rotated_pose = std::get<4>(rotaxis_objpose_cands[axis_i]);
  //   auto rotation_axis_mat = std::make_tuple(std::get<0>(rotaxis_objpose_cands[axis_i])
  //                                           ,std::get<1>(rotaxis_objpose_cands[axis_i]),std::get<2>(rotaxis_objpose_cands[axis_i]));
  //   motion_.clearObjWorld(target_object_file_);
  //   for(int i=0; i<std::get<3>(rotaxis_objpose_cands[axis_i]); i++){
  //     motion_.rotateObj(rotation_axis_mat, i, object_pose, rotated_pose);
  //     motion_.genMeshObjTrans(target_object_file_, rotated_pose, i);
  //     // motion_.getVisualTools()->publishAxis(rotated_pose*grasp_pose_both_origin.first);
  //     // motion_.getVisualTools()->publishAxis(rotated_pose*grasp_pose_both_origin.second);
  //     // motion_.getVisualTools()->publishAxis(rotated_pose*pregrasp_pose_both_origin.first);
  //     // motion_.getVisualTools()->publishAxis(rotated_pose*pregrasp_pose_both_origin.second);
  //     motion_.getVisualTools()->publishAxis(rotated_pose*sup_grasp_pose_both_origin.first);
  //   }
  //   // motion_.getVisualTools()->publishAxis(rotated_pose*sup_grasp_pose_both_origin.first);
  //   motion_.getVisualTools()->trigger();
  //   motion_.getVisualTools()->prompt("show steps of rotated object and frame"); 
  //   for(int j=0; j<std::get<3>(rotaxis_objpose_cands[axis_i]); j++){
  //     motion_.clearObjWorld(target_object_file_ + "_" + std::to_string(j));
  //   }
  //   motion_.getVisualTools()->deleteAllMarkers();
  // }

  // // show rotated object (goal pose)
  // motion_.clearObjWorld("dummy_gripper1", false);
  // motion_.clearObjWorld("dummy_gripper2", false);
  // for(int axis_i=0; axis_i<rotaxis_objpose_cands.size(); axis_i++){
  //   Eigen::Vector3d point_start = std::get<0>(rotaxis_objpose_cands[axis_i]);
  //   Eigen::Vector3d point_end = std::get<1>(rotaxis_objpose_cands[axis_i]);
  //   motion_.getVisualTools()->publishArrow(tf2::toMsg(point_start), tf2::toMsg(point_end), rviz_visual_tools::colors::LIME_GREEN, rviz_visual_tools::scales::SMALL);

  //   Eigen::Isometry3d rotated_pose = std::get<4>(rotaxis_objpose_cands[axis_i]);

  //   grasp_.genMeshObj(target_object_file_, rotated_pose, target_object, false);
  //   motion_.getVisualTools()->trigger();
  //   motion_.getVisualTools()->prompt("show rotated goal pose of the object[" + std::to_string(axis_i+1) + "/" + std::to_string(rotaxis_objpose_cands.size()) + "]"); 
  //   motion_.getVisualTools()->deleteAllMarkers();
  //   motion_.getVisualTools()->trigger();
  // }
  

  getIKObj(target_object, grasp_pose_both_origin, rotaxis_objpose_cands, rotaxis_objpose_ik_cands);
  
  getSupIKObj(target_object, grasp_pose_both_origin, sup_grasp_pose_both_origin, rotaxis_objpose_ik_cands, rotaxis_objpose_sup_ik_cands);

}

void Manipulation::alg3(moveit_msgs::CollisionObject& target_object
                      , std::vector<std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d, int, Eigen::Isometry3d, std::vector<std::vector<double>>>>& rotaxis_objpose_ik_cands
                      , std::vector<std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d, int, Eigen::Isometry3d, std::string, std::vector<std::vector<double>>>>& rotaxis_objpose_sup_ik_cands
                      , std::vector<std::vector<moveit_msgs::RobotTrajectory>>& integrated_paths
                      , std::multimap<uint32_t, std::pair<std::string, double>>& gripper_motion){
  // Load & set grasp pose
  std::pair<Eigen::Isometry3d, Eigen::Isometry3d> grasp_pose_both_origin, sup_grasp_pose_both_origin, grasp_pose_both, sup_grasp_pose_both
                                                , pregrasp_pose_both_origin, sup_pregrasp_pose_both_origin, pregrasp_pose_both, sup_pregrasp_pose_both; 
  loadGraspPose(grasp_pose_both_origin, sup_grasp_pose_both_origin);
  setPregraspPose(grasp_pose_both_origin, pregrasp_pose_both_origin);
  setPregraspPose(sup_grasp_pose_both_origin, sup_pregrasp_pose_both_origin);

  Eigen::Isometry3d object_pose, rotated_pose;
  Eigen::Vector3d point_start, point_end;
  Eigen::Matrix3d rotation_mat;
  tf2::fromMsg(target_object.mesh_poses.front(), object_pose); 

  std::vector<std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d, int, Eigen::Isometry3d, std::string
            , std::vector<robot_state::RobotStatePtr>, int, std::vector<moveit_msgs::RobotTrajectory>, Eigen::Isometry3d>> rotaxis_objpose_sup_traj_cands;
  getSupRobPath(target_object, sup_grasp_pose_both_origin, grasp_pose_both_origin, rotaxis_objpose_sup_ik_cands, rotaxis_objpose_sup_traj_cands);

  std::vector<std::tuple<Eigen::Isometry3d, std::string, std::vector<robot_state::RobotStatePtr>, int
            , std::vector<moveit_msgs::RobotTrajectory>, Eigen::Isometry3d>> sup_traj_right; // the path of support robot arm to rotate object, and its order is right order
  std::vector<robot_state::RobotStatePtr> sup_traj_tmp; // set of the supportive robot arm to rotate the object
  std::string supportive_robot;
  
  for(int path_i=0; path_i<rotaxis_objpose_sup_traj_cands.size(); path_i++){
    sup_traj_tmp.clear();
    supportive_robot = std::get<5>(rotaxis_objpose_sup_traj_cands[path_i]);
    rotated_pose = std::get<4>(rotaxis_objpose_sup_traj_cands[path_i]);
    reversePathOrder(std::get<6>(rotaxis_objpose_sup_traj_cands[path_i]), sup_traj_tmp); // set right order of the path
    int boundary_index_right = std::get<7>(rotaxis_objpose_sup_traj_cands[path_i]);
    if(!(boundary_index_right < 0)) // positive: a regrasp motion exists (negative: no regrasp motion)
      boundary_index_right =  std::get<6>(rotaxis_objpose_sup_traj_cands[path_i]).size() - boundary_index_right; // change boundary_index because the order of the path is changed
    sup_traj_right.push_back(std::make_tuple(rotated_pose, supportive_robot, sup_traj_tmp, boundary_index_right
                           , std::get<8>(rotaxis_objpose_sup_traj_cands[path_i]), std::get<9>(rotaxis_objpose_sup_traj_cands[path_i])));
  }

  // // Show supportive robot path to rotate object
  // ROS_WARN_STREAM("valid sup traj: " << sup_traj_right.size());
  // for(int i=0; i<sup_traj_right.size(); i++){
  //   std::vector<robot_state::RobotStatePtr> robot_state_trajs = std::get<2>(sup_traj_right[i]);
  //   ROS_WARN_STREAM("path size: " << robot_state_trajs.size() << ", boundary trajectory index: " << std::get<3>(sup_traj_right[i]));
  //   for(int j=0; j<robot_state_trajs.size(); j++){
  //     robot_state_trajs[j]->printStatePositions();
  //     motion_.getVisualTools()->publishRobotState(*robot_state_trajs[j].get(), rviz_visual_tools::colors::RED);
  //     motion_.getVisualTools()->trigger();
  //     motion_.getVisualTools()->prompt("supportive robot path");
  //   }
  // }


  planPreMPL(target_object, sup_traj_right, integrated_paths, gripper_motion);

}

void Manipulation::example(){
  motion_.getVisualTools()->publishAxisLabeled(motion_.getRobotStatePtr()->getFrameTransform("rob1_precision_point"), "precision");
  motion_.getVisualTools()->publishAxisLabeled(motion_.getRobotStatePtr()->getFrameTransform("rob1_power_point"), "power");
  motion_.getVisualTools()->trigger();
  motion_.getVisualTools()->prompt("test");

}

void Manipulation::MarkerLocationTest(){ 
  // Marker location test
  Eigen::Isometry3d marker_pose;
  int32_t fiducial_id;
  searchAruco(marker_pose, fiducial_id, 0.87);

  moveit_msgs::RobotTrajectory derived_traj;
  motion_.moveJoint(marker_pose, true, motion_.getJMGPtr(rob2_group_name_), rob2_precision_point_, derived_traj);
  motion_.showRobotMotion(derived_traj);
  motion_.updatePlanningScene();
  motion_.getVisualTools()->prompt("motion test");

  moveGripperHW(rob1_group_name_, motion_.getGripperClose());
  moveGripperHW(rob2_group_name_, motion_.getGripperClose());
  playHW(derived_traj);

  motion_.getVisualTools()->prompt("move home configuration");
  moveit_msgs::RobotTrajectory move_home_trj;
  motion_.moveHome(move_home_trj);
  moveGripperHW(rob1_group_name_, motion_.getGripperOpen());
  moveGripperHW(rob2_group_name_, motion_.getGripperOpen());
  playHW(move_home_trj);

  motion_.getVisualTools()->prompt("test finished");

}

} // namespace



int main(int argc, char** argv){
  ros::init(argc, argv, "manipulation");

  // Allow the action server to recieve and send ros messagestriangle_count
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle nh("~");
  preparatory_manipulation::Params params;
  preparatory_manipulation::loadParam(nh, params);
  preparatory_manipulation::MotionPlanning motion(nh, params);
  preparatory_manipulation::GraspGeneration grasp(motion, params);
  preparatory_manipulation::Manipulation mpl(nh, motion, grasp, params);

  // check if HW is connect
  bool hw_connect;
  if (!nh.hasParam("hw_connect")){
    ROS_WARN_STREAM("No param named 'hw_connect'. Assuming 'hw_connect' is false");
    hw_connect = false;
  }
  else    nh.getParam("hw_connect", hw_connect);

  // mpl.example();
  // mpl.MarkerLocationTest();
  // motion.test();  

  moveit_msgs::CollisionObject target_object;
  std::vector<std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d, int>> rotation_axis_cands;
  std::vector<std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d, int, Eigen::Isometry3d, std::vector<std::vector<double>>>> rotaxis_objpose_ik_cands;
  std::vector<std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d, int, Eigen::Isometry3d, std::string, std::vector<std::vector<double>>>> rotaxis_objpose_sup_ik_cands;
  std::vector<std::vector<moveit_msgs::RobotTrajectory>> integrated_paths;
  std::multimap<uint32_t, std::pair<std::string, double>> gripper_motion;

  ros::WallTime start_wall_time; // for check time elapsed after vision step
  
  if(!params.load_path_bag){ // planning path
    mpl.alg1(target_object, rotation_axis_cands, start_wall_time);
    mpl.alg2(target_object, rotation_axis_cands, rotaxis_objpose_ik_cands, rotaxis_objpose_sup_ik_cands);
    mpl.alg3(target_object, rotaxis_objpose_ik_cands, rotaxis_objpose_sup_ik_cands, integrated_paths, gripper_motion);
  }
  else{ // load planned path
    mpl.loadRobotMotion(params, target_object, integrated_paths, gripper_motion);
  }
  ROS_WARN_STREAM("time elapsed: " << (ros::WallTime::now()-start_wall_time).toSec());

  motion.getVisualTools()->prompt("play robot motion in simulation");
  mpl.playRobotMotion(target_object, integrated_paths, gripper_motion); // show all integrated paths in simulation
  // mpl.playRobotMotion(target_object, integrated_paths, gripper_motion, params.load_ith_path); // show ith integrated path in simulation
  // mpl.playRobotMotionAllatOnce(target_object, integrated_paths, params.load_ith_path); // show ith integrated path all at once in simulation

  // If hw is connected
  std::string rob1_group_name(params.rob1_group_name);
  std::string rob2_group_name(params.rob2_group_name);
  moveit_msgs::RobotTrajectory move_home_trj;
  if(hw_connect){
    if(params.arrange_object){ // if the function of align the object is activated
      ROS_INFO_STREAM("Align the target object");
      // At first, move the robot into initial robot configuration
      motion.moveHome(move_home_trj);
      mpl.moveGripperHW(rob1_group_name, motion.getGripperOpen());
      mpl.moveGripperHW(rob2_group_name, motion.getGripperOpen());
      mpl.playHW(move_home_trj);
      motion.getVisualTools()->prompt("move home configuration(align the object)");

      mpl.arrangeObj(target_object);
      
    }

    // Check if there is a regrasp motion motion
    std::vector<int> regrasp_index; // 0: no regrasp, 1: regrasp motion
    for(int sol_i=0; sol_i<integrated_paths.size(); sol_i++){
      if(integrated_paths[sol_i].size() > 8)  regrasp_index.push_back(sol_i);
    }

    ROS_INFO_STREAM("Start manipulation");

    bool for_break = false;
    int ith_path;
    if(params.priority_regrasp){
      ROS_INFO_STREAM("Regrasp motion priority mode");
      if(regrasp_index.size() != 0){
        ROS_INFO_STREAM("Regrasp motion exists");
        ith_path = regrasp_index.front();
        for_break = true;
      }
      else{
        ROS_INFO_STREAM("No regrasp motion exists");
        if(params.load_ith_path < 0){
          ROS_INFO_STREAM("Play HW all of robot motions");
          ith_path = 0;
        }
        else{
          ROS_INFO_STREAM("Play HW " << params.load_ith_path << "th robot motion");
          ith_path = params.load_ith_path;
          for_break = true;
        }
      }
    }
    else{
      ROS_INFO_STREAM("Normal mode");
      ROS_INFO_STREAM("No regrasp motion exists");
      if(params.load_ith_path < 0){
        ROS_INFO_STREAM("Play HW all of robot motions");
        ith_path = 0;
      }
      else{
        ROS_INFO_STREAM("Play HW " << params.load_ith_path << "th robot motion");
        ith_path = params.load_ith_path;
        for_break = true;
      }
    }

    // move the object up to avoid interfering with path planning
    Eigen::Isometry3d object_away = Eigen::Isometry3d::Identity();
    object_away.translation().z() = 10;
    grasp.genMeshObj(target_object.id, object_away, target_object, false);
    
    // At first, move the robot into initial robot configuration
    motion.moveHome(move_home_trj);
    mpl.moveGripperHW(rob1_group_name, motion.getGripperOpen());
    mpl.moveGripperHW(rob2_group_name, motion.getGripperOpen());
    mpl.playHW(move_home_trj);
    motion.getVisualTools()->prompt("move home configuration");
    
    for(int sol_i=ith_path; sol_i<integrated_paths.size(); sol_i++){
      for(int point_i=0; point_i<integrated_paths[sol_i].size(); point_i++){
        mpl.playHW(integrated_paths[sol_i][point_i], gripper_motion);
        motion.getVisualTools()->prompt("HW traj[" + std::to_string(sol_i) + "]: " + std::to_string(point_i));
      }

      mpl.moveGripperHW(rob1_group_name, motion.getGripperClose()); // Add closing motion of gripper
      mpl.moveGripperHW(rob2_group_name, motion.getGripperClose()); // Add closing motion of gripper
      motion.getVisualTools()->prompt("traj is done");

      ros::WallDuration(2.0).sleep();
      mpl.moveGripperHW(rob1_group_name, motion.getGripperOpen());
      mpl.moveGripperHW(rob2_group_name, motion.getGripperOpen());

      // TODO: move to home position if the robot arm move back at first
      // if(motion.moveHome(move_home_trj)){
      //   mpl.playHW(move_home_trj);
      //   motion.getVisualTools()->prompt("move home configuration");
      // }
      // else{
      //   ROS_ERROR_STREAM("Move to home configuration is not possible");
      // }

      if(for_break) break;
    }
    
  }

  ros::shutdown();
  return 0;
}