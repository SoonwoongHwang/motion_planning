#include <hyundai_robot_manipulation/motion_planning.h>

namespace motion_planning{
MotionPlanning::MotionPlanning(ros::NodeHandle& nh) : nh_(nh)
  , robot_group_name_("HH020_motion")
  , global_link_("world")
  , robot_model_loader_(new robot_model_loader::RobotModelLoader("robot_description"))
  , robot_model_(robot_model_loader_->getModel())
  , robot_state_(new robot_state::RobotState(robot_model_))
  , planning_scene_(new planning_scene::PlanningScene(robot_model_))
  , psm_(new planning_scene_monitor::PlanningSceneMonitor(planning_scene_, robot_model_loader_, nh_))
  , planning_pipeline_(new planning_pipeline::PlanningPipeline(robot_model_, nh_, "planning_plugin", "request_adapters"))
  , sleep_t_(0.1)
  , obj_name_(std::string())
  , obj_x_(0.0)
  , obj_y_(0.0)
  , obj_z_(0.0)
{
  jmg_ = robot_model_->getJointModelGroup(robot_group_name_);
  visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools(global_link_, rviz_visual_tools::RVIZ_MARKER_TOPIC));
  initEvn();
}

void MotionPlanning::initEvn(){
  // Advertise the required topic
  // create a publisher and wait for subscribers to visualize the object(planning_scene)
  planning_scene_diff_publisher_ = nh_.advertise<moveit_msgs::PlanningScene>("/planning_scene", 10);
  while (planning_scene_diff_publisher_.getNumSubscribers() < 1)  sleep_t_.sleep();
  planning_scene_diff_publisher_.publish(planning_scene_msgs_);

  psm_->startSceneMonitor("/planning_scene"); // start to monitor planning_scene based on this topic
  sleep_t_.sleep();
  psm_->providePlanningSceneService("/get_planning_scene"); // Service server to response finish of applying planning scene
  sleep_t_.sleep();
  planning_scene_diff_client_ = nh_.serviceClient<moveit_msgs::GetPlanningScene>("/get_planning_scene");
  planning_scene_diff_client_.waitForExistence(); // Wait until applying planning_scene

  // Advertise the required topic
  // create a publisher and wait for subscribers to send contact points
  marker_array_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>("/interactive_robot_array", 100);
  while (marker_array_publisher_.getNumSubscribers() < 1){
    sleep_t_.sleep();
  }
  marker_array_publisher_.publish(collision_points_); // Update joint state(tf)

  // initialize visualization
  visual_tools_->loadMarkerPub();
  visual_tools_->loadRemoteControl();
  visual_tools_->loadRobotStatePub("/display_robot_state"); 
  visual_tools_->loadTrajectoryPub("/display_planned_path");

  // initialize robot joints name & value
  robot_joint_names_ = jmg_->getActiveJointModelNames(); // active joints of rob1 and rob2 
  robot_joint_init_ = {0.0, M_PI/2, 0.0, 0.0, 0.0, 0.0};
  // robot_joint_init_ = {-1.48722, -0.0109338, 0.0688651, 0.0, 0.0, 0.0};

  // set planning scene initial state
  moveit_msgs::GetPlanningScene wait_srv;
  moveit_msgs::PlanningScene planning_scene_msg;
  for(int i=0; i<robot_joint_names_.size(); i++)  planning_scene_msg.robot_state.joint_state.name.push_back(robot_joint_names_[i]);
  for(int i=0; i<robot_joint_init_.size(); i++)   planning_scene_msg.robot_state.joint_state.position.push_back(robot_joint_init_[i]);
  
  planning_scene_msg.is_diff = true;
  planning_scene_msg.robot_state.is_diff = true;
  planning_scene_msg.robot_state.joint_state.header.stamp = ros::Time::now();
  planning_scene_diff_publisher_.publish(planning_scene_msg);
  planning_scene_diff_client_.call(wait_srv); // to wait until applying planning scene

  // initialize robot_state_
  robot_state_->setJointGroupPositions(jmg_, robot_joint_init_);

  //Subscriber setting
  picking_vision_coordinate_ = nh_.subscribe("/recognized_obj_Info", 10, &MotionPlanning::visionMsgCallback, this);
  sleep_t_.sleep();

  aruco_marker_detection_ = nh_.subscribe("/aruco_pose_Info", 10, &MotionPlanning::arucoMsgCallback, this);
  sleep_t_.sleep();
  // /aruco_single/pose, /aruco_pose_Info

  obb_object_pose_ = nh_.subscribe("/oriented_obj_Info", 10, &MotionPlanning::obbMsgCallback, this);
  sleep_t_.sleep();

  // Advertise the required topic
  trajectory_publisher_ = nh_.advertise<moveit_msgs::RobotTrajectory>("/trajectory", 10);
}

void MotionPlanning::obbMsgCallback(const geometry_msgs::PoseStamped& msg){
  obb_pose_ = msg.pose;
}

void MotionPlanning::visionMsgCallback(const picking_vision::ObjInfo& msg){
  obj_name_ = msg.obj_name;
  obj_x_ = msg.x;
  obj_y_ = msg.y;
  obj_z_ = msg.z;
}

void MotionPlanning::arucoMsgCallback(const geometry_msgs::PoseStamped& msg){
  // aruco_id_ = msg.header;
  aruco_pose_ = msg.pose;
  
  // aruco_coordinate_x_ = msg.pose.position.x;
  // aruco_coordinate_y_ = msg.pose.position.y;
  // aruco_coordinate_z_ = msg.pose.position.z;

  // aruco_coordinate_qx_ = msg.pose.orientation.x;
  // aruco_coordinate_qy_ = msg.pose.orientation.y;
  // aruco_coordinate_qz_ = msg.pose.orientation.z;
  // aruco_coordinate_qw_ = msg.pose.orientation.w;

  // tf::Quaternion aruco_coordinate_q1_(msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w); 
  // tf::Matrix3x3 m(aruco_coordinate_q1_);
  // double roll, pitch, yaw;
  // m.getRPY(roll, pitch, yaw);
  // aruco_coordinate_rx_ = roll;
  // aruco_coordinate_ry_ = pitch;
  // aruco_coordinate_rz_ = yaw;

  // Eigen::Quaterniond aruco_coordinate_q2_(msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w);
  // Eigen::Isometry3d tf_aruco = Eigen::Isometry3d::Identity();
  // tf_aruco.linear() = aruco_coordinate_q2_.toRotationMatrix();

  // ROS_WARN_STREAM("ROTATION TEST \n" << tf_aruco.linear());

  // aruco_coordinate_ = msg;
}

// Time parameterization of trajectory
// 1)input trajectory 2)parameterized trajectory 3)factor scaling both
bool MotionPlanning::parameterizeTime(moveit_msgs::RobotTrajectory& input_traj, moveit_msgs::RobotTrajectory& output_traj, double factor){
  double max_vel_factor, max_acc_factor;
  max_vel_factor = 0.1; // need to check 
  max_acc_factor = 0.0001; // need to check
  
  trajectory_processing::IterativeParabolicTimeParameterization param_process(500, 0.05);
  robot_trajectory::RobotTrajectory robot_trajectory(robot_model_, robot_group_name_);
  robot_trajectory.setRobotTrajectoryMsg(*robot_state_.get(), input_traj);
  param_process.computeTimeStamps(robot_trajectory, factor*max_vel_factor, factor*max_acc_factor);
  robot_trajectory.getRobotTrajectoryMsg(output_traj); // output_traj: final trajectory
  // To prohibit the robot motion of initial step
  if(output_traj.joint_trajectory.points.front().time_from_start == ros::Duration(0.0)){
    for(int vel_i=0; vel_i<output_traj.joint_trajectory.points.front().velocities.size(); vel_i++)
      output_traj.joint_trajectory.points.front().velocities[vel_i] = 0.0;
    for(int acc_i=0; acc_i<output_traj.joint_trajectory.points.front().accelerations.size(); acc_i++)
      output_traj.joint_trajectory.points.front().accelerations[acc_i] = 0.0;    
  }
  return true;
}

// Only visualizing robot motion on Rviz
// 1)robot trajectory 2)false if you want to see animation of robot motion
bool MotionPlanning::showRobotMotion(moveit_msgs::RobotTrajectory robot_traj, bool blocking){
  // visualize trajectory
  moveit::core::LinkModel* link_show = robot_model_->getLinkModel("end_effector_Virtualframe");
  moveit::core::LinkModel* link_show1 = robot_model_->getLinkModel("Vision_Virtualframe");
  // visual_tools_->deleteAllMarkers();
  // visual_tools_->trigger();
  visual_tools_->publishTrajectoryLine(robot_traj, link_show, jmg_);
  visual_tools_->publishTrajectoryLine(robot_traj, link_show1, jmg_);
  visual_tools_->trigger();

  // animate trajectory
  visual_tools_->publishTrajectoryPath(robot_traj, robot_state_, blocking); // false -> wait until animate is completed

  // TEST(start)
  // for(int j=0; j<robot_traj.joint_trajectory.points.size(); j++){
  //   std::vector<double> positions = robot_traj.joint_trajectory.points[j].positions;
  //   for(auto position : positions){
  //     std::cout << position << ", ";
  //   }
  //   std::cout << std::endl;
  // }

  trajectory_publisher_.publish(robot_traj);
  
  // TEST(end)
  
  return true;
}

// Update planning_scene_ according to robot_state and msg
// 1)planning_scene msg
bool MotionPlanning::updatePlanningScene(){
  // update planning_scene_ topic
  moveit_msgs::PlanningScene planning_scene_msgs;
  std::vector<std::string> joint_names = robot_state_->getJointModelGroup(robot_group_name_)->getActiveJointModelNames();
  planning_scene_msgs.is_diff = true;
  planning_scene_msgs.robot_state.is_diff = true;
  planning_scene_msgs.robot_state.joint_state.name.clear();
  planning_scene_msgs.robot_state.joint_state.position.clear();
  planning_scene_msgs.robot_state.joint_state.velocity.clear();
  planning_scene_msgs.robot_state.joint_state.effort.clear();
  planning_scene_msgs.robot_state.joint_state.header.stamp = ros::Time::now();
  for(auto joint_name : joint_names){
    planning_scene_msgs.robot_state.joint_state.name.push_back(joint_name);
    planning_scene_msgs.robot_state.joint_state.position.push_back(*robot_state_.get()->getJointPositions(joint_name));
    planning_scene_msgs.robot_state.joint_state.velocity.push_back(0.0);
  }
  planning_scene_diff_publisher_.publish(planning_scene_msgs);
  moveit_msgs::GetPlanningScene wait_srv;
  planning_scene_diff_client_.call(wait_srv); // to wait until applying planning scene
  
  return true;
}

// move robot arm to target joint value
// 1)target joint values 2)robot trajectory
bool MotionPlanning::moveJoint(std::vector<double> target_joints, moveit_msgs::RobotTrajectory& derived_traj){
  planning_interface::MotionPlanRequest req; // moveit_msgs::MotionPlanRequest
  planning_interface::MotionPlanResponse res;
  planning_interface::PlanningContextPtr context;
  moveit_msgs::MotionPlanResponse response;
  moveit_msgs::WorkspaceParameters workspace;
  std::string robot_group_name = jmg_->getName();

  // set workspace w.r.t global_link_
  workspace.header.stamp = ros::Time::now();
  workspace.header.frame_id = global_link_;
  workspace.min_corner.x = workspace.min_corner.y = -2.0;
  workspace.min_corner.z = 0.0;
  workspace.max_corner.x = workspace.max_corner.y = 2.0;
  workspace.max_corner.z = 2.5;

  // set parameters of planning
  req.group_name = robot_group_name;
  req.allowed_planning_time = 2.0;
  req.num_planning_attempts = 3;
  req.workspace_parameters = workspace;

  // define goal joint value
  robot_state_->setJointGroupPositions(jmg_, target_joints);
  moveit_msgs::Constraints joint_goal = kinematic_constraints::constructGoalConstraints(*robot_state_.get(), jmg_);
  req.goal_constraints.clear();
  req.goal_constraints.push_back(joint_goal);

  // path planning
  std::vector<std::size_t> adapter_index(0); 
  planning_pipeline_->generatePlan(planning_scene_, req, res, adapter_index); 
  if(res.error_code_.val != res.error_code_.SUCCESS){
    ROS_ERROR_STREAM("Could not compute plan successfully.");
    *robot_state_.get() = planning_scene_->getCurrentState(); // retrieve before robot state 
    return false;
  }
  res.getMessage(response);
  
  // time parameterization
  moveit_msgs::RobotTrajectory derived_traj_tmp;
  parameterizeTime(response.trajectory, derived_traj_tmp);

  // check the path whether it is valid or not. If empty string is entered, check all situations(robots, table, etc.)
  bool valid_path = planning_scene_->isPathValid(response.trajectory_start, derived_traj_tmp, std::string(), true); //true: valid
  if(!valid_path){
    ROS_ERROR_STREAM("Derived path is not valid.");
    *robot_state_.get() = planning_scene_->getCurrentState(); // retrieve before robot state 
    return false;
  }
  else  derived_traj = derived_traj_tmp;

  return true;
}

// move robot arm to target pose
// 1)target pose 2)target link 3)robot trajectory
bool MotionPlanning::moveJoint(Eigen::Isometry3d& target_pose, std::string target_link, moveit_msgs::RobotTrajectory& derived_traj){
  // save original robot_state before modify it
  robot_state::RobotState robot_state_before(*robot_state_.get());
  
  // get IK solution
  std::vector<double> joint_sol;
  if(!robot_state_->setFromIK(jmg_, target_pose, target_link, 0.1)){
    ROS_ERROR_STREAM("Finding IK solution is failed.");
    return false;
  }
  else{
    ROS_INFO_STREAM("Success to get IK solution.");
    robot_state_->copyJointGroupPositions(jmg_, joint_sol); // copy a IK solution from jmg to joint_values

    // for(auto sol : joint_sol){
    //   ROS_WARN_STREAM("JOINT: " << sol*180/M_PI);
    // }
    ROS_WARN_STREAM("JOINT: " << joint_sol[0]*180/M_PI << "," << joint_sol[1]*180/M_PI << "," << joint_sol[2]*180/M_PI << "," << joint_sol[3]*180/M_PI << "," << joint_sol[04]*180/M_PI << "," << joint_sol[5]*180/M_PI);
    robot_state_->printStatePositions();
    visual_tools_->prompt("TEST");
  
    if(moveJoint(joint_sol, derived_traj))  return true;
    else{
      ROS_ERROR_STREAM("Failed to plan motion.");
      *robot_state_.get() = robot_state_before; // retrieve robot_state to original
      return false;
    }
  }
}

// generate object
// 1)object name 2)object pose 3)object msg 4)whether show objects in planning_scene
void MotionPlanning::genObj(std::string object_name, Eigen::Isometry3d& object_pose, moveit_msgs::CollisionObject& collision_obj, bool print_objs){
  collision_obj.id = object_name;
  collision_obj.header.frame_id = global_link_;
  collision_obj.header.stamp = ros::Time::now();

  // Define the primitive and its dimensions
  collision_obj.primitives.resize(1);
  collision_obj.primitives[0].type = collision_obj.primitives[0].BOX;
  collision_obj.primitives[0].dimensions.resize(3);
  collision_obj.primitives[0].dimensions[0] = 0.1;
  collision_obj.primitives[0].dimensions[1] = 0.1;
  collision_obj.primitives[0].dimensions[2] = 0.1;

  // Define the pose
  collision_obj.primitive_poses.resize(1);
  collision_obj.primitive_poses[0] = tf2::toMsg(object_pose);

  // Add the object into the environment by adding it to the set of collision objects in the "world" part of the planning scene
  planning_scene_msgs_.world.collision_objects.push_back(collision_obj);
  planning_scene_msgs_.is_diff = true;
  planning_scene_diff_publisher_.publish(planning_scene_msgs_); // show the object in Rviz  
  moveit_msgs::GetPlanningScene wait_srv;
  planning_scene_diff_client_.call(wait_srv); // to wait until applying planning scene

  if(print_objs)  planning_scene_->printKnownObjects();
}

void MotionPlanning::run(){
  // generate collision object
  /* visual_tools_->prompt("Generate object collision");
  moveit_msgs::CollisionObject collision_object;
  Eigen::Isometry3d object_pose = Eigen::Isometry3d::Identity();
  object_pose.translation().x() = 0.8;
  object_pose.translation().z() = 1.2;
  genObj("obj1", object_pose, collision_object); */

  // Transformation Matrix, end_effector_Virtualframe
  Eigen::Isometry3d end_tfi = robot_state_->getFrameTransform("end_effector_Virtualframe");

  // ROS_WARN_STREAM("POS: \n" << end_tfi.translation().x() << ", " << end_tfi.translation().y() << ", " << end_tfi.translation().z());
  // ROS_WARN_STREAM("Orientation: \n" << end_tfi.linear());
  // ROS_WARN_STREAM("TF: \n" << end_tf);
  // Eigen::Isometry3d TF = end_tf * end_tf1;
  visual_tools_->publishAxisLabeled(end_tfi, "end_effector_Virtualframe");
  visual_tools_->publishAxis(end_tfi);
  visual_tools_->trigger();
  visual_tools_->prompt("end_effector_Virtualframe TF_i");

  // Transformation Matrix, Vision_Virtualframe
  Eigen::Isometry3d vision_tfi = robot_state_->getFrameTransform("Vision_Virtualframe");

  visual_tools_->publishAxisLabeled(vision_tfi, "Vision_Virtualframe");
  visual_tools_->publishAxis(vision_tfi);
  visual_tools_->trigger();
  visual_tools_->prompt("Vision_Virtualframe TF_i");

  // Demo: move robot arm to goal joint value
  visual_tools_->prompt("Move robot arm1 (joint)");
  std::vector<double> goal_joint = {-90.000 * M_PI/180, 120.000 * M_PI/180, -30.000 * M_PI/180, 0.0 * M_PI/180, 0.0* M_PI/180, 0.0 * M_PI/180}; // set goal joint value
  // std::vector<double> goal_joint = {-82.724 * M_PI/180, 114.472 * M_PI/180, -26.274 * M_PI/180, 0.970 * M_PI/180, -28.973* M_PI/180, -0.465 * M_PI/180}; // set goal joint value
  moveit_msgs::RobotTrajectory traj_1;
  bool success_plan = moveJoint(goal_joint, traj_1); // motion planning w.r.t joint values
  Eigen::Isometry3d end_tfm, vision_tfm;
  if(success_plan){
    showRobotMotion(traj_1, false); // show robot motion
    updatePlanningScene(); // update robot state to planning scene
    // ROS_WARN_STREAM("Motion Data: " << auto planning_scene_msgs);

    end_tfm = robot_state_->getFrameTransform("end_effector_Virtualframe");

    visual_tools_->publishAxisLabeled(end_tfm, "end_effector_Virtualframe");
    visual_tools_->publishAxis(end_tfm);
    visual_tools_->trigger();
    visual_tools_->prompt("end_effector_Virtualframe TF_m");

    vision_tfm = robot_state_->getFrameTransform("Vision_Virtualframe");

    visual_tools_->publishAxisLabeled(vision_tfm, "Vision_Virtualframe");
    visual_tools_->publishAxis(vision_tfm);
    visual_tools_->trigger();
    visual_tools_->prompt("Vision_Virtualframe TF_m");    
  }
  else{
    ROS_WARN_STREAM("Motion planning is failed");
    return;
  }
  
  // Demo: move robot arm to Visiongoal pose
  Eigen::Isometry3d goal_pose;// = vision_tfm;
  Eigen::Isometry3d goal_test_tf;
  Eigen::Isometry3d aruco_tf;
  tf2::fromMsg(aruco_pose_, aruco_tf);
  ROS_WARN_STREAM("Marker_Coordinates_position: \n" << aruco_tf.translation());
  ROS_WARN_STREAM("Marker_Coordinates_orientation: \n" << aruco_tf.linear());

  goal_pose = aruco_tf * Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitY());
  goal_test_tf = robot_state_->getFrameTransform("Vision_Virtualframe") * goal_pose;
  Eigen::Isometry3d end_effector_pose = robot_state_->getFrameTransform("end_effector_Virtualframe");
  goal_test_tf.translation() = end_effector_pose.translation();

  visual_tools_->publishAxisLabeled(goal_test_tf, "goal_test_tf");
  visual_tools_->prompt("Move robot arm2 (pose)");
  visual_tools_->trigger();

  moveit_msgs::RobotTrajectory traj_2;
  success_plan = moveJoint(goal_test_tf, "end_effector_Virtualframe", traj_2); // motion planning w.r.t joint values
  if(success_plan){
    showRobotMotion(traj_2, false); // show robot motion
    updatePlanningScene(); // update robot state to planning scene

    Eigen::Isometry3d end_tff = robot_state_->getFrameTransform("end_effector_Virtualframe");

    ROS_WARN_STREAM("POS: \n" << end_tff.translation().x() << ", " << end_tff.translation().y() << ", " << end_tff.translation().z());
    ROS_WARN_STREAM("Orientation: \n" << end_tff.linear());

    visual_tools_->publishAxisLabeled(end_tff, "end_effector_Virtualframe");
    visual_tools_->publishAxis(end_tff);
    visual_tools_->trigger();
    visual_tools_->prompt("end_effector_Virtualframe TF_f");
  }
  else{
    ROS_WARN_STREAM("Motion planning is failed");
  }

  Eigen::Isometry3d goal_pose2;// = vision_tfm;
  Eigen::Isometry3d obj_goal;
  tf2::fromMsg(obb_pose_, obj_goal);
  ROS_WARN_STREAM("Obj_Coordinates_position: \n" << obj_goal.translation());

  goal_pose2 = obj_goal * Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitX());
  Eigen::Isometry3d goal_test_tf2 = robot_state_->getFrameTransform("Vision_Virtualframe") * goal_pose2;

  visual_tools_->publishAxisLabeled(goal_test_tf2, "goal_test_tf2");
  visual_tools_->prompt("Move robot arm4 (pose)");
  visual_tools_->trigger();

  moveit_msgs::RobotTrajectory traj_4;
  success_plan = moveJoint(goal_test_tf2, "end_effector_Virtualframe", traj_4); // motion planning w.r.t joint values
  if(success_plan){
    showRobotMotion(traj_4, false); // show robot motion
    updatePlanningScene(); // update robot state to planning scene

    Eigen::Isometry3d end_tf2 = robot_state_->getFrameTransform("end_effector_Virtualframe");

    ROS_WARN_STREAM("POS: \n" << end_tf2.translation().x() << ", " << end_tf2.translation().y() << ", " << end_tf2.translation().z());
    ROS_WARN_STREAM("Orientation: \n" << end_tf2.linear());

    visual_tools_->publishAxisLabeled(end_tf2, "end_effector_Virtualframe");
    visual_tools_->publishAxis(end_tf2);
    visual_tools_->trigger();
    visual_tools_->prompt("end_effector_Virtualframe TF_f");
  }
  else{
    ROS_WARN_STREAM("Motion planning is failed");
  }

  visual_tools_->prompt("Demo is finishied.");
  visual_tools_->trigger();
}
} // namespace

int main (int argc, char** argv){
  ros::init(argc, argv, "motion");

  // Allow the action server to recieve and send ros messagestriangle_count
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle nh("~");

  motion_planning::MotionPlanning motion(nh);
  motion.run();

  ros::shutdown();
  return 0;
}