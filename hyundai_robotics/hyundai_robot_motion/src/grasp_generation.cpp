#include <preparatory_manipulation/grasp_generation.h>

namespace preparatory_manipulation{

GraspGeneration::GraspGeneration(preparatory_manipulation::MotionPlanning& motion, preparatory_manipulation::Params& params) : 
    motion_(motion)
  , model_package_name_(params.model_package_name)
  , global_link_(params.global_link)
  , model_path_(params.model_path)
{}

// generate mesh object
// 1)object name 2)object pose 3)object msg 4)whether show objects in planning_scene
shapes::Mesh* GraspGeneration::genMeshObj(std::string object_name, Eigen::Isometry3d& object_pose, moveit_msgs::CollisionObject& collision_object, bool print_objs){
  const std::string part_file_path = "package://" + model_package_name_ + "/stl/target_object/";
  shapes::Mesh* mesh_file = shapes::createMeshFromResource(part_file_path + object_name + ".STL"); // Load mesh file
  ROS_INFO_STREAM(object_name << " is loaded(shapes::Mesh).");
  
  shapes::ShapeMsg target_mesh_msg;
  shapes::constructMsgFromShape(mesh_file, target_mesh_msg);
  shape_msgs::Mesh target_mesh = boost::get<shape_msgs::Mesh>(target_mesh_msg);

  collision_object.header.frame_id = global_link_;
  collision_object.header.stamp = ros::Time::now();
  collision_object.id = object_name;
  
  collision_object.meshes.resize(1);
  collision_object.meshes[0] = target_mesh;
  collision_object.mesh_poses.resize(1);
  collision_object.mesh_poses[0] = tf2::toMsg(object_pose);
  collision_object.operation = collision_object.ADD; // Adds the object to the planning scene

  // Set object color
  moveit_msgs::ObjectColor collision_object_color;
  collision_object_color.id = object_name;
  collision_object_color.color.r = 0.7;
  collision_object_color.color.g = 0.35;
  collision_object_color.color.b = 0.0;
  collision_object_color.color.a = 1;
  motion_.getPlanningSceneMsgs().object_colors.push_back(collision_object_color);

  // If 'collision_object' already involve in the planning_scene, it is deleted from the planning_scene to minimize the time it takes to check for collisions
  auto& object_world = motion_.getPlanningSceneMsgs().world.collision_objects;
  for(auto it=object_world.begin(); it!=object_world.end(); ){
    if(it->id == collision_object.id)  it = object_world.erase(it);
    else                               it++;
  }

  // Add the object into the environment by adding it to the set of collision objects in the "world" part of the planning scene
  motion_.getPlanningSceneMsgs().world.collision_objects.push_back(collision_object);
  motion_.getPlanningSceneMsgs().is_diff = true;
  motion_.getPlanningSceneDiffPublisher().publish(motion_.getPlanningSceneMsgs()); // show the object in Rviz  
  moveit_msgs::GetPlanningScene wait_srv;
  motion_.getPlanningSceneDiffClient().call(wait_srv); // to wait until applying planning scene

  if(print_objs)  motion_.getPlanningScenePtr()->printKnownObjects();

  return mesh_file;
}

// generate mesh object
// 1)object name 2)object pose 3)object msg 4)three vertice and normal vector of the same triangle / triangle number
shapes::Mesh* GraspGeneration::genMeshObj(std::string object_name, Eigen::Isometry3d& object_pose, moveit_msgs::CollisionObject& collision_object
                                        , std::vector<std::tuple<EigenSTL::vector_Vector3d, Eigen::Vector3d, unsigned int>>& triangle_info){
  const std::string part_file_path = "package://" + model_package_name_ + "/stl/target_object/";
  shapes::Mesh* mesh_file = shapes::createMeshFromResource(part_file_path + object_name + ".STL"); // Load mesh file
  ROS_INFO_STREAM(object_name << " is loaded(shapes::Mesh).");
  
  shapes::ShapeMsg target_mesh_msg;
  shapes::constructMsgFromShape(mesh_file, target_mesh_msg);
  shape_msgs::Mesh target_mesh = boost::get<shape_msgs::Mesh>(target_mesh_msg);

  collision_object.header.frame_id = global_link_;
  collision_object.header.stamp = ros::Time::now();
  collision_object.id = object_name;
  
  collision_object.meshes.resize(1);
  collision_object.meshes[0] = target_mesh;
  collision_object.mesh_poses.resize(1);
  collision_object.mesh_poses[0] = tf2::toMsg(object_pose);
  collision_object.operation = collision_object.ADD; // Adds the object to the planning scene

  // Set object color
  moveit_msgs::ObjectColor collision_object_color;
  collision_object_color.id = object_name;
  collision_object_color.color.r = 0.7;
  collision_object_color.color.g = 0.35;
  collision_object_color.color.b = 0.0;
  collision_object_color.color.a = 1;
  motion_.getPlanningSceneMsgs().object_colors.push_back(collision_object_color);

  // If 'collision_object' already involve in the planning_scene, it is deleted from the planning_scene to minimize the time it takes to check for collisions
  auto& object_world = motion_.getPlanningSceneMsgs().world.collision_objects;
  for(auto it=object_world.begin(); it!=object_world.end(); ){
    if(it->id == collision_object.id)  it = object_world.erase(it);
    else                               it++;
  }

  // Add the object into the environment by adding it to the set of collision objects in the "world" part of the planning scene
  motion_.getPlanningSceneMsgs().world.collision_objects.push_back(collision_object);
  motion_.getPlanningSceneMsgs().is_diff = true;
  motion_.getPlanningSceneDiffPublisher().publish(motion_.getPlanningSceneMsgs()); // show the object in Rviz
  moveit_msgs::GetPlanningScene wait_srv;
  motion_.getPlanningSceneDiffClient().call(wait_srv); // to wait until applying planning scene
  motion_.getPlanningScenePtr()->printKnownObjects();

  // Derive position of three vertice and normal vector of the same triangle
  mesh_file->computeTriangleNormals();
  motion_.getSleepShort().sleep();
  mesh_file->computeVertexNormals();
  motion_.getSleepShort().sleep();
  for(unsigned int k=0; k<mesh_file->triangle_count; k++){
    std::vector<unsigned int> vertice_tri_i{mesh_file->triangles[3*k], mesh_file->triangles[3*k+1], mesh_file->triangles[3*k+2]}; // vertice index

    EigenSTL::vector_Vector3d vertice_tri;
    for(int ver_i=0; ver_i<vertice_tri_i.size(); ver_i++) // assign vertice in triangle(three vertice)
      vertice_tri.push_back(Eigen::Vector3d(mesh_file->vertices[3*vertice_tri_i[ver_i]], mesh_file->vertices[3*vertice_tri_i[ver_i]+1], mesh_file->vertices[3*vertice_tri_i[ver_i]+2]));
    
    Eigen::Vector3d tri_normal_vec(mesh_file->triangle_normals[3*k], mesh_file->triangle_normals[3*k+1], mesh_file->triangle_normals[3*k+2]);
    
    // // show the vertice and normal direcion of the triangle
    // for(int i=0; i<vertice_tri.size(); i++){
    //   Eigen::Vector3d vert_1 = object_pose.linear()*vertice_tri[i]+object_pose.translation();
    //   Eigen::Vector3d vert_2 = object_pose.linear()*(vertice_tri[i]+tri_normal_vec.normalized()/10)+object_pose.translation();
    //   motion_.getVisualTools()->publishArrow(tf2::toMsg(vert_1)
    //                                       , tf2::toMsg(vert_2)
    //                                       , rviz_visual_tools::colors::PINK, rviz_visual_tools::scales::SMALL);
    // }
    // motion_.getVisualTools()->trigger();
    // motion_.getVisualTools()->prompt("point test");
    // motion_.getVisualTools()->deleteAllMarkers();

    triangle_info.push_back(std::make_tuple(vertice_tri, tri_normal_vec, k)); // three vertice and normal vector of the same triangle / triangle number
  }

  // // show all vertice & edges and its position value
  // for(int i=0; i<triangle_info.size(); i++){
  //   EigenSTL::vector_Vector3d vertice_tri_tmp = std::get<0>(triangle_info[i]);
  //   motion_.getVisualTools()->publishCylinder(object_pose.linear()*vertice_tri_tmp[0]+object_pose.translation()
  //                                           , object_pose.linear()*vertice_tri_tmp[1]+object_pose.translation()
  //                                           , rviz_visual_tools::colors::GREY, 0.002);
  //   motion_.getVisualTools()->publishCylinder(object_pose.linear()*vertice_tri_tmp[1]+object_pose.translation()
  //                                           , object_pose.linear()*vertice_tri_tmp[2]+object_pose.translation()
  //                                           , rviz_visual_tools::colors::GREY, 0.002);
  //   motion_.getVisualTools()->publishCylinder(object_pose.linear()*vertice_tri_tmp[1]+object_pose.translation()
  //                                           , object_pose.linear()*vertice_tri_tmp[0]+object_pose.translation()
  //                                           , rviz_visual_tools::colors::GREY, 0.002);
  //   for(int j=0; j<vertice_tri_tmp.size(); j++){
  //     Eigen::Vector3d vert_global = object_pose.linear()*vertice_tri_tmp[j]+object_pose.translation(); // global position
  //     motion_.getVisualTools()->publishSphere(vert_global, rviz_visual_tools::colors::LIME_GREEN, rviz_visual_tools::scales::SMALL);
  //     Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  //     text_pose.translation() = vert_global + Eigen::Vector3d(0,0,0.01);
  //     motion_.getVisualTools()->publishText(text_pose, std::to_string(vertice_tri_tmp[j][0]) + "," + std::to_string(vertice_tri_tmp[j][1]) + "," +  std::to_string(vertice_tri_tmp[j][2])
  //                                         , rviz_visual_tools::colors::WHITE, rviz_visual_tools::scales::SMALL, false);
  //   }
  // }
  // motion_.getVisualTools()->publishAxis(object_pose, rviz_visual_tools::scales::SMALL);
  // motion_.getVisualTools()->trigger();
  // motion_.getVisualTools()->prompt("vertex location test");
  // motion_.getVisualTools()->deleteAllMarkers();

  return mesh_file;
}

// Calculate convex hull & get edges of it
// 1)input mesh model(Open3D) 2)both vertice of convexhull's edges(position end point of edge) 3)delete same edge(only direction is different)
bool GraspGeneration::getConvexEdges(std::shared_ptr<open3d::geometry::TriangleMesh>& mesh_model
                                   , std::vector<std::tuple<Eigen::Vector3d, Eigen::Vector3d>>& convex_edges, bool delete_same_edge){
  // derive convex hull
  std::tuple<std::shared_ptr<open3d::geometry::TriangleMesh>, std::vector<std::size_t>> convex_model = mesh_model->ComputeConvexHull();
  std::vector<Eigen::Vector3d> convex_vertice = std::get<0>(convex_model)->vertices_; // key: index of vertex / value: position of vertex
  std::vector<Eigen::Vector3i> convex_triangles = std::get<0>(convex_model)->triangles_; // key: index of triangle / value: index of vertice in triangle

  convex_edges.clear();
  for(int tri_i=0; tri_i<convex_triangles.size(); tri_i++){
    // position of vertice in triangle
    std::vector<Eigen::Vector3d> convex_triangle_vertice = {convex_vertice[convex_triangles[tri_i][0]]
                                                          , convex_vertice[convex_triangles[tri_i][1]], convex_vertice[convex_triangles[tri_i][2]]};
    convex_edges.push_back(std::make_tuple(convex_triangle_vertice[0], convex_triangle_vertice[1]));
    convex_edges.push_back(std::make_tuple(convex_triangle_vertice[1], convex_triangle_vertice[2]));
    convex_edges.push_back(std::make_tuple(convex_triangle_vertice[2], convex_triangle_vertice[0]));
  }

  if(delete_same_edge){
    // Delete same edge(only its direction is different)
    for(int i=0; i<convex_edges.size()-1; i++){
      for(int j=i+1; j<convex_edges.size(); j++){
        if(std::get<0>(convex_edges[i])==std::get<1>(convex_edges[j]) && std::get<1>(convex_edges[i])==std::get<0>(convex_edges[j])){
          // same edge but different direction
          convex_edges.erase(convex_edges.begin()+j);
          j--;
        }
      }
    }
  }

  // // Show edges of convex hull
  // for(int edge_i=0; edge_i<convex_edges.size(); edge_i++)
  //   motion_.getVisualTools()->publishCylinder(std::get<0>(convex_edges[edge_i]), std::get<1>(convex_edges[edge_i]), rviz_visual_tools::colors::PURPLE, 0.005);
  // motion_.getVisualTools()->trigger();
  // motion_.getVisualTools()->prompt("convex edge: " + std::to_string(convex_edges.size()));
  // motion_.getVisualTools()->deleteAllMarkers();
 
  return true;
}

// Generate mesh object using Open3D & center of mesh object
//1)part name 2)loaded model(Open3D)
bool GraspGeneration::genMeshObj(std::string object_name, std::shared_ptr<open3d::geometry::TriangleMesh>& mesh_model
                               , Eigen::Vector3d& center_mesh_model){
  bool result = genMeshObj(object_name, mesh_model);
  
  // Get center of mass in mesh_model
  // TODO: derive center of mass by calculating mesh file
  // if(object_name == "chair_part1")        center_mesh_model = Eigen::Vector3d(0.000000, -0.012306, 0.005000);
  // else if(object_name == "chair_part2")   center_mesh_model = Eigen::Vector3d(0.000000, 0.000030, 0.009993);
  // else if(object_name == "chair_part3")   center_mesh_model = Eigen::Vector3d(0.000000, 0.000038, 0.009991);
  // else if(object_name == "chair_part4")   center_mesh_model = Eigen::Vector3d(0.000000, -0.128979, 0.005064);
  // else if(object_name == "chair_part5")   center_mesh_model = Eigen::Vector3d(0.252653, 0.350675, 0.022467);
  // else if(object_name == "chair_part6")   center_mesh_model = Eigen::Vector3d(0.252653, 0.350675, -0.022467);
  // else{
  //   ROS_WARN_STREAM("'object_name' is not correct");
  //   return false;
  // }
  
  if(object_name == "chair_side_part")        center_mesh_model = Eigen::Vector3d(0.252653, 0.350675, 0.022467);
  else if(object_name == "shelf_side_part")   center_mesh_model = Eigen::Vector3d(0.175000, -0.390215, -0.009250);
  else if(object_name == "stool_side_part")   center_mesh_model = Eigen::Vector3d(0.107500, -0.261209, -0.015047);
  else{
    ROS_WARN_STREAM("'object_name' is not correct");
    return false;
  }

  return result;
}

// Generate mesh object using Open3D
//1)part name 2)loaded model(Open3D)
bool GraspGeneration::genMeshObj(std::string object_name, std::shared_ptr<open3d::geometry::TriangleMesh>& mesh_model){
  // Load mesh file
  open3d::io::ReadTriangleMeshOptions read_mesh_opt;
  read_mesh_opt.enable_post_processing = true;
  read_mesh_opt.print_progress = true;
  if(!open3d::io::ReadTriangleMesh(model_path_ + object_name + ".STL", *mesh_model, read_mesh_opt)){
    ROS_ERROR_STREAM("Loading the mesh file is failed.(Open3D)");
    return false;
  } 
  ROS_INFO_STREAM(object_name << " is loaded(Open3D).");

  mesh_model->ComputeTriangleNormals();
  mesh_model->ComputeVertexNormals();
  mesh_model->ComputeAdjacencyList();

  return true;
}

// Derive the boundary edges of mesh model
// 1)part name 2)loaded model(Open3D) 3)boundary info(both vertice and normal vector of the boundary edges)
bool GraspGeneration::getBoundaryEdges(std::string object_name, std::shared_ptr<open3d::geometry::TriangleMesh>& mesh_model
                                     , std::vector<std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d>>& boundary_edges_info){
  // // Load mesh file
  // open3d::io::ReadTriangleMeshOptions read_mesh_opt;
  // read_mesh_opt.enable_post_processing = true;
  // read_mesh_opt.print_progress = true;
  // if(!open3d::io::ReadTriangleMesh(model_path_ + object_name + ".STL", *mesh_model, read_mesh_opt)){
  //   ROS_ERROR_STREAM("Loading the mesh file is failed.(Open3D)");
  //   return false;
  // } 
  // ROS_INFO_STREAM(object_name << " is loaded(Open3D).");

  // mesh_model->ComputeTriangleNormals();
  // mesh_model->ComputeVertexNormals();
  // mesh_model->ComputeAdjacencyList();

  // Derive outline of mesh model
  std::vector<Eigen::Vector2i> boundary_edges_index = mesh_model->GetNonManifoldEdges(false); // value: index of vertice belong to the boundary edge
  std::vector<int> boundary_vertice_index = mesh_model->GetNonManifoldVertices();

  std::vector<Eigen::Vector3d> mesh_vertice = mesh_model->vertices_; // key: index of vertex / value: position of vertex
  std::vector<Eigen::Vector3d> triangle_normals = mesh_model->triangle_normals_; // key: index of triangle / value: normal vector of triangle
  std::vector<Eigen::Vector3i> mesh_triangles = mesh_model->triangles_; // key: index of triangle / value: index of vertice in triangle

  boundary_edges_info.clear();
  for(int edge_i=0; edge_i<boundary_edges_index.size(); edge_i++){
    Eigen::Vector3d vertex_start = mesh_vertice[boundary_edges_index[edge_i][0]]; // vertex of boundary edge
    Eigen::Vector3d vertex_end = mesh_vertice[boundary_edges_index[edge_i][1]]; // vertex of boundary edge

    for(int tri_i=0; tri_i<mesh_triangles.size(); tri_i++){
      std::vector<int> vertice_index = {mesh_triangles[tri_i][0], mesh_triangles[tri_i][1], mesh_triangles[tri_i][2]}; // vertice index of triangle
      auto iter_1 = std::find(vertice_index.begin(), vertice_index.end(), boundary_edges_index[edge_i][0]); // check if vertex index is same as that of edge
      auto iter_2 = std::find(vertice_index.begin(), vertice_index.end(), boundary_edges_index[edge_i][1]); // check if vertex index is same as that of edge

      if(iter_1!=vertice_index.end() && iter_2!=vertice_index.end()){ // if these vertice are same as that of boundary edge
        boundary_edges_info.push_back(std::make_tuple(vertex_start, vertex_end, triangle_normals[tri_i])); // both vertice & normal vector of boundary edge
      }
    }
  }

  // // show the boundary edge & that of normal vector
  // for(int info_i=0; info_i<boundary_edges_info.size(); info_i++){
  //   Eigen::Vector3d arrow_last_1, arrow_last_2;
  //   arrow_last_1 = std::get<0>(boundary_edges_info[info_i]) + std::get<2>(boundary_edges_info[info_i]).normalized()/10;
  //   arrow_last_2 = std::get<1>(boundary_edges_info[info_i]) + std::get<2>(boundary_edges_info[info_i]).normalized()/10;
  //   if((std::get<0>(boundary_edges_info[info_i])-std::get<1>(boundary_edges_info[info_i])).norm() > 0.05){ // if edge length is not short, show the result
  //     motion_.getVisualTools()->publishCylinder(std::get<0>(boundary_edges_info[info_i]), std::get<1>(boundary_edges_info[info_i]), rviz_visual_tools::colors::MAGENTA, 0.005);
  //     motion_.getVisualTools()->publishArrow(tf2::toMsg(std::get<0>(boundary_edges_info[info_i])), tf2::toMsg(arrow_last_1), rviz_visual_tools::colors::RED, rviz_visual_tools::scales::SMALL, 0);
  //     motion_.getVisualTools()->publishArrow(tf2::toMsg(std::get<1>(boundary_edges_info[info_i])), tf2::toMsg(arrow_last_2), rviz_visual_tools::colors::RED, rviz_visual_tools::scales::SMALL, 0);
  //     motion_.getVisualTools()->trigger();
  //     motion_.getVisualTools()->prompt("boundary edge & normal vector test");
  //     motion_.getVisualTools()->deleteAllMarkers();
  //   }
  // }

  // // Show the boundary edges
  // for(int i=0; i<boundary_edges_info.size(); i++){
  //   motion_.getVisualTools()->publishCylinder(std::get<0>(boundary_edges_info[i]), std::get<1>(boundary_edges_info[i]), rviz_visual_tools::colors::LIME_GREEN, 0.005);
  // }
  // motion_.getVisualTools()->trigger();
  // motion_.getVisualTools()->prompt("boundary edge test");
  // motion_.getVisualTools()->deleteAllMarkers();

  return true;
}

} // namespace