/*#include <cstdio>

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  printf("hello world hello_moveit package\n");
  return 0;
}*/
/*#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/robot_model/robot_model.h>

int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "hello_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("hello_moveit");

  // We spin up a SingleThreadedExecutor so MoveItVisualTools interact with ROS
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });


  // Next step goes here

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "panda_arm");

  // Construct and initialize MoveItVisualTools
  auto moveit_visual_tools = moveit_visual_tools::MoveItVisualTools{
      node, "panda_link0", rviz_visual_tools::RVIZ_MARKER_TOPIC,
      move_group_interface.getRobotModel()};
  moveit_visual_tools.deleteAllMarkers();
  moveit_visual_tools.loadRemoteControl();
  // Set a target Pose
  auto const target_pose = []{
    geometry_msgs::msg::Pose msg;
    msg.orientation.w = 1.0;
    msg.position.x = 0.28;
    msg.position.y = -0.2;
    msg.position.z = 0.5;
    return msg;
  }();
  move_group_interface.setPoseTarget(target_pose);

  // Create a plan to that target pose
  auto const [success, plan] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Create a closures for visualization
  auto const draw_title = [&moveit_visual_tools](auto text) {
    auto const text_pose = [] {
      auto msg = Eigen::Isometry3d::Identity();
      msg.translation().z() = 1.0;
      return msg;
    }();
    moveit_visual_tools.publishText(text_pose, text, rviz_visual_tools::WHITE,
                                  rviz_visual_tools::XLARGE);
  };
  auto const prompt = [&moveit_visual_tools](auto text) {
    moveit_visual_tools.prompt(text);
  };
  auto const draw_trajectory_tool_path =
      [&moveit_visual_tools,
      jmg = move_group_interface.getRobotModel()->getJointModelGroup(
           "panda_arm")](auto const trajectory) {
        moveit_visual_tools.publishTrajectoryLine(trajectory, jmg);
      };

// Set a target Pose
auto const target_pose = [] {
  geometry_msgs::msg::Pose msg;
  msg.orientation.w = 1.0;
  msg.position.x = 0.28;
  msg.position.y = 0.4;  // <---- This value was changed
  msg.position.z = 0.5;
  return msg;
}();
move_group_interface.setPoseTarget(target_pose);

// Create collision object for the robot to avoid
auto const collision_object = [frame_id =
                                 move_group_interface.getPlanningFrame()] {
  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = frame_id;
  collision_object.id = "box1";
  shape_msgs::msg::SolidPrimitive primitive;

  // Define the size of the box in meters
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[primitive.BOX_X] = 0.5;
  primitive.dimensions[primitive.BOX_Y] = 0.1;
  primitive.dimensions[primitive.BOX_Z] = 0.5;

  // Define the pose of the box (relative to the frame_id)
  geometry_msgs::msg::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = 0.2;
  box_pose.position.y = 0.2;
  box_pose.position.z = 0.25;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  return collision_object;
}();

// Add the collision object to the scene
moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
planning_scene_interface.applyCollisionObject(collision_object);

// Create a MoveIt2 Planning Pipeline
auto planning_pipeline = std::make_shared<moveit::planning_interface::PlanningPipeline>(
    node->get_node_base_interface(),
    node->get_node_timers_interface(),
    node->get_node_logging_interface(),
    node->get_node_clock_interface(),
    node->get_node_parameters_interface());

// Get the current planning pipeline configuration
const auto planning_pipeline_settings = node->get_parameters({"move_group", "planning_plugin"}).get<rclcpp::ParameterValue>();

// Create an OMPL planner
auto ompl_planner = std::make_shared<ompl_interface::OMPLPlanner>();
ompl_planner->initialize(node, "ompl_planning_plugin", "robot_description", planning_pipeline_settings);

// Create a CHOMP planner
auto chomp_planner = std::make_shared<chomp_motion_planner::ChompMotionPlanner>();
chomp_planner->initialize(node, "chomp_planning_plugin", "robot_description", planning_pipeline_settings);

// Create a planning pipeline description with both planners
moveit_msgs::msg::PlanningPipeline pipeline_description;
pipeline_description.planner_sequence.emplace_back("ompl_planning_plugin", "manipulator");
pipeline_description.planner_sequence.emplace_back("chomp_planning_plugin", "manipulator");

// Configure the planning pipeline with the description
planning_pipeline->configure(pipeline_description);

// Plan using the planning pipeline
auto planning_context = planning_pipeline->create_new_context("manipulator");
moveit::planning_interface::MotionPlanRequest request;
moveit::planning_interface::MotionPlanResponse response;
request.group_name = "manipulator";
request.planner_id = "chomp_planning_plugin"; // Set the planner ID to CHOMP
planning_context->setPlanningRequest(request);
planning_context->solve(response);
// Create a plan to that target pose
prompt("Press 'Next' in the RvizVisualToolsGui window to plan");
draw_title("Planning");
moveit_visual_tools.trigger();
auto const [success, plan] = [&move_group_interface] {
  moveit::planning_interface::MoveGroupInterface::Plan msg;
  auto const ok = static_cast<bool>(move_group_interface.plan(msg));
  return std::make_pair(ok, msg);
}();

// Execute the plan
if (success) {
  draw_trajectory_tool_path(plan.trajectory_);
  moveit_visual_tools.trigger();
  prompt("Press 'Next' in the RvizVisualToolsGui window to execute");
  draw_title("Executing");
  moveit_visual_tools.trigger();
  move_group_interface.execute(plan);
} else {
  draw_title("Planning Failed!");
  moveit_visual_tools.trigger();
  RCLCPP_ERROR(logger, "Planing failed!");
}
// Execute the plan
  if(success) {
    move_group_interface.execute(plan);
  } else {
    RCLCPP_ERROR(logger, "Planing failed!");
  }

  rclcpp::shutdown();  // <--- This will cause the spin function in the thread to return
  spinner.join();  // <--- Join the thread before exiting
  return 0;
}*/



// Test

#include <memory>
#include <thread>
#include <chrono>
#include <string>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

int main(int argc, char* argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
      "hello_moveit", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("hello_moveit");

  // We spin up a SingleThreadedExecutor so MoveItVisualTools interact with ROS
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });

  // Set the variable for pre_planner and post planner
  std::string case_arg(argv[5]);
  std::cout << case_arg <<std::endl;
  int testcase = std::stoi(case_arg);
  std::string pre_planner;
  std::string post_planner;
  //std::cout << "get value" << std::endl;
  if(testcase==1 || testcase==4 || testcase==5 || testcase==6 || testcase==9 || testcase==10 || testcase==11 || testcase==14 || testcase==15)
  {
    pre_planner = "ompl";
    if(testcase==4 || testcase==9 || testcase==14)
    {
      post_planner = "chomp";
    }
    else
    {
      post_planner = "stomp";
    }
  }
  else if(testcase==2 || testcase==7 || testcase==12)
  {
    pre_planner = "chomp";
  }
  else
  {
    pre_planner = "stomp";
  }
  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "panda_arm");

  // Construct and initialize MoveItVisualTools
  auto moveit_visual_tools = moveit_visual_tools::MoveItVisualTools{
      node, "panda_link0", rviz_visual_tools::RVIZ_MARKER_TOPIC,
      move_group_interface.getRobotModel()};
  moveit_visual_tools.deleteAllMarkers();
  moveit_visual_tools.loadRemoteControl();

  // Create a closures for visualization
  auto const draw_title = [&moveit_visual_tools](auto text, const Eigen::Isometry3d& pose) {
    
      moveit_visual_tools.publishText(pose, text, rviz_visual_tools::WHITE,
                                    rviz_visual_tools::XLARGE);                        
  };

  auto const prompt = [&moveit_visual_tools](auto text) {
    moveit_visual_tools.prompt(text);
  };
  auto const draw_trajectory_tool_path =
      [&moveit_visual_tools,
      jmg = move_group_interface.getRobotModel()->getJointModelGroup(
          "panda_arm")](auto const trajectory) {
       moveit_visual_tools.publishTrajectoryLine(trajectory, jmg);
      };
  
  auto const text_pose1 = [] {
      auto msg = Eigen::Isometry3d::Identity();
      msg.translation().z() = 1.0;
      return msg;
    }();
  auto const text_pose2 = [] {
      auto msg = Eigen::Isometry3d::Identity();
      msg.translation().z() = 1.5;
      return msg;
    }();

  // Set the planner ID to CHOMP
  move_group_interface.setPlannerId(pre_planner);

  // Set the planning time
  move_group_interface.setPlanningTime(5.0);

  // Set a target Pose
  auto const target_pose = [] {
    geometry_msgs::msg::Pose msg;
    msg.orientation.w = 1.0;
    msg.position.x = 0.28;
    msg.position.y = 0.4;
    msg.position.z = 0.5;
    return msg;
  }();
  move_group_interface.setPoseTarget(target_pose);

  // Create collision object for the robot to avoid
  auto const collision_object = [frame_id =
                                  move_group_interface.getPlanningFrame()] {
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = frame_id;
    collision_object.id = "box1";
    shape_msgs::msg::SolidPrimitive primitive;

    // Define the size of the box in meters
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 0.5;
    primitive.dimensions[primitive.BOX_Y] = 0.1;
    primitive.dimensions[primitive.BOX_Z] = 0.5;

    // Define the pose of the box (relative to the frame_id)
    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.2;
    box_pose.position.y = 0.2;
    box_pose.position.z = 0.25;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    return collision_object;
  }();

  auto const collision_object2 = [frame_id =
                                  move_group_interface.getPlanningFrame()] {
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = frame_id;
    collision_object.id = "box2";
    shape_msgs::msg::SolidPrimitive primitive;

    // Define the size of the box in meters
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 0.3;
    primitive.dimensions[primitive.BOX_Y] = 0.05;
    primitive.dimensions[primitive.BOX_Z] = 0.3;

    // Define the pose of the box (relative to the frame_id)
    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.5;
    box_pose.position.y = 0.2;
    box_pose.position.z = 0.6;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    return collision_object;
  }();

  auto const collision_object3 = [frame_id =
                                  move_group_interface.getPlanningFrame()] {
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = frame_id;
    collision_object.id = "box3";
    shape_msgs::msg::SolidPrimitive primitive;

    // Define the size of the box in meters
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 0.5;
    primitive.dimensions[primitive.BOX_Y] = 0.05;
    primitive.dimensions[primitive.BOX_Z] = 0.2;

    // Define the pose of the box (relative to the frame_id)
    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.2;
    box_pose.position.y = 0.2;
    box_pose.position.z = 0.85;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    return collision_object;
  }();

  // Add the collision object to the scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  if(testcase > 5)
  {
    std::cout << "get value" << std::endl;
    planning_scene_interface.applyCollisionObject(collision_object);
    if(testcase > 10)
    {
      planning_scene_interface.applyCollisionObject(collision_object2);
      planning_scene_interface.applyCollisionObject(collision_object3);
    }
  }
  // Create a plan to that target pose
  prompt("Press 'Next' in the RvizVisualToolsGui window to plan");
  draw_title("Planning", text_pose1);
  moveit_visual_tools.trigger();
  
  // Set the start time before planning
  std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();
  // Create a plan to that target pose
  auto const [success, plan] = [&move_group_interface] {
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();
  
  //auto success_final = success;

  // Execute the plan

  // First Define the final plan to execute
  moveit::planning_interface::MoveGroupInterface::Plan final_plan;
  //auto const success_final = ;
  if (success)
  {
    // Set the planner ID to CHOMP
    if(testcase==4 || testcase==5 || testcase==9 || testcase==10 || testcase==14 || testcase==15)
    {
      moveit::planning_interface::MoveGroupInterface::Plan chomp_plan;
      chomp_plan.trajectory_ = plan.trajectory_;
      move_group_interface.setPlannerId(post_planner);

    // Assign the time-stamped trajectory back to the OMPL plan
    //plan.trajectory_.joint_trajectory.points = robot_trajectory;
    //plan.trajectory_ = robot_trajectory->getRobotTrajectoryMsg();
    //moveit_msgs::msg::RobotTrajectory robot_traj_msg;
    //robot_trajectory->getRobotTrajectoryMsg(robot_traj_msg);

    
    // Set the initial trajectory for CHOMP optimization
    //move_group_interface.setStartState(*move_group_interface.getCurrentState());
    //move_group_interface.setTrajectory(plan.trajectory);
      auto success_final = move_group_interface.plan(chomp_plan);
      final_plan = chomp_plan;
      if (!success_final)
      {
        RCLCPP_ERROR(logger, "Execution of Post-planner failed!");
        return 1;
      }
    }
    else
    {
      //success_final = success;
      final_plan = plan;
    }
    /*if (!success_final)
    {
      RCLCPP_ERROR(logger, "Execution of Post-planner failed!");
      return 1;
    }*/
    //else
    //{
      // Set the end time and calculate the time consumption after planning
      std::chrono::steady_clock::time_point end_time = std::chrono::steady_clock::now();
      moveit_visual_tools.trigger();
      double duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count() / 1000.0;
      draw_title("Time:"+std::to_string(duration), text_pose1);
      moveit_visual_tools.trigger();
      
      const moveit::core::RobotStatePtr& robot_state = move_group_interface.getCurrentState();
      const moveit::core::JointModelGroup* joint_model_group = robot_state->getJointModelGroup("panda_arm");

      double smooth;

      for (size_t i = 0; i < final_plan.trajectory_.joint_trajectory.points.size(); ++i)
      {
        const trajectory_msgs::msg::JointTrajectoryPoint& trajectory_point = final_plan.trajectory_.joint_trajectory.points[i];
        std::vector<double> joint_values = trajectory_point.positions;

        // Print the joint values at each waypoint
        for (size_t j = 1; j < joint_values.size(); ++j)
        {
          const moveit::core::JointModel* joint_model = joint_model_group->getJointModel(joint_model_group->getJointModelNames()[j]);
          const std::string& joint_name = joint_model->getName();
          smooth = (smooth < std::abs(joint_values[j]-joint_values[j-1])) ? std::abs(joint_values[j]-joint_values[j-1]) : smooth;
        }
      }
      draw_title("Time:"+std::to_string(duration)+"Smooth:"+std::to_string(smooth), text_pose1);
      moveit_visual_tools.trigger();
      // Execute the CHOMP-optimized trajectory
      draw_trajectory_tool_path(final_plan.trajectory_);
      moveit_visual_tools.trigger();
      prompt("Press 'Next' in the RvizVisualToolsGui window to execute");
      draw_title("Time:"+std::to_string(duration)+"Smooth:"+std::to_string(smooth), text_pose1);
      moveit_visual_tools.trigger();
      move_group_interface.execute(final_plan);
    //}
  }
  else
  {
    draw_title("Planning Failed!", text_pose1);
    moveit_visual_tools.trigger();
    RCLCPP_ERROR(logger, "Planning failed!");
  }
  // Shutdown ROS
  rclcpp::shutdown();
  spinner.join();  // <--- Join the thread before exiting
  return 0;
}