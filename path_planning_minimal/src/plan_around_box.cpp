#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

#include <thread>
#include <chrono>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>

static geometry_msgs::msg::Pose makePose(double x, double y, double z,
                                         double qw = 1.0, double qx = 0.0,
                                         double qy = 0.0, double qz = 0.0)
{
  geometry_msgs::msg::Pose p;
  p.position.x = x; p.position.y = y; p.position.z = z;
  p.orientation.w = qw; p.orientation.x = qx; p.orientation.y = qy; p.orientation.z = qz;
  return p;
}

static bool waitForObject(moveit::planning_interface::PlanningSceneInterface& scene,
                          const std::string& id, double timeout_s = 2.0)
{
  const auto t0 = std::chrono::steady_clock::now();
  while (true) {
    auto names = scene.getKnownObjectNames(false);
    if (std::find(names.begin(), names.end(), id) != names.end()) return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    if (std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count() > timeout_s) return false;
  }
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions opts;
  opts.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("panda_plan_around_box", opts);

  if (!node->has_parameter("interactive")) {
   node->declare_parameter<bool>("interactive", false);
  }
  if (!node->has_parameter("loop_mode")) {
   node->declare_parameter<bool>("loop_mode", true);
  }
  bool interactive = false, loop_mode = true;
  node->get_parameter("interactive", interactive);
  node->get_parameter("loop_mode", loop_mode);


  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  auto spinner = std::make_shared<std::thread>([&exec]() { exec.spin(); });

  const std::string group_name = "panda_arm";
  const std::string ee_link    = "panda_link8";

  moveit::planning_interface::MoveGroupInterface move_group(node, group_name);
  moveit::planning_interface::PlanningSceneInterface scene;

  move_group.setPlanningTime(15.0);
  move_group.setNumPlanningAttempts(5);
  move_group.setMaxVelocityScalingFactor(0.5);
  move_group.setMaxAccelerationScalingFactor(0.5);

  const std::string planning_frame = move_group.getPlanningFrame();
  RCLCPP_INFO(node->get_logger(), "Planning frame: %s", planning_frame.c_str());

  
  const geometry_msgs::msg::Pose A = makePose(0.38, -0.28, 0.45, 1.0);
  const geometry_msgs::msg::Pose B = makePose(0.72,  0.28, 0.45, 1.0);

  move_group.setPoseTarget(A, ee_link);
  moveit::planning_interface::MoveGroupInterface::Plan to_A;
  if (move_group.plan(to_A) != moveit::core::MoveItErrorCode::SUCCESS ||
      move_group.execute(to_A) != moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to reach START pose A. Tweak A if needed.");
    exec.cancel(); spinner->join(); rclcpp::shutdown(); return 1;
  }
  rclcpp::sleep_for(std::chrono::milliseconds(300));

  const std::string cube_id = "blocking_cube";
  const double cube_side = 0.22;
  const double min_clear = 0.06;

  geometry_msgs::msg::Pose cube_pose = makePose(
      0.5 * (A.position.x + B.position.x),
      0.5 * (A.position.y + B.position.y),
      0.5 * (A.position.z + B.position.z), 1.0);

  {
    const double hx = cube_side * 0.5, hy = cube_side * 0.5, hz = cube_side * 0.5;
    double dx = std::abs(cube_pose.position.x - A.position.x);
    double dy = std::abs(cube_pose.position.y - A.position.y);
    double dz = std::abs(cube_pose.position.z - A.position.z);
    if (dx < (hx + min_clear) && dy < (hy + min_clear) && dz < (hz + min_clear)) {
      const double needed = (hz + min_clear) - dz + 0.01; // +1cm
      cube_pose.position.z += needed;
      RCLCPP_WARN(node->get_logger(), "Cube nudged +z by %.3f m to avoid initial collision.", needed);
    }
  }

  shape_msgs::msg::SolidPrimitive prim;
  prim.type = prim.BOX;
  prim.dimensions = {cube_side, cube_side, cube_side};

  moveit_msgs::msg::CollisionObject cube;
  cube.header.frame_id = planning_frame;
  cube.id = cube_id;
  cube.primitives.push_back(prim);
  cube.primitive_poses.push_back(cube_pose);
  cube.operation = cube.ADD;

  scene.applyCollisionObjects({cube});
  (void)waitForObject(scene, cube_id, 2.0);
  RCLCPP_INFO(node->get_logger(), "Cube inserted at midpoint (%.3f, %.3f, %.3f).",
              cube_pose.position.x, cube_pose.position.y, cube_pose.position.z);

  if (interactive) {

    RCLCPP_INFO(node->get_logger(),
      "Interactive mode ON:\n"
      " - In RViz MotionPlanning panel:\n"
      "   • Start State: click 'Use Current State' (this sets the initial pose)\n"
      "   • Goal State: use the Pose Goal widget or drag the interactive marker\n"
      "   • Click 'Plan' then 'Execute'.\n"
      "The cube stays in the scene for collision-avoidance.");

    rclcpp::Rate r(30);
    while (rclcpp::ok()) r.sleep();
  } else if (loop_mode) {

    bool to_B = true;
    size_t iter = 0;
    while (rclcpp::ok()) {
      const auto goal = to_B ? B : A;
      move_group.setStartStateToCurrentState();
      move_group.setPoseTarget(goal, ee_link);

      moveit::planning_interface::MoveGroupInterface::Plan plan;
      const bool ok = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
      RCLCPP_INFO(node->get_logger(), "[%zu] Plan to %s (avoid cube): %s",
                  iter, (to_B ? "B" : "A"), ok ? "SUCCESS" : "FAILED");
      if (ok) (void)move_group.execute(plan);
      rclcpp::sleep_for(std::chrono::milliseconds(900));
      to_B = !to_B; ++iter;
    }
  }

  scene.removeCollisionObjects({cube_id});
  exec.cancel(); spinner->join(); rclcpp::shutdown();
  return 0;
}

