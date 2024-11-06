#include <control_msgs/action/parallel_gripper_command.hpp>
#include <memory>
#include <moveit_ros_control_interface/ControllerHandle.h>
#include <moveit_simple_controller_manager/action_based_controller_handle.h>
#include <pluginlib/class_list_macros.hpp>

namespace moveit_ros_control_interface {
/*
 * This is an interface for a gripper using control_msgs/ParallelGripperCommand
 * action interface (single DOF).
 */
class ParallelGripperControllerHandle
    : public moveit_simple_controller_manager::ActionBasedControllerHandle<
          control_msgs::action::ParallelGripperCommand> {
public:
  /* Topics will map to name/ns/goal, name/ns/result, etc */
  ParallelGripperControllerHandle(const rclcpp::Node::SharedPtr &node,
                                  const std::string &name,
                                  const std::string &ns,
                                  const std::vector<std::string> &resources,
                                  const double max_effort = 0.0)
      : ActionBasedControllerHandle<
            control_msgs::action::ParallelGripperCommand>(
            node, name, ns, "parallel_gripper_controller_handle"),
        allow_failure_(false), parallel_jaw_gripper_(false),
        max_effort_(max_effort),
        command_joints_(resources.begin(), resources.end()) {}

  bool
  sendTrajectory(const moveit_msgs::msg::RobotTrajectory &trajectory) override {
    RCLCPP_DEBUG_STREAM(logger_, "Received new trajectory for " << name_);

    if (!controller_action_client_)
      return false;

    if (!isConnected()) {
      RCLCPP_ERROR_STREAM(
          logger_,
          "Action client not connected to action server: " << getActionName());
      return false;
    }

    if (!trajectory.multi_dof_joint_trajectory.points.empty()) {
      RCLCPP_ERROR(logger_, "Gripper cannot execute multi-dof trajectories.");
      return false;
    }

    if (trajectory.joint_trajectory.points.empty()) {
      RCLCPP_ERROR(
          logger_,
          "GripperController requires at least one joint trajectory point.");
      return false;
    }

    if (trajectory.joint_trajectory.points.size() > 1) {
      RCLCPP_DEBUG_STREAM(logger_,
                          "Trajectory: " << trajectory_msgs::msg::to_yaml(
                              trajectory.joint_trajectory));
    }

    if (trajectory.joint_trajectory.joint_names.empty()) {
      RCLCPP_ERROR(logger_, "No joint names specified");
      return false;
    }

    std::vector<std::size_t> gripper_joint_indexes;
    for (std::size_t i = 0; i < trajectory.joint_trajectory.joint_names.size();
         ++i) {
      if (command_joints_.find(trajectory.joint_trajectory.joint_names[i]) !=
          command_joints_.end()) {
        gripper_joint_indexes.push_back(i);
        if (!parallel_jaw_gripper_)
          break;
      }
    }

    if (gripper_joint_indexes.empty()) {
      RCLCPP_WARN(
          logger_,
          "No command_joint was specified for the MoveIt controller parallel gripper handle. \
                      Please see ParallelGripperControllerHandle::addCommandJoint() and \
                      ParallelGripperControllerHandle::setCommandJoint(). Assuming index 0.");
      gripper_joint_indexes.push_back(0);
    }

    // goal to be sent
    control_msgs::action::ParallelGripperCommand::Goal goal;
    goal.command.position.resize(gripper_joint_indexes.size(), 0.0);
    goal.command.effort.resize(gripper_joint_indexes.size(), 0.0);

    // send last point
    int tpoint = trajectory.joint_trajectory.points.size() - 1;
    RCLCPP_DEBUG(logger_, "Sending command from trajectory point %d", tpoint);

    // fill in goal from last point
    for (std::size_t i = 0; i < gripper_joint_indexes.size(); ++i) {
      const auto idx = gripper_joint_indexes[i];
      if (trajectory.joint_trajectory.points[tpoint].positions.size() <= idx) {
        RCLCPP_ERROR(
            logger_,
            "ParallelGripperController expects a joint trajectory with one \
                         point that specifies at least the position of joint \
                         '%s', but insufficient positions provided",
            trajectory.joint_trajectory.joint_names[idx].c_str());
        return false;
      }
      goal.command.position[i] +=
          trajectory.joint_trajectory.points[tpoint].positions[idx];

      // TODO: Should we enforce that effort is always specified?
      if (trajectory.joint_trajectory.points[tpoint].effort.size() > idx) {
        goal.command.effort[i] =
            trajectory.joint_trajectory.points[tpoint].effort[idx];
      } else {
        goal.command.effort[i] = max_effort_;
      }
    }
    rclcpp_action::Client<control_msgs::action::ParallelGripperCommand>::
        SendGoalOptions send_goal_options;
    // Active callback
    send_goal_options.goal_response_callback =
        [this](const rclcpp_action::Client<
               control_msgs::action::ParallelGripperCommand>::GoalHandle::
                   SharedPtr &
               /* unused-arg */) {
          RCLCPP_DEBUG_STREAM(logger_, name_ << " started execution");
        };
    // Send goal
    auto current_goal_future =
        controller_action_client_->async_send_goal(goal, send_goal_options);
    current_goal_ = current_goal_future.get();
    if (!current_goal_) {
      RCLCPP_ERROR(logger_, "Goal was rejected by server");
      return false;
    }

    done_ = false;
    last_exec_ = moveit_controller_manager::ExecutionStatus::RUNNING;
    return true;
  }

  void setCommandJoint(const std::string &name) {
    command_joints_.clear();
    addCommandJoint(name);
  }

  void addCommandJoint(const std::string &name) {
    command_joints_.insert(name);
  }

  void allowFailure(bool allow) { allow_failure_ = allow; }

  void setParallelJawGripper(const std::string &left,
                             const std::string &right) {
    addCommandJoint(left);
    addCommandJoint(right);
    parallel_jaw_gripper_ = true;
  }

private:
  void controllerDoneCallback(
      const rclcpp_action::ClientGoalHandle<
          control_msgs::action::ParallelGripperCommand>::WrappedResult
          &wrapped_result) override {
    if (wrapped_result.code == rclcpp_action::ResultCode::ABORTED &&
        allow_failure_) {
      finishControllerExecution(rclcpp_action::ResultCode::SUCCEEDED);
    } else {
      finishControllerExecution(wrapped_result.code);
    }
  }

  /*
   * Some gripper drivers may indicate a failure if they do not close all the
   * way when an object is in the gripper.
   */
  bool allow_failure_;

  /*
   * A common setup is where there are two joints that each move
   * half the overall distance. Thus, the command to the gripper
   * should be the sum of the two joint distances.
   *
   * When this is set, command_joints_ should be of size 2,
   * and the command will be the sum of the two joints.
   */
  bool parallel_jaw_gripper_;

  /*
   * The ``max_effort`` used in the GripperCommand message when no
   * ``max_effort`` was specified in the requested trajectory. Defaults to
   * ``0.0``.
   */
  double max_effort_;

  /*
   * A GripperCommand message has only a single float64 for the
   * "command", thus only a single joint angle can be sent -- however,
   * due to the complexity of making grippers look correct in a URDF,
   * they typically have >1 joints. The "command_joint" is the joint
   * whose position value will be sent in the GripperCommand action. A
   * set of names is provided for acceptable joint names. If any of
   * the joints specified is found, the value corresponding to that
   * joint is considered the command.
   */
  std::set<std::string> command_joints_;
};

class ParallelGripperControllerAllocator : public ControllerHandleAllocator {
public:
  moveit_controller_manager::MoveItControllerHandlePtr
  alloc(const rclcpp::Node::SharedPtr &node, const std::string &name,
        const std::vector<std::string> &resources) override {
    return std::make_shared<ParallelGripperControllerHandle>(
        node, name, "gripper_cmd", resources);
  }
};

} // namespace moveit_ros_control_interface

PLUGINLIB_EXPORT_CLASS(
    moveit_ros_control_interface::ParallelGripperControllerAllocator,
    moveit_ros_control_interface::ControllerHandleAllocator);
