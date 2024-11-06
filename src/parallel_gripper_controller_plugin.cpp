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
                                  const std::vector<std::string> &resources)
      : ActionBasedControllerHandle<
            control_msgs::action::ParallelGripperCommand>(
            node, name, ns, "parallel_gripper_controller_handle") {
    std::set<std::string_view> command_joints(resources.begin(),
                                              resources.end());
    if (command_joints.size() != 1) {
      RCLCPP_ERROR_STREAM(logger_, "ParallelGripperControllerHandle expects "
                                   "exactly one command joint, but got "
                                       << command_joints.size());
    }
    command_joint_ = *command_joints.begin();
  }

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
      RCLCPP_ERROR(logger_, "ParallelGripperController requires at least one "
                            "joint trajectory point.");
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

    const auto command_joint_it = std::find(
        trajectory.joint_trajectory.joint_names.begin(),
        trajectory.joint_trajectory.joint_names.end(), command_joint_);
    if (command_joint_it == trajectory.joint_trajectory.joint_names.end()) {
      RCLCPP_ERROR_STREAM(logger_, "Input trajectory doesn't contain the "
                                   "command joint: "
                                       << command_joint_);
      return false;
    }
    std::size_t gripper_joint_index = std::distance(
        trajectory.joint_trajectory.joint_names.begin(), command_joint_it);

    // goal to be sent
    control_msgs::action::ParallelGripperCommand::Goal goal;
    goal.command.position = {0.0};

    // send last point
    int tpoint = trajectory.joint_trajectory.points.size() - 1;
    RCLCPP_DEBUG(logger_, "Sending command from trajectory point %d", tpoint);

    if (trajectory.joint_trajectory.points[tpoint].positions.size() <=
        gripper_joint_index) {
      RCLCPP_ERROR(
          logger_,
          "ParallelGripperController expects a joint trajectory with one \
                         point that specifies at least the position of joint \
                         '%s', but insufficient positions provided",
          trajectory.joint_trajectory.joint_names[gripper_joint_index].c_str());
      return false;
    }
    goal.command.position[0] += trajectory.joint_trajectory.points[tpoint]
                                    .positions[gripper_joint_index];

    if (trajectory.joint_trajectory.points[tpoint].effort.size() >
        gripper_joint_index) {
      goal.command.effort = {trajectory.joint_trajectory.points[tpoint]
                                 .effort[gripper_joint_index]};
    } else {
      RCLCPP_WARN_STREAM(
          logger_,
          "No effort specified for joint "
              << trajectory.joint_trajectory.joint_names[gripper_joint_index]
              << ". Sending without effort.");
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

private:
  void controllerDoneCallback(
      const rclcpp_action::ClientGoalHandle<
          control_msgs::action::ParallelGripperCommand>::WrappedResult
          &wrapped_result) override {
    if (wrapped_result.code == rclcpp_action::ResultCode::ABORTED) {
      finishControllerExecution(rclcpp_action::ResultCode::SUCCEEDED);
    } else {
      finishControllerExecution(wrapped_result.code);
    }
  }

  /*
   * A ParallelGripperCommand message has only a single float64 for the
   * "command", thus only a single joint angle can be sent */
  std::string command_joint_;
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
