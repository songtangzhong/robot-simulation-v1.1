#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <robot_hw_interface/robot_hw_interface.h>

namespace robot_hw
{
hardware_interface::return_type RobotHardware::configure(
  const hardware_interface::HardwareInfo & info)
{
  if (configure_default(info) != hardware_interface::return_type::OK) {
    return hardware_interface::return_type::ERROR;
  }

  start_duration_sec_ = stod(info_.hardware_parameters["start_duration_sec"]);
  stop_duration_sec_ = stod(info_.hardware_parameters["stop_duration_sec"]);

  // Create arm shared memory.
  arm_shm_id = shm::create_shm(robot->arm_shm_key_, &arm_shm_ptr);
  if (arm_shm_id != SHM_STATE_NO)
  {
      std::cout << "Create arm shared memory successfully." << std::endl;
  }
  else
  {
      std::cout << "Create arm shared memory failed." << std::endl;
  }

  // Create arm semaphore.
  arm_sem_id = sem::create_semaphore(robot->arm_sem_key_);
  if (arm_sem_id != SEM_STATE_NO)
  {
      std::cout << "Create arm semaphore successfully." << std::endl;
  }
  else
  {
      std::cout << "Create arm semaphore failed." << std::endl;
  }

#if END_EFFECTOR_TRUE
  // Create end-effector shared memory.
  end_eff_shm_id = shm::create_shm(robot->end_eff_shm_key_, &end_eff_shm_ptr);
  if (end_eff_shm_id != SHM_STATE_NO)
  {
      std::cout << "Create end-effector shared memory successfully." << std::endl;
  }
  else
  {
      std::cout << "Create end-effector shared memory failed." << std::endl;
  }

  // Create end-effector semaphore.
  end_eff_sem_id = sem::create_semaphore(robot->end_eff_sem_key_);
  if (end_eff_sem_id != SEM_STATE_NO)
  {
      std::cout << "Create end-effector semaphore successfully." << std::endl;
  }
  else
  {
      std::cout << "Create end-effector semaphore failed." << std::endl;
  }
#endif

  status_ = hardware_interface::status::CONFIGURED;
  return hardware_interface::return_type::OK;
}

std::vector<hardware_interface::StateInterface>
RobotHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (unsigned int j = 0; j < robot->arm_dof_; j++) 
  {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        robot->arm_joint_names_[j], hardware_interface::HW_IF_POSITION, &robot->cur_arm_joint_positions_[j]));
  }
  for (unsigned int j = 0; j < robot->arm_dof_; j++) 
  {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        robot->arm_joint_names_[j], hardware_interface::HW_IF_VELOCITY, &robot->cur_arm_joint_velocities_[j]));
  }
  for (unsigned int j = 0; j < robot->arm_dof_; j++) 
  {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        robot->arm_joint_names_[j], hardware_interface::HW_IF_EFFORT, &robot->cur_arm_joint_efforts_[j]));
  }

#if END_EFFECTOR_TRUE
  for (unsigned int j = 0; j < robot->end_eff_dof_; j++) 
  {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        robot->end_eff_joint_names_[j], hardware_interface::HW_IF_POSITION, &robot->cur_end_eff_joint_positions_[j]));
  }
  for (unsigned int j = 0; j < robot->end_eff_dof_; j++) 
  {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        robot->end_eff_joint_names_[j], hardware_interface::HW_IF_VELOCITY, &robot->cur_end_eff_joint_velocities_[j]));
  }
  for (unsigned int j = 0; j < robot->end_eff_dof_; j++) 
  {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        robot->end_eff_joint_names_[j], hardware_interface::HW_IF_EFFORT, &robot->cur_end_eff_joint_efforts_[j]));
  }
#endif

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
RobotHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (unsigned int j = 0; j < robot->arm_dof_; j++) 
  {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        robot->arm_joint_names_[j], hardware_interface::HW_IF_POSITION, &robot->cmd_arm_joint_positions_[j]));
  }
  for (unsigned int j = 0; j < robot->arm_dof_; j++) 
  {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        robot->arm_joint_names_[j], hardware_interface::HW_IF_VELOCITY, &robot->cmd_arm_joint_velocities_[j]));
  }
  for (unsigned int j = 0; j < robot->arm_dof_; j++) 
  {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        robot->arm_joint_names_[j], hardware_interface::HW_IF_EFFORT, &robot->cmd_arm_joint_efforts_[j]));
  }

  return command_interfaces;
}


hardware_interface::return_type RobotHardware::start()
{
  RCLCPP_INFO(
    rclcpp::get_logger("RobotHardware"),
    "Starting ...please wait...");

  for (int i = 0; i <= start_duration_sec_; i++) 
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(
      rclcpp::get_logger("RobotHardware"),
      "%.1f seconds left...", start_duration_sec_ - i);
  }

  status_ = hardware_interface::status::STARTED;

  RCLCPP_INFO(
    rclcpp::get_logger("RobotHardware"),
    "System Sucessfully started!");

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RobotHardware::stop()
{
  RCLCPP_INFO(
    rclcpp::get_logger("RobotHardware"),
    "Stopping ...please wait...");

  for (int i = 0; i <= stop_duration_sec_; i++) {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(
      rclcpp::get_logger("RobotHardware"),
      "%.1f seconds left...", stop_duration_sec_ - i);
  }

  status_ = hardware_interface::status::STOPPED;

  RCLCPP_INFO(
    rclcpp::get_logger("RobotHardware"),
    "System sucessfully stopped!");

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RobotHardware::read()
{
  // Get arm semaphore.
  sem::semaphore_p(arm_sem_id);
  for (unsigned int j = 0; j < robot->arm_dof_; j++) 
  {
    // read robot arm states from shared memory to current state variables.
    robot->cur_arm_joint_positions_[j] = arm_shm_ptr->cur_arm_joint_positions_[j];
    robot->cur_arm_joint_velocities_[j] = arm_shm_ptr->cur_arm_joint_velocities_[j];
    robot->cur_arm_joint_efforts_[j] = arm_shm_ptr->cur_arm_joint_efforts_[j];
  }

#if END_EFFECTOR_TRUE
  // Get end-effector semaphore.
  sem::semaphore_p(end_eff_sem_id);
  for (unsigned int j = 0; j < robot->end_eff_dof_; j++) 
  {
    // read robot end-effector states from shared memory to current state variables.
    robot->cur_end_eff_joint_positions_[j] = end_eff_shm_ptr->cur_end_eff_joint_positions_[j];
    robot->cur_end_eff_joint_velocities_[j] = end_eff_shm_ptr->cur_end_eff_joint_velocities_[j];
    robot->cur_end_eff_joint_efforts_[j] = end_eff_shm_ptr->cur_end_eff_joint_efforts_[j];
  }
  // Release end-effector semaphore.
  sem::semaphore_v(end_eff_sem_id);
#endif

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RobotHardware::write()
{
  // read arm control mode from shared memory.
  // write arm joint commands to shared memory.
  for (unsigned int j = 0; j < robot->arm_dof_; j++) 
  {
    if ((robot->arm_control_modes_[j]=arm_shm_ptr->arm_control_modes_[j]) & robot->position_mode_) 
    {
      arm_shm_ptr->cmd_arm_joint_positions_[j] = robot->cmd_arm_joint_positions_[j];
    }
    else if ((robot->arm_control_modes_[j]=arm_shm_ptr->arm_control_modes_[j]) & robot->velocity_mode_) 
    {
      arm_shm_ptr->cmd_arm_joint_velocities_[j] = robot->cmd_arm_joint_velocities_[j];
    }
    else if ((robot->arm_control_modes_[j]=arm_shm_ptr->arm_control_modes_[j]) & robot->effort_mode_) 
    {
      arm_shm_ptr->cmd_arm_joint_efforts_[j] = robot->cmd_arm_joint_efforts_[j];
    }
  }
  // Release arm semaphore.
  sem::semaphore_v(arm_sem_id);
  
  return hardware_interface::return_type::OK;
}

}  // namespace robot_hw

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  robot_hw::RobotHardware,
  hardware_interface::SystemInterface
)
