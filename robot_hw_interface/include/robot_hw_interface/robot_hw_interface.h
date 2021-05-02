#ifndef ROBOT_HW_INTERFACE_H_
#define ROBOT_HW_INTERFACE_H_

#include <memory>
#include <string>
#include <vector>
#include <hardware_interface/base_interface.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_status_values.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <robot_info/robot_info.h>
#include <shared_memory/shared_memory.h>
#include <shared_memory/semaphore.h>

namespace robot_hw
{
class RobotHardware : public
  hardware_interface::BaseInterface<hardware_interface::SystemInterface>
{
public:

  hardware_interface::return_type configure(const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type start() override;

  hardware_interface::return_type stop() override;

  hardware_interface::return_type read() override;

  hardware_interface::return_type write() override;

private:
  double start_duration_sec_;
  double stop_duration_sec_;

  // store robot information
  std::shared_ptr<robot_info::RobotInfo> robot = std::make_shared<robot_info::RobotInfo>();

  // pointer to arm shared memory.
  shm::ArmShm *arm_shm_ptr;
  // ID to arm shared memory.
  int arm_shm_id;
  // ID to arm semaphore.
  int arm_sem_id;

#if END_EFFECTOR_TRUE
  // pointer to end-effector shared memory.
  shm::EndEffShm *end_eff_shm_ptr;
  // ID to end-effector shared memory.
  int end_eff_shm_id;
  // ID to end-effector semaphore.
  int end_eff_sem_id;
#endif
};

}  // end namespace robot_hw

#endif
