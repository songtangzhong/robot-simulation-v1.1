#ifndef ROBOT_INFO_
#define ROBOT_INFO_

#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <vector>

// Define the condition macro to select if the end_effector is used.
#define END_EFFECTOR_TRUE  1  // use
//#define END_EFFECTOR_TRUE  0  // not use

// Define the macro of robot arm dof.
#define ARM_DOF 7

#if END_EFFECTOR_TRUE
// Define the macro of end-effector dof.
#define END_EFF_DOF 2
#endif

namespace robot_info
{
class RobotInfo
{
public:
    RobotInfo();
    ~RobotInfo();

    // Control mode of robot.
    const unsigned int position_mode_ = (1<<0);
    const unsigned int velocity_mode_ = (1<<1);
    const unsigned int effort_mode_ = (1<<2);

    // Robot arm degree of freedom.
    unsigned int arm_dof_;

    // Robot arm joint names.
    std::vector<std::string> arm_joint_names_;

    // Store current arm joint states.
    std::vector<double> cur_arm_joint_positions_;
    std::vector<double> cur_arm_joint_velocities_;
    std::vector<double> cur_arm_joint_efforts_;

    // Store current arm joint commands.
    std::vector<double> cmd_arm_joint_positions_;
    std::vector<double> cmd_arm_joint_velocities_;
    std::vector<double> cmd_arm_joint_efforts_;

    // Stroe the control mode of arm joints.
    std::vector<unsigned int> arm_control_modes_;

#if END_EFFECTOR_TRUE
    // End-effector degree of freedom.
    unsigned int end_eff_dof_;

    // End-effector joint names.
    std::vector<std::string> end_eff_joint_names_;

    // Store current end-effector joint states.
    std::vector<double> cur_end_eff_joint_positions_;
    std::vector<double> cur_end_eff_joint_velocities_;
    std::vector<double> cur_end_eff_joint_efforts_;

    // Store current end-effector joint commands.
    std::vector<double> cmd_end_eff_joint_positions_;
    std::vector<double> cmd_end_eff_joint_velocities_;
    std::vector<double> cmd_end_eff_joint_efforts_;

    // Stroe the control mode of end-effector joints.
    std::vector<unsigned int> end_eff_control_modes_;
#endif

    // Key value of shared memory to robot arm.
    key_t arm_shm_key_;
    // Key value of semaphore to robot arm shared memory.
    key_t arm_sem_key_;

#if END_EFFECTOR_TRUE
    // Key value of shared memory to end-effector.
    key_t end_eff_shm_key_;
    // Key value of semaphore to end-effector shared memory.
    key_t end_eff_sem_key_;
#endif

}; // end class RobotInfo

} // end namespace robot_info

#endif
