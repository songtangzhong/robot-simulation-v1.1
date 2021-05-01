#include <shared_memory/shared_memory.h>
#include <robot_info/robot_info.h>
#include <rclcpp/rclcpp.hpp>
#include <iostream>

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    std::shared_ptr<robot_info::RobotInfo> robot = std::make_shared<robot_info::RobotInfo>();

    // pointer to arm shared memory.
    shm::ArmShm *arm_shm_ptr;
    // ID to arm shared memory.
    int arm_shm_id;

    arm_shm_id = shm::create_shm(robot->arm_shm_key_, &arm_shm_ptr);
    if (arm_shm_id != SHM_STATE_NO)
    {
        std::cout << "Create arm shared memory successfully." << std::endl;
        std::cout << "arm_shm_id: " << arm_shm_id << std::endl;
    }
    else
    {
        std::cout << "Create arm shared memory failed." << std::endl;
        return 0;
    }

#if END_EFFECTOR_TRUE
    // pointer to end-effector shared memory.
    shm::EndEffShm *end_eff_shm_ptr;
    // ID to end-effector shared memory.
    int end_eff_shm_id;

    end_eff_shm_id = shm::create_shm(robot->end_eff_shm_key_, &end_eff_shm_ptr);
    if (end_eff_shm_id != SHM_STATE_NO)
    {
        std::cout << "Create end-effector shared memory successfully." << std::endl;
        std::cout << "end_eff_shm_id: " << end_eff_shm_id << std::endl;
    }
    else
    {
        std::cout << "Create end-effector shared memory failed." << std::endl;
        return 0;
    }
#endif

    for (unsigned int j=0; j< robot->arm_dof_; j++)
    {
        arm_shm_ptr->cur_arm_joint_positions_[j] = 1;
        arm_shm_ptr->cur_arm_joint_velocities_[j] = 1;
        arm_shm_ptr->cur_arm_joint_efforts_[j] = 1;

        arm_shm_ptr->cmd_arm_joint_positions_[j] = 1;
        arm_shm_ptr->cmd_arm_joint_velocities_[j] = 1;
        arm_shm_ptr->cmd_arm_joint_efforts_[j] = 1;

        arm_shm_ptr->arm_control_modes_[j] = 1;
    }

#if END_EFFECTOR_TRUE
    for (unsigned int j=0; j< robot->end_eff_dof_; j++)
    {
        end_eff_shm_ptr->cur_end_eff_joint_positions_[j] = 1;
        end_eff_shm_ptr->cur_end_eff_joint_velocities_[j] = 1;
        end_eff_shm_ptr->cur_end_eff_joint_efforts_[j] = 1;

        end_eff_shm_ptr->cmd_end_eff_joint_positions_[j] = 1;
        end_eff_shm_ptr->cmd_end_eff_joint_velocities_[j] = 1;
        end_eff_shm_ptr->cmd_end_eff_joint_efforts_[j] = 1;

        end_eff_shm_ptr->end_eff_control_modes_[j] = 1;
    }
#endif

    std::this_thread::sleep_for(std::chrono::seconds(5));

    for (unsigned int j=0; j< robot->arm_dof_; j++)
    {
        std::cout << "arm_shm_ptr->cur_arm_joint_positions_[" << j << "]: " << arm_shm_ptr->cur_arm_joint_positions_[j] << std::endl;
        std::cout << "arm_shm_ptr->cur_arm_joint_velocities_[" << j << "]: " << arm_shm_ptr->cur_arm_joint_velocities_[j] << std::endl;
        std::cout << "arm_shm_ptr->cur_arm_joint_efforts_[" << j << "]: " << arm_shm_ptr->cur_arm_joint_efforts_[j] << std::endl;

        std::cout << "arm_shm_ptr->cmd_arm_joint_positions_[" << j << "]: " << arm_shm_ptr->cmd_arm_joint_positions_[j] << std::endl;
        std::cout << "arm_shm_ptr->cmd_arm_joint_velocities_[" << j << "]: " << arm_shm_ptr->cmd_arm_joint_velocities_[j] << std::endl;
        std::cout << "arm_shm_ptr->cmd_arm_joint_efforts_[" << j << "]: " << arm_shm_ptr->cmd_arm_joint_efforts_[j] << std::endl;

        std::cout << "arm_shm_ptr->arm_control_modes_[" << j << "]: " << arm_shm_ptr->arm_control_modes_[j] << std::endl;
    }

#if END_EFFECTOR_TRUE
    for (unsigned int j=0; j< robot->end_eff_dof_; j++)
    {
        std::cout << "end_eff_shm_ptr->cur_end_eff_joint_positions_[" << j << "]: " << end_eff_shm_ptr->cur_end_eff_joint_positions_[j] << std::endl;
        std::cout << "end_eff_shm_ptr->cur_end_eff_joint_velocities_[" << j << "]: " << end_eff_shm_ptr->cur_end_eff_joint_velocities_[j] << std::endl;
        std::cout << "end_eff_shm_ptr->cur_end_eff_joint_efforts_[" << j << "]: " << end_eff_shm_ptr->cur_end_eff_joint_efforts_[j] << std::endl;

        std::cout << "end_eff_shm_ptr->cmd_end_eff_joint_positions_[" << j << "]: " << end_eff_shm_ptr->cmd_end_eff_joint_positions_[j] << std::endl;
        std::cout << "end_eff_shm_ptr->cmd_end_eff_joint_velocities_[" << j << "]: " << end_eff_shm_ptr->cmd_end_eff_joint_velocities_[j] << std::endl;
        std::cout << "end_eff_shm_ptr->cmd_end_eff_joint_efforts_[" << j << "]: " << end_eff_shm_ptr->cmd_end_eff_joint_efforts_[j] << std::endl;

        std::cout << "end_eff_shm_ptr->end_eff_control_modes_[" << j << "]: " << end_eff_shm_ptr->end_eff_control_modes_[j] << std::endl;
    }
#endif

    if (shm::release_shm(arm_shm_id, &arm_shm_ptr) != SHM_STATE_OK)
    {
        std::cout << "Failed to release arm shared memory." << std::endl;
        return 0;
    }
    else
    {
        std::cout << "Release arm shared memory successfully." << std::endl;
    }

#if END_EFFECTOR_TRUE
    if (shm::release_shm(end_eff_shm_id, &end_eff_shm_ptr) != SHM_STATE_OK)
    {
        std::cout << "Failed to release end-effector shared memory." << std::endl;
        return 0;
    }
    else
    {
        std::cout << "Release end-effector shared memory successfully." << std::endl;
    }
#endif

    return 0;
}