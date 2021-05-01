#include <robot_info/robot_info.h>
#include <shared_memory/shared_memory.h>
#include <shared_memory/semaphore.h>
#include <rclcpp/rclcpp.hpp>
#include <iostream>

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    // Store the information of robot.
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

    // ID to arm semaphore.
    int arm_sem_id;

    arm_sem_id = sem::create_semaphore(robot->arm_sem_key_);
    if (arm_sem_id != SEM_STATE_NO)
    {
        std::cout << "Create arm semaphore successfully." << std::endl;
        std::cout << "arm_sem_id: " << arm_sem_id << std::endl;
    }
    else
    {
        std::cout << "Create arm semaphore failed." << std::endl;
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

    // ID to end-effector semaphore.
    int end_eff_sem_id;

    end_eff_sem_id = sem::create_semaphore(robot->end_eff_sem_key_);
    if (end_eff_sem_id != SEM_STATE_NO)
    {
        std::cout << "Create end-effector semaphore successfully." << std::endl;
        std::cout << "end_eff_sem_id: " << end_eff_sem_id << std::endl;
    }
    else
    {
        std::cout << "Create end-effector semaphore failed." << std::endl;
        return 0;
    }
#endif

    rclcpp::WallRate loop_rate(1000);
    unsigned int i = 0;
    while (rclcpp::ok())
    {
        sem::semaphore_p(arm_sem_id);
        for (unsigned int j=0; j< robot->arm_dof_; j++)
        {
            arm_shm_ptr->arm_control_modes_[j] = robot->position_mode_;
            //arm_shm_ptr->arm_control_modes_[j] = robot->velocity_mode_;
            //arm_shm_ptr->arm_control_modes_[j] = robot->effort_mode_;

            arm_shm_ptr->cmd_arm_joint_positions_[j] = arm_shm_ptr->cur_arm_joint_positions_[j]+0.001;
            arm_shm_ptr->cmd_arm_joint_velocities_[j] = 0;
            arm_shm_ptr->cmd_arm_joint_efforts_[j] = 0;
        }
        sem::semaphore_v(arm_sem_id);
        loop_rate.sleep();
        i++;
        if (i>1000)
        {
            std::cout << "arm execute successfully." << std::endl;
            break;
        }
    }

#if END_EFFECTOR_TRUE
    i = 0;
    while (rclcpp::ok())
    {
        sem::semaphore_p(end_eff_sem_id);
        for (unsigned int j=0; j< robot->end_eff_dof_; j++)
        {
            end_eff_shm_ptr->end_eff_control_modes_[j] = robot->position_mode_;
            //end_eff_shm_ptr->end_eff_control_modes_[j] = robot->velocity_mode_;
            //end_eff_shm_ptr->end_eff_control_modes_[j] = robot->effort_mode_;

            end_eff_shm_ptr->cmd_end_eff_joint_positions_[j] = end_eff_shm_ptr->cur_end_eff_joint_positions_[j]+0.001;
            end_eff_shm_ptr->cmd_end_eff_joint_velocities_[j] = 0;
            end_eff_shm_ptr->cmd_end_eff_joint_efforts_[j] = 0;
        }
        sem::semaphore_v(end_eff_sem_id);
        loop_rate.sleep();
        i++;
        if (i>1000)
        {
            std::cout << "end-effector execute successfully." << std::endl;
            break;
        }
    }
#endif

    return 0;
}