/*
    This file is used to test gazebo control plugin according to reading operation.
*/

#include <robot_info/robot_info.h>
#include <shared_memory/shared_memory.h>
#include <rclcpp/rclcpp.hpp>
#include <iostream>

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    // Store the information of robot.
    std::shared_ptr<robot_info::RobotInfo> robot = std::make_shared<robot_info::RobotInfo>();

    // pointer to shared memory.
    shm::Shm *shm_ptr;
    // ID to shared memory.
    int shm_id;

    shm_id = shm::create_shm(&shm_ptr);
    if (shm_id != PROCESS_STATE_NO)
    {
        std::cout << "Create shared memory successfully." << std::endl;
        std::cout << "shm_id: " << shm_id << std::endl;
    }
    else
    {
        std::cout << "Create shared memory failed." << std::endl;
        return 0;
    }

    // ID to semaphore.
    int sem_id;

    sem_id = sem::create_semaphore();
    if (sem_id != PROCESS_STATE_NO)
    {
        std::cout << "Create semaphore successfully." << std::endl;
        std::cout << "sem_id: " << sem_id << std::endl;
    }
    else
    {
        std::cout << "Create semaphore failed." << std::endl;
        return 0;
    }

    rclcpp::WallRate loop_rate(1000);
    while (rclcpp::ok())
    {
        sem::semaphore_p(sem_id);
        for (unsigned int j=0; j< robot->arm_dof_; j++)
        { 
            std::cout << "cur_joint_positions_[" << j << "]: " << shm_ptr->cur_joint_positions_[j] << std::endl;
            std::cout << "cur_joint_velocities_[" << j << "]: " << shm_ptr->cur_joint_velocities_[j] << std::endl;
            std::cout << "cur_joint_efforts_[" << j << "]: " << shm_ptr->cur_joint_efforts_[j] << std::endl;
        }
        sem::semaphore_v(sem_id);
        loop_rate.sleep();
    }

    return 0;
}