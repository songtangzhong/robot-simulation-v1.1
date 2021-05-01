#include <shared_memory/semaphore.h>
#include <robot_info/robot_info.h>
#include <rclcpp/rclcpp.hpp>
#include <iostream>

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    if (argc != 3)
    {
        std::cout << "Usage:" << std::endl;
        std::cout << "ros2 run shared_memory test_block_process arg1 arg2" << std::endl;
        std::cout << "the value of arg1: p" << std::endl;
        std::cout << "the value of arg2: arm, end_effector" << std::endl;

        return 0;
    }

    std::shared_ptr<robot_info::RobotInfo> robot = std::make_shared<robot_info::RobotInfo>();

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

    if ((std::string(argv[1]) == "p") && (std::string(argv[2]) == "arm"))
    {
        while (sem::semaphore_p(arm_sem_id) != SEM_STATE_OK)
        {
            std::cout << "Get a arm semaphore failed." << std::endl;
        }
        std::cout << "Get a arm semaphore successfully." << std::endl;
    }
#if END_EFFECTOR_TRUE
    else if ((std::string(argv[1]) == "p") && (std::string(argv[2]) == "end_effector"))
    {
        while (sem::semaphore_p(end_eff_sem_id) != SEM_STATE_OK)
        {
            std::cout << "Get a end-effector semaphore failed." << std::endl;
        }
        std::cout << "Get a end-effector semaphore successfully." << std::endl;
    }
#endif
    else
    {
        std::cout << "error!!!" << std::endl;
    }

    while (rclcpp::ok());

    return 0;
}