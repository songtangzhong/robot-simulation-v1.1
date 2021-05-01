#include <shared_memory/semaphore.h>
#include <robot_info/robot_info.h>
#include <rclcpp/rclcpp.hpp>
#include <iostream>

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

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

    if (sem::semaphore_p(arm_sem_id) == SEM_STATE_OK)
    {
        std::cout << "Get a arm semaphore successfully." << std::endl;
    }
    else
    {
        std::cout << "Get a arm semaphore failed." << std::endl;
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

    if (sem::semaphore_p(end_eff_sem_id) == SEM_STATE_OK)
    {
        std::cout << "Get a end-effector semaphore successfully." << std::endl;
    }
    else
    {
        std::cout << "Get a end-effector semaphore failed." << std::endl;
        return 0;
    }
#endif

    std::this_thread::sleep_for(std::chrono::seconds(5));

    if (sem::semaphore_v(arm_sem_id) == SEM_STATE_OK)
    {
        std::cout << "Release a arm semaphore successfully." << std::endl;
    }
    else
    {
        std::cout << "Release a arm semaphore failed." << std::endl;
        return 0;
    }

    if (sem::delete_semaphore(arm_sem_id) == SEM_STATE_OK)
    {
        std::cout << "Delete arm semaphore successfully." << std::endl;
    }
    else
    {
        std::cout << "Delete arm semaphore failed." << std::endl;
        return 0;
    }

#if END_EFFECTOR_TRUE
    if (sem::semaphore_v(end_eff_sem_id) == SEM_STATE_OK)
    {
        std::cout << "Release a end-effector semaphore successfully." << std::endl;
    }
    else
    {
        std::cout << "Release a end-effector semaphore failed." << std::endl;
        return 0;
    }

    if (sem::delete_semaphore(end_eff_sem_id) == SEM_STATE_OK)
    {
        std::cout << "Delete end-effector semaphore successfully." << std::endl;
    }
    else
    {
        std::cout << "Delete end-effector semaphore failed." << std::endl;
        return 0;
    }
#endif

    return 0;
}