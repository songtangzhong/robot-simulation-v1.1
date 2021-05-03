#include <memory>
#include <robot_interface_fun/robot_interface_fun.h>
#include <iostream>

using std::placeholders::_1;

namespace robot_fun
{
RobotFun::RobotFun(const std::string &nh_name)
{
    nh_ = std::make_shared<rclcpp::Node>(nh_name);

    robot_state_sub_ = nh_->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 100, std::bind(&RobotFun::callback_robot_state_sub_, this, _1));

    // Create arm shared memory.
    arm_shm_id = shm::create_shm(robot->arm_state_shm_key_, &arm_shm_ptr);
    if (arm_shm_id != SHM_STATE_NO)
    {
        std::cout << "Create arm shared memory successfully." << std::endl;
    }
    else
    {
        std::cout << "Create arm shared memory failed." << std::endl;
    }

    // Create arm semaphore.
    arm_sem_id = sem::create_semaphore(robot->arm_state_sem_key_);
    if (arm_sem_id != SEM_STATE_NO)
    {
        std::cout << "Create arm semaphore successfully." << std::endl;
    }
    else
    {
        std::cout << "Create arm semaphore failed." << std::endl;
    }
}

RobotFun::~RobotFun()
{
    // Delete arm semaphore.
    if (sem::delete_semaphore(arm_sem_id) == SEM_STATE_OK)
    {
        std::cout << "Delete arm semaphore successfully." << std::endl;
    }
    else
    {
        std::cout << "Delete arm semaphore failed." << std::endl;
    }

    // Release arm shared memory.
    if (shm::release_shm(arm_shm_id, &arm_shm_ptr) == SHM_STATE_OK)
    {
        std::cout << "Release arm shared memory successfully." << std::endl;
    }
    else
    {
        std::cout << "Release arm shared memory failed." << std::endl;
    }
    
    std::cout << "end robot interface fun." << std::endl;
}

void RobotFun::callback_robot_state_sub_(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    sem::semaphore_p(arm_sem_id);
    for (unsigned int j=0; j< msg->name.size(); j++)
    {
        for (unsigned int i=0; i< robot->arm_dof_; i++)
        {
            if (msg->name[j] == robot->arm_joint_names_[i])
            {
                arm_shm_ptr->cur_arm_joint_positions_[i] = msg->position[j];
                arm_shm_ptr->cur_arm_joint_velocities_[i] = msg->velocity[j];
                arm_shm_ptr->cur_arm_joint_efforts_[i] = msg->effort[j];
            }
        }
    }
    sem::semaphore_v(arm_sem_id);
}

void RobotFun::get_arm_joint_positions(double * positions)
{
    sem::semaphore_p(arm_sem_id);
    for (unsigned int j=0; j< robot->arm_dof_; j++)
    {
        *(positions+j) = arm_shm_ptr->cur_arm_joint_positions_[j];
    }
    sem::semaphore_v(arm_sem_id);
}

} // end namespace robot_fun
