#ifndef ROBOT_INTERFACE_FUN_H_
#define ROBOT_INTERFACE_FUN_H_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <robot_info/robot_info.h>
#include <shared_memory/shared_memory.h>
#include <shared_memory/semaphore.h>

namespace robot_fun
{
class RobotFun
{
public:
    std::shared_ptr<rclcpp::Node> nh_;

    RobotFun(const std::string &nh_name);
    ~RobotFun();

    void get_arm_joint_positions(double * positions);

private:
    std::shared_ptr<robot_info::RobotInfo> robot = std::make_shared<robot_info::RobotInfo>();

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr robot_state_sub_;

    void callback_robot_state_sub_(const sensor_msgs::msg::JointState::SharedPtr msg);

    // pointer to arm shared memory.
    shm::ArmShm *arm_shm_ptr;
    // ID to arm shared memory.
    int arm_shm_id;
    // ID to arm semaphore.
    int arm_sem_id;

}; // end class RobotFun

} // end namespace robot_fun

#endif