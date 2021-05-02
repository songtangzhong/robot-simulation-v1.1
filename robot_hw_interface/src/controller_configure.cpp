#include <chrono>
#include <memory>
#include <string>
#include <controller_manager_msgs/srv/load_start_controller.hpp>
#include <controller_manager_msgs/srv/load_configure_controller.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <rclcpp/rclcpp.hpp>
#include <robot_hw_interface/controller_configure.h>

using namespace std::chrono_literals;

namespace controller_configure
{
ControllerConfigure::ControllerConfigure(const std::string & node_name)
{
    nh_ = std::make_shared<rclcpp::Node>(node_name);

    load_start_controller_cli_ = 
    nh_->create_client<controller_manager_msgs::srv::LoadStartController>("/controller_manager/load_and_start_controller");

    load_configure_controller_cli_ = 
    nh_->create_client<controller_manager_msgs::srv::LoadConfigureController>("/controller_manager/load_and_configure_controller");

    switch_controller_cli_ = 
    nh_->create_client<controller_manager_msgs::srv::SwitchController>("/controller_manager/switch_controller");

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
}

ControllerConfigure::~ControllerConfigure(){}

void ControllerConfigure::load_start_controller(const std::string & controller_name)
{
    auto request = std::make_shared<controller_manager_msgs::srv::LoadStartController::Request>();
    request->name = controller_name;
    while (!load_start_controller_cli_->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service [/controller_manager/load_and_start_controller] not available, waiting again...");
    }

    auto result = load_start_controller_cli_->async_send_request(request);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(nh_, result) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "load and start %s successfully.", controller_name.c_str());
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to load and start %s.", controller_name.c_str());
    }
}

void ControllerConfigure::load_configure_controller(const std::string & controller_name)
{
    auto request = std::make_shared<controller_manager_msgs::srv::LoadConfigureController::Request>();
    request->name = controller_name;
    while (!load_configure_controller_cli_->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service [/controller_manager/load_and_configure_controller] not available, waiting again...");
    }

    auto result = load_configure_controller_cli_->async_send_request(request);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(nh_, result) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "load and configure %s successfully.", controller_name.c_str());
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to load and start %s.", controller_name.c_str());
    }
}

void ControllerConfigure::switch_controller(const std::string & start_controller, const std::string & stop_controller)
{
    std::vector<std::string> start_controller_ = {start_controller};
    std::vector<std::string> stop_controller_ = {stop_controller};
    auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
    request->start_controllers = start_controller_;
    request->stop_controllers = stop_controller_;
    request->strictness = request->BEST_EFFORT;
    request->start_asap = false;
    request->timeout = rclcpp::Duration(static_cast<rcl_duration_value_t>(0.0));
    while (!switch_controller_cli_->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service [/controller_manager/switch_controller] not available, waiting again...");
    }

    auto result = switch_controller_cli_->async_send_request(request);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(nh_, result) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        // Get arm semaphore.
        sem::semaphore_p(arm_sem_id);
        for (unsigned int j=0; j< robot->arm_dof_; j++)
        {
            if (start_controller == "position_controllers")
            {
                // Set current arm joint positions to commands by ros2 controller manager,
                // not by shared memory.
                arm_shm_ptr->arm_control_modes_[j] = robot->position_mode_;
            }
            else if (start_controller == "velocity_controllers")
            {
                // Set current arm joint velocities (zeros) to commands by ros2 controller manager,
                // not by shared memory.
                arm_shm_ptr->arm_control_modes_[j] = robot->velocity_mode_;
            }
            else if (start_controller == "effort_controllers")
            {
                // Set current arm joint efforts (zeros) to commands by ros2 controller manager,
                // not by shared memory.
                arm_shm_ptr->arm_control_modes_[j] = robot->effort_mode_;
            }
        }
        // Release arm semaphore.
        sem::semaphore_v(arm_sem_id);

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "switch %s to %s successfully.", stop_controller.c_str(), start_controller.c_str());
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to switch %s to %s.", stop_controller.c_str(), start_controller.c_str());
    }
}

} // end namespace controller_configure
