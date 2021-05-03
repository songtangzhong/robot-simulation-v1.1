#include <robot_interface_fun/robot_interface_fun.h>
#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <thread>

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    std::shared_ptr<rclcpp::Executor> executor =
    std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

    std::shared_ptr<robot_fun::RobotFun> robot_fun = std::make_shared<robot_fun::RobotFun>("test_robot_interface_fun");
    std::shared_ptr<robot_fun::RobotFun> robot_fun_1 = std::make_shared<robot_fun::RobotFun>("test_robot_interface_fun_1");

    std::thread thread_1([robot_fun]() {});
    std::thread thread_2([robot_fun_1]() {
        rclcpp::WallRate loop_rate(1000);
        double cur_arm_positions[ARM_DOF];
        std::this_thread::sleep_for(std::chrono::seconds(1));
        while (rclcpp::ok())
        {
            robot_fun_1->get_arm_joint_positions(cur_arm_positions);
            
            for (unsigned int j=0; j< ARM_DOF; j++)
            {
                std::cout << "cur_positions[" << j << "]: " << cur_arm_positions[j] << std::endl;
            }
            std::cout << "*********" << std::endl;
            loop_rate.sleep();
        }
    });

    executor->add_node(robot_fun->nh_);
    executor->add_node(robot_fun_1->nh_);
    executor->spin();
    thread_1.join();
    thread_2.join();
    rclcpp::shutdown();

    return 0;
}
