#ifndef ROBOT_GAZEBO_PLUGIN_H_
#define ROBOT_GAZEBO_PLUGIN_H_

#include <gazebo/physics/Model.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/Joint.hh>
#include <robot_info/robot_info.h>
#include <shared_memory/shared_memory.h>
#include <shared_memory/semaphore.h>
#include <iostream>

namespace robot_gazebo_plugin
{
class ControlPlugin : public gazebo::ModelPlugin
{
public:
    ControlPlugin();
    ~ControlPlugin();

    // Overloaded Gazebo entry point
    void Load(gazebo::physics::ModelPtr parent, sdf::ElementPtr sdf);

    // Called by the world update start event
    void Update();

private:
    // Pointer to the model
    gazebo::physics::ModelPtr parent_model_;

    // Pointer to the arm joints
    std::vector<gazebo::physics::JointPtr> arm_joints_;

#if END_EFFECTOR_TRUE
    // Pointer to the end-effector joints
    std::vector<gazebo::physics::JointPtr> end_eff_joints_;
#endif

    // Pointer to the update event connection
    gazebo::event::ConnectionPtr update_connection_;

    // Store the information of a robot.
    std::shared_ptr<robot_info::RobotInfo> robot = std::make_shared<robot_info::RobotInfo>();

    // pointer to arm shared memory.
    shm::ArmShm *arm_shm_ptr;
    // ID to arm shared memory.
    int arm_shm_id;
    // ID to arm semaphore.
    int arm_sem_id;

#if END_EFFECTOR_TRUE
    // pointer to end-effector shared memory.
    shm::EndEffShm *end_eff_shm_ptr;
    // ID to end-effector shared memory.
    int end_eff_shm_id;
    // ID to end-effector semaphore.
    int end_eff_sem_id;
#endif
}; // end class ControlPlugin

} // end namespace robot_gazebo_plugin

#endif
