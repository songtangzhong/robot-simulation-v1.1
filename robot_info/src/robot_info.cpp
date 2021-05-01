#include <robot_info/robot_info.h>

namespace robot_info
{
RobotInfo::RobotInfo()
{
    // Robot arm degree of freedom.
    arm_dof_ = 7;

    // Robot arm joint names.
    arm_joint_names_.resize(arm_dof_);
    arm_joint_names_ = {"panda_joint1","panda_joint2","panda_joint3","panda_joint4",
                        "panda_joint5","panda_joint6","panda_joint7"};
    
    // Resize arm joint state variables.
    cur_arm_joint_positions_.resize(arm_dof_);
    cur_arm_joint_velocities_.resize(arm_dof_);
    cur_arm_joint_efforts_.resize(arm_dof_);

    // Resize arm joint command variables.
    cmd_arm_joint_positions_.resize(arm_dof_);
    cmd_arm_joint_velocities_.resize(arm_dof_);
    cmd_arm_joint_efforts_.resize(arm_dof_);

    // Resize arm control mode variable.
    arm_control_modes_.resize(arm_dof_);

    // Set the initial values of arm information.
    for (unsigned int j=0; j<arm_dof_; j++)
    {
        cur_arm_joint_positions_[j] = cmd_arm_joint_positions_[j] = 0;
        cur_arm_joint_velocities_[j] = cmd_arm_joint_velocities_[j] = 0;
        cur_arm_joint_efforts_[j] = cmd_arm_joint_efforts_[j] = 0;

        arm_control_modes_[j] = position_mode_; // default: position mode
    }
    cur_arm_joint_positions_[3] = cmd_arm_joint_positions_[3] =  -1.5;
    cur_arm_joint_positions_[3] = cmd_arm_joint_positions_[5] = 1.88;

    // key value of shared memory 
    arm_shm_key_ = 1234;
    // key value of semaphore
    arm_sem_key_ = 1235;

#if END_EFFECTOR_TRUE
    // End-effector degree of freedom.
    end_eff_dof_ = 7;

    // End-effector joint names.
    end_eff_joint_names_.resize(end_eff_dof_);
    end_eff_joint_names_ = {"panda_finger_joint1","panda_finger_joint2"};
    
    // Resize end-effector joint state variables.
    cur_end_eff_joint_positions_.resize(end_eff_dof_);
    cur_end_eff_joint_velocities_.resize(end_eff_dof_);
    cur_end_eff_joint_efforts_.resize(end_eff_dof_);

    // Resize end-effector joint command variables.
    cmd_end_eff_joint_positions_.resize(end_eff_dof_);
    cmd_end_eff_joint_velocities_.resize(end_eff_dof_);
    cmd_end_eff_joint_efforts_.resize(end_eff_dof_);

    // Resize end-effector control mode variable.
    end_eff_control_modes_.resize(end_eff_dof_);

    // Set the initial values of end-effector information.
    for (unsigned int j=0; j<end_eff_dof_; j++)
    {
        cur_end_eff_joint_positions_[j] = cmd_end_eff_joint_positions_[j] = 0;
        cur_end_eff_joint_velocities_[j] = cmd_end_eff_joint_velocities_[j] = 0;
        cur_end_eff_joint_efforts_[j] = cmd_end_eff_joint_efforts_[j] = 0;

        end_eff_control_modes_[j] = position_mode_; // default: position mode
    }
    
    // key value of shared memory 
    arm_shm_key_ = 12345;
    // key value of semaphore
    arm_sem_key_ = 12346;
#endif
}

RobotInfo::~RobotInfo(){}

} // end namespace robot_info
