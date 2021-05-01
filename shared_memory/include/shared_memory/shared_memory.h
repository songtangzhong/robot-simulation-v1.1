#ifndef SHARED_MEMORY_H_
#define SHARED_MEMORY_H_

#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/shm.h>
#include <robot_info/robot_info.h>

// Define state in shared memory operation.
#define SHM_STATE_OK  1
#define SHM_STATE_NO -1

namespace shm
{
// Robot arm shared memory.
typedef struct
{
    // Store current arm joint states.
    double cur_arm_joint_positions_[ARM_DOF];
    double cur_arm_joint_velocities_[ARM_DOF];
    double cur_arm_joint_efforts_[ARM_DOF];

    // Store current arm joint commands.
    double cmd_arm_joint_positions_[ARM_DOF];
    double cmd_arm_joint_velocities_[ARM_DOF];
    double cmd_arm_joint_efforts_[ARM_DOF];

    // Store arm joint control modes.
    unsigned int arm_control_modes_[ARM_DOF];
}ArmShm;

#if END_EFFECTOR_TRUE
// End-effector shared memory.
typedef struct
{
    // Store current End-effector joint states.
    double cur_end_eff_joint_positions_[END_EFF_DOF];
    double cur_end_eff_joint_velocities_[END_EFF_DOF];
    double cur_end_eff_joint_efforts_[END_EFF_DOF];

    // Store current End-effector joint commands.
    double cmd_end_eff_joint_positions_[END_EFF_DOF];
    double cmd_end_eff_joint_velocities_[END_EFF_DOF];
    double cmd_end_eff_joint_efforts_[END_EFF_DOF];

    // Store End-effector joint control modes.
    unsigned int end_eff_control_modes_[END_EFF_DOF];
}EndEffShm;
#endif

// Create shared memory.
/*
    key (input): key value of shared memory.
    shm_ptr (output): shared memory pointer.
    return: 
        success: shared memory ID.
        fail: SHM_STATE_NO
*/
template <class shm_type>
int create_shm(key_t key, shm_type ** shm_ptr);

// Release shared memory.
/*
    shm_id (input): shared memory ID. 
    shm_ptr (input): shared memory pointer.
    return:
        success: SHM_STATE_OK
        fail: SHM_STATE_NO
*/
template <class shm_type>
int release_shm(int shm_id, shm_type ** shm_ptr);

} // end namespace shm

#endif
