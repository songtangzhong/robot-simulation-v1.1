#include <shared_memory/shared_memory.h>
#include <rclcpp/rclcpp.hpp>
#include <iostream>

namespace shm
{
// Create shared memory.
/*
    key (input): key value of shared memory.
    shm_ptr (output): shared memory pointer.
    return: 
        success: shared memory ID.
        fail: SHM_STATE_NO
*/
template <class shm_type>
int create_shm(key_t key, shm_type ** shm_ptr)
{
	// link pointer to shared memory.
    void *shm_ln_ptr = NULL;

    // pointer to shared memory.
    shm_type *shm_ptr_;

    // ID to shared memory.
    int shm_id;

    // Create shared memory.
    shm_id = shmget(key, sizeof(shm_type), 0666|IPC_CREAT);
    if (shm_id == -1)
    {
        std::cout << "Failed to create shared memory." << std::endl;
		return SHM_STATE_NO;
    }

    // Create the link address of shared memory.
    shm_ln_ptr = shmat(shm_id, 0, 0);
    if (shm_ln_ptr == (void*)-1)
    {
        std::cout << "Failed to create link address of shared memory." << std::endl;
		return SHM_STATE_NO;
    }

    // Link the address of shared memory to current address.
    *shm_ptr = shm_ptr_ = (shm_type*)shm_ln_ptr;

	return shm_id;
}

// Release shared memory.
/*
    shm_id (input): shared memory ID. 
    shm_ptr (input): shared memory pointer.
    return:
        success: SHM_STATE_OK
        fail: SHM_STATE_NO
*/
template <class shm_type>
int release_shm(int shm_id, shm_type ** shm_ptr)
{
	// Release shared memory.
    if (shmdt(*shm_ptr) == -1)
    {
        std::cout << "Failed to seperate shared memory." << std::endl;
		return SHM_STATE_NO;
    }
    if (shmctl(shm_id, IPC_RMID, 0) == -1)
    {
        std::cout << "Failed to release shared memory." << std::endl;
		return SHM_STATE_NO;
    }

	return SHM_STATE_OK;
}

// Realize function template.
template int create_shm<ArmShm>(key_t key, ArmShm ** shm_ptr);
template int release_shm<ArmShm>(int shm_id, ArmShm ** shm_ptr);

#if END_EFFECTOR_TRUE
// Realize function template.
template int create_shm<EndEffShm>(key_t key, EndEffShm ** shm_ptr);
template int release_shm<EndEffShm>(int shm_id, EndEffShm ** shm_ptr);
#endif

} // end namespace shm
