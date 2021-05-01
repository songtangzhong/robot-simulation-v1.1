#include <shared_memory/semaphore.h>
#include <rclcpp/rclcpp.hpp>
#include <iostream>

namespace sem
{
// Create a semaphore.
/*
    key (input): key value of semaphore.
    return:
        success: semaphore ID
        fail: SEM_STATE_NO
*/
int create_semaphore(key_t key)
{
	// ID of semaphore.
    int sem_id;

	// Create a semaphore.
    sem_id = semget(key, 1, 0666 | IPC_CREAT);
    if (sem_id == -1)
    {
        std::cout << "Failed to create the semaphore." << std::endl;
		return SEM_STATE_NO;
    }

	sem_un sem;
    sem.val = 1;
    if  (semctl(sem_id, 0, SETVAL, sem) == -1)
    {
        std::cout << "Failed to set the value of semaphore." << std::endl;
        return SEM_STATE_NO;
    }

    return sem_id;
}

// Delete a semaphore.
/*
    sem_id (input): semaphore ID
    return:
        success: SEM_STATE_OK
        fail: SEM_STATE_NO
*/
int delete_semaphore(int sem_id)
{
	sem_un sem;

    if (semctl(sem_id, 0, IPC_RMID, sem) == -1)
    {
        std::cout << "Failed to delete the semaphore." << std::endl;
        return SEM_STATE_NO;
    }

    return SEM_STATE_OK;
}

// Get a semaphore.
/*
    sem_id (input): semaphore ID
    return:
        success: SEM_STATE_OK
        fail: SEM_STATE_NO
*/
int semaphore_p(int sem_id)
{
	struct sembuf sem_b;
	sem_b.sem_num = 0;
	sem_b.sem_op = -1;
	sem_b.sem_flg = SEM_UNDO;
	if (semop(sem_id, &sem_b, 1) == -1)
	{
		std::cout << "Failed to get the semaphore." << std::endl;
		return SEM_STATE_NO;
	}

    return SEM_STATE_OK;
}

// Release a semaphore.
/*
    sem_id (input): semaphore ID
    return:
        success: SEM_STATE_OK
        fail: SEM_STATE_NO
*/
int semaphore_v(int sem_id)
{
	struct sembuf sem_b;
	sem_b.sem_num = 0;
	sem_b.sem_op = 1;
	sem_b.sem_flg = SEM_UNDO;
	if (semop(sem_id, &sem_b, 1) == -1)
	{
		std::cout << "Failed to release the semaphore." << std::endl;
		return SEM_STATE_NO;
	}

	return SEM_STATE_OK;
}

} // end namespace sem
