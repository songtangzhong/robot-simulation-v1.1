#ifndef SEMAPHORE_H_
#define SEMAPHORE_H_

#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/sem.h>
#include <iostream>

// Define state in semaphore operation.
#define SEM_STATE_OK  1
#define SEM_STATE_NO -1

namespace sem
{
// parameter union
typedef union 
{
	int val;
	struct semid_ds *buf;
	unsigned short *arry;
}sem_un;

// Create a semaphore.
/*
    key (input): key value of semaphore.
    return:
        success: semaphore ID
        fail: SEM_STATE_NO
*/
int create_semaphore(key_t key);

// Delete a semaphore.
/*
    sem_id (input): semaphore ID
    return:
        success: SEM_STATE_OK
        fail: SEM_STATE_NO
*/
int delete_semaphore(int sem_id);

// Get a semaphore.
/*
    sem_id (input): semaphore ID
    return:
        success: SEM_STATE_OK
        fail: SEM_STATE_NO
*/
int semaphore_p(int sem_id);

// Release a semaphore.
/*
    sem_id (input): semaphore ID
    return:
        success: SEM_STATE_OK
        fail: SEM_STATE_NO
*/
int semaphore_v(int sem_id);

} // end namespace sem

#endif