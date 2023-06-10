/** @file syscall_thread.h
 *
 *  @brief  Custom syscalls to support thread library.
 *
 *  @date   October 24, 2019
 *
 *  @author Benjamin Huang <zemingbh@andrew.cmu.edu>
 */

#ifndef _SYSCALL_MUTEX_H_
#define _SYSCALL_MUTEX_H_

#include <unistd.h>

/**
 * @brief      The struct for a mutex.
 */
typedef struct {
  volatile uint32_t locked_by; /**<thread blocking mutex>*/
  volatile uint32_t prio_ceil; /**< priority ceiling>*/
  volatile uint32_t blocked_threads; /**< bitvector of blocked threads>*/
} kmutex_t;

/**
 * @brief      Used to create a mutex object. The mutex resides in kernel
 *             space. The user receives a handle to it. With memory
 *             protection, the user cannot modify it. However, the pointer
 *             can still be passed around and used with lock and unlock.
 *
 * @param      max_prio  The maximum priority of a thread which could use
 *                       this mutex (the lowest number, following convention).
 *
 * @return     A pointer to the mutex. NULL if max_mutexes would be exceeded.
 */
kmutex_t *sys_mutex_init( uint32_t max_prio );

/**
 * @brief      Lock a mutex
 *
 *             This function will not return until the current thread has
 *             obtained the mutex.
 *
 * @param[in]  mutex  The mutex to act on.
 */
void sys_mutex_lock( kmutex_t *mutex );

/**
 * @brief      Unlock a mutex
 *
 * @param[in]  mutex  The mutex to act on.
 */
void sys_mutex_unlock( kmutex_t *mutex );

/**
 * @brief kernel initializes mutexes to default values
 */
void kernel_initialize_mutexes();

#endif /* _SYSCALL_MUTEX_H_ */
