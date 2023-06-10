/** @file syscall_thread.h
 *
 *  @brief  Custom syscalls to support thread library.
 *
 *  @date   March 27, 2019
 *
 *  @author Ronit Banerjee <ronitb@andrew.cmu.edu>
 */

#ifndef _SYSCALL_THREAD_H_
#define _SYSCALL_THREAD_H_

#include <unistd.h>

/**
 * @enum protection_mode
 *
 * @brief      Enums for protection mode, PER_THREAD and KERNEL_ONLY.
 */
typedef enum { PER_THREAD = 1, KERNEL_ONLY = 0 } protection_mode;

/**
 * @enum thread_status_t
 *
 * @brief      Enums for thread status
 */
typedef enum{ WAITING = 0, RUNNABLE = 1, RUNNING = 2, DEAD = 3} thread_status_t;


/**
 * @struct interrupt_stack_frame
 *
 * @brief  Stack frame upon exception.
 */
typedef struct { 
  uint32_t r4; /** @brief register 4 */
  uint32_t r5; /** @brief register 5 */
  uint32_t r6; /** @brief register 6 */
  uint32_t r7; /** @brief register 7 */
  uint32_t r8; /** @brief register 8 */
  uint32_t r9; /** @brief register 9 */
  uint32_t r10; /** @brief register 10*/
  uint32_t r11; /** @brief register 11*/
  uint32_t rtn_code; /** @brief cpu return code */
  uint32_t psp; /** @brief psp */
  uint32_t r0;   /** @brief Register value for r0 */
  uint32_t r1;   /** @brief Register value for r1 */
  uint32_t r2;   /** @brief Register value for r2 */
  uint32_t r3;   /** @brief Register value for r3 */
  uint32_t r12;  /** @brief Register value for r12 */
  uint32_t lr;   /** @brief Register value for lr*/
  uint32_t pc;   /** @brief Register value for pc */
  uint32_t xPSR; /** @brief Register value for xPSR */
} interrupt_stack_frame;

/**
 * @struct user_stack_frame
 *
 * @brief  ONLY used when creating a thread
 */
typedef struct { 
  uint32_t r0;   /** @brief Register value for r0 */
  uint32_t r1;   /** @brief Register value for r1 */
  uint32_t r2;   /** @brief Register value for r2 */
  uint32_t r3;   /** @brief Register value for r3 */
  uint32_t r12;  /** @brief Register value for r12 */
  uint32_t lr;   /** @brief Register value for lr*/
  uint32_t pc;   /** @brief Register value for pc */
  uint32_t xPSR; /** @brief Register value for xPSR */
} user_stack_frame;


/*
 * 
 * @brief TCB data structure
 */

typedef struct {
    uint32_t C; /** @brief worst case computation time*/
    uint32_t T; /**@brief period*/
    uint32_t msp; /**@brief address of msp*/
    uint32_t msp_static_start; /**@brief address of starting address of msp*/
    uint32_t psp_static_start; /**@brief address of starting address of psp*/
    uint32_t static_priority; /**@brief static priority*/
    thread_status_t stat; /**@brief status of thread*/
    int svc_stat; /**@brief svc status*/
    protection_mode p_mode; /**@brief protection mode*/
    uint32_t fn; /**@brief function that thread will run*/
    uint32_t end_of_period; /**@brief end of current period*/
    uint32_t ticks_computation_time; /**@brief ticks used in current period*/
    uint32_t ticks_ct_from_beginning; /**@brief ticks since thread started running*/
    uint32_t dynamic_priority; /**@brief dynamic priority*/
} TCB;

/**
 * @brief      The SysTick interrupt handler.
 */
void systick_c_handler( void );

/**
 * @brief      The PendSV interrupt handler.
 */
void *pendsv_c_handler( interrupt_stack_frame* );

/**
 * @brief      Initialize the thread library
 *
 *             A user program must call this initializer before attempting to
 *             create any threads or starting the scheduler.
 *
 * @param[in]  max_threads        Maximum number of threads that will be
 *                                created.
 * @param[in]  stack_size         Declares the size in words of all user and
 *                                kernel stacks created.
 * @param[in]  idle_fn            Pointer to a thread function to run when no
 *                                other threads are runnable. If NULL is
 *                                is supplied, the kernel will provide its
 *                                own idle function that will sleep.
 * @param[in]  memory_protection  Enum for memory protection, either
 *                                PER_THREAD or KERNEL_ONLY
 * @param[in]  max_mutexes        Maximum number of mutexes that will be
 *                                created.
 *
 * @return     0 on success or -1 on failure
 */
int sys_thread_init(
  uint32_t        max_threads,
  uint32_t        stack_size,
  void           *idle_fn,
  protection_mode memory_protection,
  uint32_t        max_mutexes
);

/**
 * @brief      Create a new thread running the given function. The thread will
 *             not be created if the UB test fails, and in that case this function
 *             will return an error.
 *
 * @param[in]  fn     Pointer to the function to run in the new thread.
 * @param[in]  prio   Priority of this thread. Lower number are higher
 *                    priority.
 * @param[in]  C      Real time execution time (scheduler ticks).
 * @param[in]  T      Real time task period (scheduler ticks).
 * @param[in]  vargp  Argument for thread function (usually a pointer).
 *
 * @return     0 on success or -1 on failure
 */
int sys_thread_create( void *fn, uint32_t prio, uint32_t C, uint32_t T, void *vargp );

/**
 * @brief      Allow the kernel to start running the thread set.
 *
 *             This function should enable SysTick and thus enable your
 *             scheduler. It will not return immediately unless there is an error.
 *			   It may eventually return successfully if all thread functions are
 *   		   completed or killed.
 *
 * @param[in]  frequency  Frequency (Hz) of context swaps.
 *
 * @return     0 on success or -1 on failure
 */
int sys_scheduler_start( uint32_t frequency );

/**
 * @brief      Get the current time.
 *
 * @return     The time in ticks.
 */
uint32_t sys_get_time( void );

/**
 * @brief      Get the effective priority of the current running thread
 *
 * @return     The thread's effective priority
 */
uint32_t sys_get_priority( void );

/**
 * @brief      Gets the total elapsed time for the thread (since its first
 *             ever period).
 *
 * @return     The time in ticks.
 */
uint32_t sys_thread_time( void );

/**
 * @brief      Waits efficiently by descheduling thread.
 */
void sys_wait_until_next_period( void );

/**
* @brief      Kills current running thread. Aborts program if current thread is
*             main thread or the idle thread or if current thread exited
*             while holding a mutex.
*
* @return     Does not return.
*/
void sys_thread_kill( void );

#endif /* _SYSCALL_THREAD_H_ */
