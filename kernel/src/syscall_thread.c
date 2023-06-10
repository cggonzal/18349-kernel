/** @file   syscall_thread.c
 *
 *  @brief  
 *
 *  @date   
 *
 *  @author 
 */

#include <stdint.h>
#include "syscall_thread.h"
#include "syscall_mutex.h"
#include "arm.h"
#include "timer.h"
#include "mpu.h"
#include "printk.h"
#include "syscall.h"

/** @brief      Initial XPSR value, all 0s except thumb bit. */
#define XPSR_INIT 0x1000000
/** @brief max possible number of threads */
#define MAX_POSSIBLE_THREADS 16
/** @brief max possible number of user threads */
#define MAX_USER_THREADS 14
/** @brief maximum bits that can be shifted*/
#define MAX_BIT_SHIFT (uint32_t)31
/** @brief maximum number of mutexes supported*/
#define MUTEX_THEORETICAL_MAX 32
/** @brief maximum size of 32kb for all thread user and kernel stacks*/
#define TOTAL_MAX_STACK_SIZES 32 * (1024) 
/** @brief index of main thread */
#define DEFAULT_THREAD_INDEX 15
/** @brief index of idle thread*/
#define IDLE_THREAD_INDEX 14
/** @brief minimum thread stack size*/
#define MIN_STACK_SIZE 1024 // min stack size of 1kb
/** @brief user return code*/
#define RETURN_CODE_USER 0xFFFFFFFD
/** @brief inactive value of svc*/
#define SVC_INACTIVE 0
/** @brief unintiailized mutex lock value*/
#define UNINITIALIZED_MUTEX_LOCK 40
/** @brief uninitialized priority ceiling value*/
#define UNINITIALIZED_PRIO_CEIL 20

/**
 * @brief      Heap high and low pointers.
 */
//@{ 
extern char
  __thread_u_stacks_low,
  __thread_u_stacks_top,
  __thread_k_stacks_low,
  __thread_k_stacks_top;
//@}

/** @brief user thread kill function*/
extern int thread_kill;

/** @brief idlle default function*/
extern void* idle_default;

/** total threads created */
uint32_t TOTAL_THREADS_CREATED = 0;
/** max number of mutexes */
uint32_t MAX_NUM_MUTEXES = 32;

/**
 * @brief      Precalculated values for UB test.
 */
float ub_table[] = {
  0.000, 1.000, .8284, .7798, .7568,
  .7435, .7348, .7286, .7241, .7205,
  .7177, .7155, .7136, .7119, .7106,
  .7094, .7083, .7075, .7066, .7059,
  .7052, .7047, .7042, .7037, .7033,
  .7028, .7025, .7021, .7018, .7015,
  .7012, .7009
};

/** initialize globals for waiting and runnable pools. */
uint32_t NUM_USER_THREADS = 0;
TCB TCB_THREADS[MAX_POSSIBLE_THREADS];

/** index of running thread */
uint32_t RUNNING_THREAD; 

/** total ticks since beginning of time, see 1st paragraph section 3.3 */
uint32_t TICK_COUNTER;

/** total usability used by all threads*/
float TOTAL_USABILITY = 0.0;

/** stack size of each thread*/
uint32_t PER_THREAD_STACK_SIZE;

/** array holding all mutexes */
kmutex_t MUTEXES[MUTEX_THEORETICAL_MAX];

/**
* @brief returns the highest priority thread that is RUNNABLE 
*
* @param[in] queue queue to enqueue into
* @param[in] c character to enqueue
* @param[out] (uint32_t)-1 if no threads are RUNNABLE, otherwise returns the max dynamic priority of RUNNABLE threads
* 
* @return uint32_t
*/
uint32_t get_highest_priority_thread(){
    uint32_t max_prio_thread = (uint32_t)-1;
    uint32_t max_prio = (uint32_t) -1;
    for(uint32_t x = 0; x < NUM_USER_THREADS; x++){
        if(TCB_THREADS[x].stat == RUNNABLE && TCB_THREADS[x].dynamic_priority < max_prio){
            max_prio = TCB_THREADS[x].dynamic_priority;
            max_prio_thread = x;
        }
    }

    return max_prio_thread;
}


/**
* @brief uses round robin scheduling to return next thread to be run
*
* @param[out] next thread to be run
* 
* @return uint32_t
*/
uint32_t round_robin_scheduler(){
  uint32_t last_run_thread = RUNNING_THREAD;
  uint32_t next_thread = RUNNING_THREAD + 1;

  // rest of function finds the index of next thread to be run and returns it
  if(next_thread >= MAX_USER_THREADS){
    next_thread = 0;
  }

  while(TCB_THREADS[next_thread].stat != RUNNABLE){
    // NOTE: also what about the edge case that the thread we just prempted is the default thread and there are no RUNNABLE user threads? This cause an infinite loop here since we are only checking against max number of user threads? we should run idle function in this case.
    if(next_thread == last_run_thread){
        
        // This occurs when none of the user threads are runnable. This is when the idle function is supposed to run but is only relevant for RMS
        
        break; 
        
    }
    if(next_thread >= MAX_USER_THREADS){
        next_thread = 0;
    }
    else{
        next_thread++;
    }
  }
  
  return next_thread;
}

/**
* @brief uses rate monotonic scheduling with PCP to schedule threads
*
* @param[out] next thread to be run
* 
* @return uint32_t
*/
uint32_t pcp_scheduler(){
    
    if(TOTAL_THREADS_CREATED == 0){
        return DEFAULT_THREAD_INDEX;
    }

    for(uint32_t i = 0; i < NUM_USER_THREADS; i++){
        if(TCB_THREADS[i].stat == RUNNING){
            return i;
        }
        
    }

    // no threads running
    uint32_t n = get_highest_priority_thread();
    if(n != ((uint32_t)-1)){
        return n;
    }

    return IDLE_THREAD_INDEX;

}

/**
* @brief increments ticks, charges relevant thread, checks which thread need to be made runnable, then checks if pre-emption needs to happen
*
* @return void
*/
void systick_c_handler() { 
    TICK_COUNTER++;
    TCB_THREADS[RUNNING_THREAD].ticks_computation_time++;
    TCB_THREADS[RUNNING_THREAD].ticks_ct_from_beginning++;
    
    // check if any waiting are done and ready to be made runnable
    for(uint32_t i = 0; i < NUM_USER_THREADS; i++){
        if(TCB_THREADS[i].stat == WAITING && TICK_COUNTER >= TCB_THREADS[i].end_of_period){
            TCB_THREADS[i].stat = RUNNABLE;
            TCB_THREADS[i].end_of_period += TCB_THREADS[i].T;
        }
    }
    
    for (uint32_t j = 0; j < NUM_USER_THREADS; j++) {
        if (TCB_THREADS[j].stat == RUNNABLE
                && TCB_THREADS[RUNNING_THREAD].dynamic_priority > TCB_THREADS[j].dynamic_priority){
            TCB_THREADS[RUNNING_THREAD].stat = RUNNABLE;
            pend_pendsv();
            return;
        }
    }
    
    if(TCB_THREADS[RUNNING_THREAD].ticks_computation_time >= TCB_THREADS[RUNNING_THREAD].C){
       TCB_THREADS[RUNNING_THREAD].ticks_computation_time = 0U;
       TCB_THREADS[RUNNING_THREAD].stat = WAITING;
    }

    // set pendsv to pending
    pend_pendsv();
}

/**
* @brief performs a context switch
*
* @param[out] address of msp of next thread to be run
* 
* @return void*
*/
void *pendsv_c_handler(interrupt_stack_frame* msp){
  
  // clear pendsv 
  clear_pendsv();
  
  // if we are in any thread that is not the default thread, disable memory protection for that thread 
  if(RUNNING_THREAD != DEFAULT_THREAD_INDEX && TCB_THREADS[RUNNING_THREAD].p_mode == PER_THREAD){
    mm_region_disable(6);
    mm_region_disable(7);
  }
  
  // save current thread's context, see section 3.1 paragraph starting with "We will be saving a thread's..."
  TCB_THREADS[RUNNING_THREAD].msp = (uint32_t) msp;

  // save whether the thread was in an svc so that it can be restored correctly next time the thread is run
  TCB_THREADS[RUNNING_THREAD].svc_stat = get_svc_status();
  
  // set RUNNING_THREAD to value of next RUNNABLE thread returned by scheduler
  RUNNING_THREAD = pcp_scheduler();
  TCB_THREADS[RUNNING_THREAD].stat = RUNNING;

  // set svc_status if the thread that is going to be run was previously interrupted inside of an svc
  set_svc_status(TCB_THREADS[RUNNING_THREAD].svc_stat);

  // if we are going to a thread that is not the default thread, enable memory protection for that thread
  if(RUNNING_THREAD != DEFAULT_THREAD_INDEX && TCB_THREADS[RUNNING_THREAD].p_mode == PER_THREAD){
    mm_region_enable(6, (void *)(TCB_THREADS[RUNNING_THREAD].psp_static_start - PER_THREAD_STACK_SIZE), mm_log2ceil_size(PER_THREAD_STACK_SIZE), 0, 1); // user stack
    mm_region_enable(7, (void *)(TCB_THREADS[RUNNING_THREAD].msp_static_start - PER_THREAD_STACK_SIZE), mm_log2ceil_size(PER_THREAD_STACK_SIZE), 0, 1); // kernel stack
  }
  
  // enable entire region if in kernel mode
  if(TCB_THREADS[RUNNING_THREAD].p_mode == KERNEL_ONLY){
    mm_region_enable(6, (void *)&__thread_u_stacks_low, mm_log2ceil_size(32*1024), 0, 1);
    mm_region_enable(7, (void *)&__thread_k_stacks_low, mm_log2ceil_size(32*1024), 0, 1);
  }
  
  return (void *) TCB_THREADS[RUNNING_THREAD].msp;
}

/**
* @brief initializes thread data structures
*
* @param[in] max_threads max threads the user will create 
* @param[in] stack_size stack size of each thread
* @param[in] idle_fn pointer to function idle thread will run
* @param[in] memory_protection memory protection mode that threads will run in
* @param[in] max_mutexes maximum number of mutexes that will be used
*
* @return 0 on success, -1 on failure
*/
int sys_thread_init(
  uint32_t max_threads,
  uint32_t stack_size,
  void *idle_fn,
  protection_mode memory_protection,
  uint32_t max_mutexes
){
  
  if(max_threads > MAX_USER_THREADS || max_threads == 0){
    return -1;
  }

  NUM_USER_THREADS = max_threads;
 
  uint32_t requested_exponent = mm_log2ceil_size(4 * stack_size); 
  
  if(requested_exponent > MAX_BIT_SHIFT){ // can't shift by more than 31 bits....
    return -1;
  }

  PER_THREAD_STACK_SIZE = 1U << requested_exponent;
  
  if(PER_THREAD_STACK_SIZE * (NUM_USER_THREADS + 1) > TOTAL_MAX_STACK_SIZES){
    return -1;
  }
  else if(PER_THREAD_STACK_SIZE < MIN_STACK_SIZE){ // minimum stack size of 1kb, see 3.3
    PER_THREAD_STACK_SIZE = MIN_STACK_SIZE;
  }
  
  // initialize msp and psp stacks of all user threads and idle thread in the corresponding TCB
  uint32_t current_k_stack_addr = (uint32_t)&__thread_k_stacks_top; 
  uint32_t current_u_stack_addr = (uint32_t)&__thread_u_stacks_top; 
  for(uint32_t t = 0; t < NUM_USER_THREADS; t++){ 
        TCB_THREADS[t].msp = current_k_stack_addr;
        TCB_THREADS[t].msp_static_start = current_k_stack_addr; // needed in case we reset a thread
        TCB_THREADS[t].psp_static_start = current_u_stack_addr;
        current_k_stack_addr -= PER_THREAD_STACK_SIZE;
        current_u_stack_addr -= PER_THREAD_STACK_SIZE;
        TCB_THREADS[t].p_mode = memory_protection; // initialize protection mode

  }
  
  for(uint32_t i = 0; i < MAX_USER_THREADS; i++){
        TCB_THREADS[i].stat = DEAD;
  }

  TCB_THREADS[IDLE_THREAD_INDEX].msp = current_k_stack_addr;
  TCB_THREADS[IDLE_THREAD_INDEX].msp_static_start = current_k_stack_addr;
  TCB_THREADS[IDLE_THREAD_INDEX].psp_static_start = current_u_stack_addr;
  TCB_THREADS[IDLE_THREAD_INDEX].stat = RUNNABLE;
  TCB_THREADS[IDLE_THREAD_INDEX].p_mode = memory_protection;

  // initialize idle thread
  user_stack_frame* psp_stack_frame_start = (user_stack_frame*)(TCB_THREADS[IDLE_THREAD_INDEX].psp_static_start - sizeof(user_stack_frame));
  
  interrupt_stack_frame* msp_stack_frame_start = (interrupt_stack_frame*)(TCB_THREADS[IDLE_THREAD_INDEX].msp_static_start - sizeof(interrupt_stack_frame));

  if(idle_fn != NULL){ // user gives idle function
    psp_stack_frame_start->pc = (uint32_t)idle_fn;
  }
  else{ // use default idle function
    psp_stack_frame_start->pc = (uint32_t)&idle_default;
    psp_stack_frame_start->lr = (uint32_t)&thread_kill; 
  }

  psp_stack_frame_start->xPSR = XPSR_INIT;
  
  msp_stack_frame_start->rtn_code = RETURN_CODE_USER;
  msp_stack_frame_start->psp = (uint32_t)psp_stack_frame_start;
  
  // update position of idle thread's msp stack and set its initial svc_status to inactive
  TCB_THREADS[IDLE_THREAD_INDEX].msp = (uint32_t)msp_stack_frame_start;
  TCB_THREADS[IDLE_THREAD_INDEX].svc_stat = SVC_INACTIVE; 
  TCB_THREADS[IDLE_THREAD_INDEX].stat = RUNNABLE;

  // initialize global TICK_COUNTER
  TICK_COUNTER = 0;

  // initialize running thread global variable to 15 since default thread is running
  RUNNING_THREAD = 15;
  
  // set number of max mutexes
  MAX_NUM_MUTEXES = max_mutexes;

  return 0;
}

/**
* @brief creates a thread
*
* @param[in] fn function that the thread will run
* @param[in] prio priority of the thread
* @param[in] C worst case execution time, measured in ticks
* @param[in] T time period, measured in ticks
* @param[in] vargp pointer to arguments to pass into thread
*
* @return 0 on success, -1 on failure
*/
int sys_thread_create(
  void *fn,
  uint32_t prio,
  uint32_t C,
  uint32_t T,
  void *vargp
){
  if(TOTAL_THREADS_CREATED + 1 > NUM_USER_THREADS){
    return -1;
  }
  
  if(prio >= MAX_USER_THREADS){ // lowest priority user thread is 13, anything lower is error
    return -1;
  }
  else if(C > T){
    return -1;
  }

  TOTAL_THREADS_CREATED++;

  // NOTE: the msp and psp_static_start values of all threads are initialized in sys_thread_init
  TCB_THREADS[prio].C = C;
  TCB_THREADS[prio].T = T;
  TCB_THREADS[prio].static_priority = prio;
  TCB_THREADS[prio].dynamic_priority = prio;
  TCB_THREADS[prio].stat = RUNNABLE;
  TCB_THREADS[prio].end_of_period = TICK_COUNTER + T;
  TCB_THREADS[prio].ticks_computation_time = 0;
  TCB_THREADS[prio].ticks_ct_from_beginning = 0;
  TCB_THREADS[prio].fn = (uint32_t)fn;
  
  // ub test
  TOTAL_USABILITY += (float)C / (float)T;
  if(TOTAL_USABILITY > ub_table[TOTAL_THREADS_CREATED]){
    TOTAL_USABILITY -= (float)C / (float)T;
    TCB_THREADS[prio].stat = DEAD;
    TOTAL_THREADS_CREATED--;
    return -1;
  }

  // initialize thread so when it goes into pendsv the first time it runs fn with vargp as argument
  user_stack_frame* psp_stack_frame_start = (user_stack_frame*)(TCB_THREADS[prio].psp_static_start - sizeof(user_stack_frame));
  psp_stack_frame_start->lr = (uint32_t)(&thread_kill); 
  psp_stack_frame_start->pc = (uint32_t)fn;
  psp_stack_frame_start->r0 = (uint32_t)vargp;
  psp_stack_frame_start->xPSR = XPSR_INIT;

  // initialize the msp stack with return code and psp
  interrupt_stack_frame* msp_stack_frame_start = (interrupt_stack_frame*)(TCB_THREADS[prio].msp_static_start - sizeof(interrupt_stack_frame));
  msp_stack_frame_start->rtn_code = RETURN_CODE_USER; 
  msp_stack_frame_start->psp = (uint32_t)psp_stack_frame_start;
  
  // update position of thread's msp stack and set its initial svc_status to inactive
  TCB_THREADS[prio].msp = (uint32_t)msp_stack_frame_start;
  TCB_THREADS[prio].svc_stat = SVC_INACTIVE;

  return 0;
}

/**
* @brief starts the scheduler
*
* @param[in] frequency frequency at which systick will occur
*
* @return 0 on success, -1 on failure
*/
int sys_scheduler_start( uint32_t frequency ){
 
  // NOTE: default thread's context is saved the first time that pendsv does a context switch

  timer_start(frequency);

  // set PendSV, this will be serviced immediately
  pend_pendsv();

  // if we get here all threads besides idle_thread have returned or been killed, so now resume default thread

  return 0;
}

/**
* @brief returns priority of running thread
*
* @return priority of running thread
*/
uint32_t sys_get_priority(){
  return TCB_THREADS[RUNNING_THREAD].dynamic_priority;
}

/**
* @brief returns number of ticks since scheduler was started
*
* @return number of ticks since scheduler was started
*/
uint32_t sys_get_time(){
  return TICK_COUNTER;
}

/**
* @brief returns number of ticks a thread has run for
*
* @return number of ticks a thread has run for
*/
// see section 4.7
uint32_t sys_thread_time(){
  return TCB_THREADS[RUNNING_THREAD].ticks_ct_from_beginning;
}

/**
* @brief kill the currently running thread
*
* @return void
*/
void sys_thread_kill(){
    
    if(RUNNING_THREAD == IDLE_THREAD_INDEX){
        // set idle thread to instead use default idle thread function
        user_stack_frame* psp_stack_frame_start = (user_stack_frame*)(TCB_THREADS[IDLE_THREAD_INDEX].psp_static_start - sizeof(user_stack_frame));
  
        interrupt_stack_frame* msp_stack_frame_start = (interrupt_stack_frame*)(TCB_THREADS[IDLE_THREAD_INDEX].msp_static_start - sizeof(interrupt_stack_frame));

        psp_stack_frame_start->pc = (uint32_t)&idle_default;
  
        // update position of idle thread's msp stack and set its initial svc_status to inactive
        TCB_THREADS[IDLE_THREAD_INDEX].msp = (uint32_t)msp_stack_frame_start;
        
        pend_pendsv(); 
    }
    else if(RUNNING_THREAD == DEFAULT_THREAD_INDEX){ 
        sys_exit(0);
    }
    else{ // all user threads
        TOTAL_THREADS_CREATED--;
        TCB_THREADS[RUNNING_THREAD].stat = DEAD;
        TOTAL_USABILITY -= (float)TCB_THREADS[RUNNING_THREAD].C / (float)TCB_THREADS[RUNNING_THREAD].T;
        
        if(TOTAL_THREADS_CREATED == 0){
            timer_stop();
        }
        
        pend_pendsv();
    }
}

/**
* @brief running thread yields, see section 4.6 and page 29 of write up
*
* @return void
*/
void sys_wait_until_next_period(){ 
   
   for(uint32_t m = 0; m < MAX_NUM_MUTEXES; m++){
      if(MUTEXES[m].locked_by == RUNNING_THREAD){
        printk("WARNING: End of period but mutex is still held by thread.");
      }
   }

   TCB_THREADS[RUNNING_THREAD].stat = WAITING;
   TCB_THREADS[RUNNING_THREAD].ticks_computation_time = 0U;

   pend_pendsv();
}

/**
* @brief used by kernel to initialize mutexes to default values
*
* @return void
*/
void kernel_initialize_mutexes(){
    for(uint32_t m = 0; m < MAX_NUM_MUTEXES; m++){
        MUTEXES[m].locked_by = UNINITIALIZED_MUTEX_LOCK;
        MUTEXES[m].prio_ceil = UNINITIALIZED_PRIO_CEIL;
    }
}

/**
* @brief used by kernel to initialize mutexes to default values
*
* @param[in] max_prio max priority of mutex
*
* @return pointer to mutex that was initialized
*/
kmutex_t *sys_mutex_init( uint32_t max_prio ) {
 
  // NOTE: all mutexes have default values set in kernel_main when it calls kernel_initializes_mutexes()
  for(uint32_t m = 0; m < MAX_NUM_MUTEXES; m++){
    if(MUTEXES[m].prio_ceil == UNINITIALIZED_PRIO_CEIL){
        MUTEXES[m].prio_ceil = max_prio;
        MUTEXES[m].blocked_threads = 0;
        return &(MUTEXES[m]);
    }
  }
  return NULL;
}

/**
* @brief checks whether mutex is lockable
*
* @param[in] mutex mutex that is checked for lockability
*
* @return 0 if not lockable, returns 1 if lockable 
*/
uint32_t mutex_lockable(kmutex_t *mutex){
  
  uint32_t flag = 1;
  
  // mutex locked
  if(mutex->locked_by != UNINITIALIZED_MUTEX_LOCK){
     flag = 0;
     mutex->blocked_threads |= (1 << RUNNING_THREAD);  
     TCB_THREADS[mutex->locked_by].dynamic_priority = mutex->prio_ceil;
  }
 

  // check against priority ceilings of all locked mutexes
  for(uint32_t m = 0; m < MAX_NUM_MUTEXES; m++){
    if(MUTEXES[m].locked_by != UNINITIALIZED_MUTEX_LOCK && 
       TCB_THREADS[RUNNING_THREAD].dynamic_priority >= MUTEXES[m].prio_ceil && 
       MUTEXES[m].locked_by != RUNNING_THREAD){
        
        flag = 0;
        MUTEXES[m].blocked_threads |= (1 << RUNNING_THREAD);  
            TCB_THREADS[MUTEXES[m].locked_by].dynamic_priority = MUTEXES[m].prio_ceil;
    }
  }
  
  if(flag == 0){
    return 0;
  }

  return 1;
}

/**
* @brief locks the given mutex, uses PCP
*
* @param[in] mutex that is attempting to be locked
*
* @return void
*/
void sys_mutex_lock( kmutex_t *mutex ) {
  
  // check valid prio_ceil
  if(mutex->prio_ceil > MAX_NUM_MUTEXES || TCB_THREADS[RUNNING_THREAD].static_priority < mutex->prio_ceil){
    printk("WARNING: Bad prio ceiling value\n");
    sys_thread_kill();
    return;
  }
  
  // check if locking a mutex it already locked
  if(mutex->locked_by == RUNNING_THREAD){
    printk("WARNING: Thread tried to lock a mutex it already has\n");
    return;
  }
  
  //race condition between breaking while loop and acquiring mutex, need to disable interrupts
  int current_state = save_interrupt_state_and_disable();
  
  // block if can't get lock
  while(!mutex_lockable(mutex)){
      
      TCB_THREADS[RUNNING_THREAD].stat = WAITING;
      
      restore_interrupt_state(current_state);
      pend_pendsv();
      current_state = save_interrupt_state_and_disable();
  }

  mutex->locked_by = RUNNING_THREAD;
  restore_interrupt_state(current_state);
}

/**
* @brief unlocks the given mutex, uses PCP
*
* @param[in] mutex that is attempting to be unlocked
*
* @return void
*/
void sys_mutex_unlock( kmutex_t *mutex ) {
  
  if(mutex->locked_by != RUNNING_THREAD) {
      printk("WARNING: this thread has not locked this mutex\n");
      return;
  }

  // find highest priority blocked thread
  uint32_t max_prio = UNINITIALIZED_PRIO_CEIL;
  uint32_t max_thread = (uint32_t)-1;
  for(uint32_t t = 0; t < NUM_USER_THREADS; t++){
    if((mutex->blocked_threads & (1 << t)) && TCB_THREADS[t].dynamic_priority < max_prio){
        max_prio = TCB_THREADS[t].dynamic_priority;
        max_thread = t;
    }
  }

  // set status of highest priority blocked thread to runnable, skip if no threads are being blocked
  if(max_thread != (uint32_t)-1){
    TCB_THREADS[max_thread].stat = RUNNABLE;
    mutex->blocked_threads &= ~(1 << max_thread);
  }
  
  // restore priority of running thread
  uint32_t max_prio_ceil = TCB_THREADS[RUNNING_THREAD].static_priority;
  for(uint32_t m = 0; m < MAX_NUM_MUTEXES; m++){
    if(&MUTEXES[m] != mutex && MUTEXES[m].locked_by == RUNNING_THREAD && max_prio_ceil > MUTEXES[m].prio_ceil){
      max_prio_ceil = MUTEXES[m].prio_ceil;
    }
  }

  TCB_THREADS[RUNNING_THREAD].dynamic_priority = max_prio_ceil;
  mutex->locked_by = UNINITIALIZED_MUTEX_LOCK;
  TCB_THREADS[RUNNING_THREAD].stat = RUNNABLE; // change RUNNING_THREAD from RUNNING to RUNNABLE
  // check if any waiting are done and ready to be made runnable
  for(uint32_t i = 0; i < NUM_USER_THREADS; i++){
      if(TCB_THREADS[i].stat == WAITING && TICK_COUNTER >= TCB_THREADS[i].end_of_period){
          TCB_THREADS[i].stat = RUNNABLE;
          TCB_THREADS[i].end_of_period += TCB_THREADS[i].T;
      }
  }
    
    // check for pre-emption
  for (uint32_t j = 0; j < NUM_USER_THREADS; j++) {
      if (j != RUNNING_THREAD && TCB_THREADS[j].stat == RUNNABLE
                && TCB_THREADS[RUNNING_THREAD].dynamic_priority > TCB_THREADS[j].dynamic_priority){
          TCB_THREADS[RUNNING_THREAD].stat = RUNNABLE;
          pend_pendsv();
          return;
        }
    }
  if(TCB_THREADS[RUNNING_THREAD].ticks_computation_time >= TCB_THREADS[RUNNING_THREAD].C){
     TCB_THREADS[RUNNING_THREAD].ticks_computation_time = 0U;
     TCB_THREADS[RUNNING_THREAD].stat = WAITING;
     pend_pendsv();
    }
}
