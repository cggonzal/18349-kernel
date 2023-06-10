/**
 * @file svc_handler.c 
 *
 * @brief interrupt handler for any svc call
 *
 * @date 10/24/2021
 *
 * @author Carlos Gonzalez (cggonzal) MeeDm Bossard (mbossard)
 */

#include <stdint.h>
#include <debug.h>
#include "svc_num.h"
#include "syscall.h"
#include "syscall_thread.h"
#include "syscall_mutex.h"

/**
* @brief struct for stack frame
* taken from lecture 10, slide 22
*/
typedef struct {
  uint32_t r0;   /**< register 0 >*/
  uint32_t r1;   /**< register 1 >*/
  uint32_t r2;   /**< register 2 >*/
  uint32_t r3;   /**< register 3 >*/
  uint32_t r12;  /**< register 12 >*/
  uint32_t lr;   /**< link register >*/
  uint32_t pc;   /**< prgram counter >*/
  uint32_t xPSR; /**< xPSR>*/
  uint32_t arg4; /**< arg4> gets pushed onto the stack before rest of stack is stored*/ 
} stack_frame_t;

/**
* @brief interrupt handler for svc calls
* calls appropriate function for svc number
* 
* param[in] psp pointer to stack
*/
void svc_c_handler(void *psp) {
  stack_frame_t *s = (stack_frame_t*)psp;
  uint32_t pc = s->pc;
  uint8_t svc_number = *(((uint8_t *)pc)-2);

  switch (svc_number) {
    case SVC_SBRK:
      s->r0 = (uint32_t) sys_sbrk((int)s->r0); 
      break;
    case SVC_WRITE:
      s->r0 = (uint32_t) sys_write((int)s->r0, (char *)s->r1, (int)s->r2);
      break;
    case SVC_CLOSE:
      break;
    case SVC_FSTAT:
      break;
     case SVC_ISATTY:
      break;
    case SVC_LSEEK:
      break;
    case SVC_READ:
      s->r0 = (uint32_t) sys_read((int)s->r0, (char *)s->r1, (int)s->r2);
      break;
    case SVC_EXIT:
      sys_exit((int)s->r0);
      break;
    //case SVC_KILL:
      //break;
    case SVC_THR_INIT:
      s->r0 = (uint32_t) sys_thread_init((uint32_t)s->r0, (uint32_t)s->r1, (void*) s->r2, (protection_mode)s->r3, (uint32_t)s->arg4);
      break;
    case SVC_THR_CREATE:
      s->r0 = (uint32_t) sys_thread_create((void*)s->r0, (uint32_t)s->r1, (uint32_t)s->r2, (uint32_t)s->r3, (void*)s->arg4);
      break;
    case SVC_THR_KILL:
      sys_thread_kill();
      break;
    //case SVC_GET_PID:
      //break;
    case SVC_MUT_INIT:
      s->r0 = (uint32_t) sys_mutex_init((uint32_t)s->r0);
      break;
    case SVC_MUT_LOK:
      sys_mutex_lock((kmutex_t*)s->r0);
      break;
    case SVC_MUT_ULK:
      sys_mutex_unlock((kmutex_t*)s->r0);
      break;
    case SVC_WAIT:
      sys_wait_until_next_period();
      break;
    case SVC_TIME:
      s->r0 = (uint32_t)sys_get_time();
      break;
    case SVC_SCHD_START:
      s->r0 = (uint32_t)sys_scheduler_start((uint32_t)s->r0);
      break;
    case SVC_PRIORITY:
      s->r0 = (uint32_t)sys_get_priority();
      break;
    case SVC_THR_TIME:
      s->r0 = (uint32_t)sys_thread_time();
      break;
    //case SVC_SLEEP_TILL_INT:
      //break;
    case SVC_SERVO_ENABLE:
      //s->r0 = (uint32_t)sys_servo_enable((uint8_t)s->r0, (uint8_t)s->r1);
      break;
    case SVC_SERVO_SET:
      //s->r0 = (uint32_t)sys_servo_set((uint8_t)s->r0, (uint8_t)s->r1);
      break;
    default:
      DEBUG_PRINT( "svc num: %d does not exist\n", svc_number );
      ASSERT( 0 );
  }
  

}
