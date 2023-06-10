/**
 * @file   swi_stubs.S
 *
 * @brief  Stub functions for newlib and 349libc syscalls
 *
 * @date 10/24/2021
 * @author Carlos Gonzalez (cggonzal) MeeDm Bossard (mbossard)
 */

.cpu cortex-m4
.syntax unified
.section .swi_stub
.thumb

#include "../../kernel/include/svc_num.h"


.global _start
_start:
  bkpt

.global _sbrk
_sbrk:
  svc #0
  bx lr
  bkpt

.global _write
_write:
  svc #1
  bx lr
  bkpt

.global _close
_close:
  svc #2
  bx lr
  bkpt

.global _fstat
_fstat:
  svc #3
  bx lr
  bkpt

.global _isatty
_isatty:
  svc #4
  bx lr
  bkpt

.global _lseek
_lseek:
  svc #5
  bx lr
  bkpt

.global _read
_read:
  svc #6
  bx lr
  bkpt

.global _exit
_exit:
  svc #7
  bx lr
  bkpt

.type thread_init, %function
.global thread_init
thread_init:
  svc #9
  bx lr
  bkpt

.type thread_create, %function
.global thread_create
thread_create:
  svc #10
  bx lr
  bkpt

.type thread_kill, %function
.global thread_kill
thread_kill:
  svc # 11
  bx lr
  bkpt

.type scheduler_start, %function
.global scheduler_start
scheduler_start:
  svc #12
  bx lr
  bkpt

.type mutex_init, %function
.global mutex_init
mutex_init:
  svc #13
  bx lr
  bkpt

.type mutex_lock, %function
.global mutex_lock
mutex_lock:
  svc #14
  bx lr
  bkpt

.type mutex_unlock, %function
.global mutex_unlock
mutex_unlock:
  svc #15
  bx lr
  bkpt

.type wait_until_next_period, %function
.global wait_until_next_period
wait_until_next_period:
  svc #16
  bx lr
  bkpt

.global get_time
get_time:
  svc #17
  bx lr
  bkpt

.global get_priority
get_priority:
  svc #19
  bx lr
  bkpt

.global thread_time
thread_time:
  svc #20
  bx lr
  bkpt

.global servo_enable
servo_enable:
  mov r0, #-1
  bx lr
  bkpt

.global servo_set
servo_set:
  mov r0, #-1
  bx lr
  bkpt

.thumb_func
.global idle_default
idle_default:
  wfi
    b idle_default


.global _kill
_kill:
  bkpt

.global _getpid
_getpid:
  bkpt
