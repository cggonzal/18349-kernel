/** @file    mpu.c
 *
 *  @brief   MPU interface implementation for thread-wise
 *           memory protection.
 *
 *  @date    30 Mar 2020
 *
 *  @author  Benjamin Huang <zemingbh@andrew.cmu.edu>
 */

#include "arm.h"
#include "debug.h"
#include "printk.h"
#include "syscall.h"
#include "mpu.h"
#include "syscall_thread.h"

// sizes of user stacks. See section 9.1
#define USER_CODE_SIZE 16 * 1024
#define USER_READ_ONLY_SIZE 2 * 1024
#define USER_DATA_SIZE 1 * 1024
#define USER_BSS_SIZE 1 * 1024
#define USER_HEAP_SIZE 4 * 1024
#define DEFAULT_THREAD_STACK_SIZE 2 * 1024

// memory protection, page 235 in M4 programming manual
#define MEMFAULTENA (1 << 16)

// page 201 in M4 programming manual
#define MPU_ENABLE (1 << 0)

// linker variables needed for memory protection
extern char 
  _user_text_start,
  _user_text_end,
  _u_rodata,
  _u_erodata,
  _u_data,
  _u_edata,
  _u_bss,
  _u_ebss,
  __heap_low,
  __heap_top,
  __psp_stack_top,
  __psp_stack_bottom,
  _swi_stub_start;



#define UNUSED __attribute__((unused))

/**
 * @struct mpu_t
 * @brief  MPU MMIO register layout.
 */
typedef struct {
  /**@brief provides information about the MPU */
  volatile uint32_t TYPER;
  /**@brief MPU enable/disable and background region control. */
  volatile uint32_t CTRL;
  /** @brief Select which MPU region to be configured*/
  volatile uint32_t RNR;
  /**@brief Defines base address of a MPU region */
  volatile uint32_t RBAR;
  /**@brief Defines size and attribues of a MPU region*/
  volatile uint32_t RASR;

  /**@brief Field aliases. */
  //@{
  volatile uint32_t RBAR_A1;
  volatile uint32_t RASR_A1;
  volatile uint32_t RBAR_A2;
  volatile uint32_t RASR_A2;
  volatile uint32_t RBAR_A3;
  volatile uint32_t RASR_A3;
  //@}
} mpu_t;

/**
 * @struct system_control_block_t
 * @brief  System control block register layout.
 */
typedef struct {
  /**@brief System handler control and state register.*/
  volatile uint32_t SHCRS;
  /**@brief Configurable fault status register.*/
  volatile uint32_t CFSR;
  /**@brief HardFault Status Register */
  volatile uint32_t HFSR;
  /**@brief Hint information of debug events.*/
  volatile uint32_t DFSR;
  /**@brief Addr value of memfault*/
  volatile uint32_t MMFAR;
  /**@brief Addr of bus fault */
  volatile uint32_t BFAR;
  /**@brief Auxilliary fault status register.*/
  volatile uint32_t AFSR;
} system_control_block_t;

/**@brief MPU base address.*/
#define MPU_BASE ( ( mpu_t * )0xE000ED90 );

/** @brief MPU CTRL register flags. */
//@{
#define CTRL_ENABLE_BG_REGION ( 1<<2 )
#define CTRL_ENABLE_PROTECTION ( 1<<0 )
//@}

/** @brief MPU RNR register flags. */
#define RNR_REGION ( 0xFF )
/** @brief Maximum region number. */
#define REGION_NUMBER_MAX 7

/** @brief MPU RBAR register flags. */
//@{
#define RBAR_VALID ( 1<<4 )
#define RBAR_REGION ( 0xF )
//@}

/** @brief MPU RASR register masks. */
//@{
#define RASR_XN ( 1<<28 )
#define RASR_AP_KERN ( 1<<26 )
#define RASR_AP_USER ( 1<<25 | 1<<24 )
#define RASR_SIZE ( 0b111110 )
#define RASR_ENABLE ( 1<<0 )
//@}

/** @brief MPU RASR AP user mode encoding. */
//@{
#define RASR_AP_USER_READ_ONLY ( 0b10<<24 )
#define RASR_AP_USER_READ_WRITE ( 0b11<<24 )
//@}

/**@brief Systen control block MMIO location.*/
#define SCB_BASE ( ( volatile system_control_block_t * )0xE000ED24 )

/**@brief Stacking error.*/
#define MSTKERR 0x1 << 4
/**@brief Unstacking error.*/
#define MUNSTKERR 0x1 << 3
/**@brief Data access error.*/
#define DACCVIOL 0x1 << 1
/**@brief Instruction access error.*/
#define IACCVIOL 0x1 << 0
/**@brief Indicates the MMFAR is valid.*/
#define MMARVALID 0x1 << 7

#define MAX_POSSIBLE_THREADS 16

extern uint32_t PER_THREAD_STACK_SIZE;

extern uint32_t RUNNING_THREAD;

extern TCB TCB_THREADS[MAX_POSSIBLE_THREADS];

void mm_c_handler( void *psp ) {

  system_control_block_t *scb = ( system_control_block_t * )SCB_BASE;
  int status = scb->CFSR & 0xFF;

  // Attempt to print cause of fault
  DEBUG_PRINT( "Memory Protection Fault\n" );
  WARN( !( status & MSTKERR ), "Stacking Error\n" );
  WARN( !( status & MUNSTKERR ), "Unstacking Error\n" );
  WARN( !( status & DACCVIOL ), "Data access violation\n" );
  WARN( !( status & IACCVIOL ), "Instruction access violation\n" );
  WARN( !( status & MMARVALID ), "Faulting Address = %x\n", scb->MMFAR );

  // You cannot recover from stack overflow because the processor has
  // already pushed the exception context onto the stack, potentially
  // clobbering whatever is in the adjacent stack.
  if (TCB_THREADS[RUNNING_THREAD].psp_static_start - PER_THREAD_STACK_SIZE > (uint32_t)psp) {
    DEBUG_PRINT( "Stack Overflow, aborting\n" );
    sys_exit( -1 );
  }

  // Other errors can be recovered from by killing the offending
  // thread. Standard thread killing rules apply. You should halt
  // if the thread is the main or idle thread!
  
  // TODO: You decide how to kill the thread.
  // Set the pc? Call a syscall? Context swap?
  sys_thread_kill();
}

/**
 * @brief  Enables a memory protection region. Regions must be aligned!
 *
 * @param  region_number      The region number to enable.
 * @param  base_address       The region's base (starting) address.
 * @param  size_log2          log[2] of the region size.
 * @param  execute            1 if the region should be executable by the user.
 *                            0 otherwise.
 * @param  user_write_access  1 if the user should have write access, 0 if
 *                            read-only
 *
 * @return 0 on success, -1 on failure
 */
int mm_region_enable(
  uint32_t region_number,
  void *base_address,
  uint8_t size_log2,
  int execute,
  int user_write_access
){
  if (region_number > REGION_NUMBER_MAX) {
    printk("Invalid region number\n");
    return -1;
  }

  if ((uint32_t)base_address & ((1 << size_log2) - 1)) {
    printk("Misaligned region\n");
    return -1;
  }

  if (size_log2 < 5) {
    printk("Region too small\n");
    return -1;
  }

  mpu_t *mpu = MPU_BASE;

  mpu->RNR = region_number & RNR_REGION;
  mpu->RBAR = (uint32_t)base_address;

  uint32_t size = ((size_log2 - 1) << 1) & RASR_SIZE;
  uint32_t ap = user_write_access ? RASR_AP_USER_READ_WRITE : RASR_AP_USER_READ_ONLY;
  uint32_t xn = execute ? 0 : RASR_XN;

  mpu->RASR |= size | ap | xn | RASR_ENABLE;

  return 0;
}

/**
 * @brief  Disables a memory protection region.
 *
 * @param  region_number      The region number to disable.
 */
void mm_region_disable( uint32_t region_number ){
  mpu_t *mpu = MPU_BASE;
  mpu->RNR = region_number & RNR_REGION;
  mpu->RASR &= ~RASR_ENABLE;
}

/**
 * @brief  Returns ceiling (log_2 n).
 */
uint32_t mm_log2ceil_size (uint32_t n) {
  uint32_t ret = 0;
  while (n > (1U << ret)) {
    ret++;
  }
  return ret;
}

// returns 0 on success, -1 on failure
int mm_initialize(){
  // enable memory management fault
  system_control_block_t *scb = ( system_control_block_t * )SCB_BASE;
  scb->SHCRS |= MEMFAULTENA;

  // enable MPU
  mpu_t *mpu = MPU_BASE;
  mpu->CTRL |= CTRL_ENABLE_BG_REGION; 
  mpu->CTRL |= CTRL_ENABLE_PROTECTION;
  // initialize memory protection for regions shown in 9.1
  // protect user code, TODO: mm_region_enable returns -1 because of "misaligned region"
  if(mm_region_enable(0, (void *)&_swi_stub_start, mm_log2ceil_size(USER_CODE_SIZE), 1, 0) < 0){
    return -1;
  }
  
  // protect user read-only data
  if(mm_region_enable(1, (void *)&_u_rodata, mm_log2ceil_size(USER_READ_ONLY_SIZE), 0, 0) < 0){
    return -1;
  }
  
  // protect user data
  if(mm_region_enable(2, (void *)&_u_data, mm_log2ceil_size(USER_DATA_SIZE), 0, 1) < 0){
    return -1;
  }

  // protect bss
  if(mm_region_enable(3, (void *)&_u_bss, mm_log2ceil_size(USER_BSS_SIZE), 0, 1) < 0){
    return -1;
  }

  // protect user heap
  if(mm_region_enable(4, (void *)&__heap_low, mm_log2ceil_size(USER_HEAP_SIZE), 0, 1) < 0){
    return -1;
  }
  
  // protect default thread user stack
  if(mm_region_enable(5, (void *)&__psp_stack_bottom, mm_log2ceil_size(DEFAULT_THREAD_STACK_SIZE), 0, 1) < 0){
    return -1;
  }



  return 0;
}

void mm_disable(){
    mm_region_disable(0);
    mm_region_disable(1);
    mm_region_disable(2);
    mm_region_disable(3);
    mm_region_disable(4);
    mm_region_disable(5);
}
