/**
 * @file kernel.c
 *
 * @brief      Kernel entry point
 *    simply initializes kernel then switches to user mode
 *
 * @date 10/24/2021
 * @author Carlos Gonzalez (cggonzal) MeeDm Bossard (mbossard)
 */

#include "arm.h"
#include "kernel.h"
#include "timer.h"
#include "uart.h"
#include "led_driver.h"
#include "i2c.h"
#include "gpio.h"
#include "mpu.h"
#include "syscall_mutex.h"

/** USART_DIV value pre-calculated */
#define USART_DIV 0x87
/** I2C CCR value pre-calculated */
#define I2C_CCR 0x50
/**timer tickes at 10,000 hz */
#define TIMER_FREQ 10000 //10000 hz


/**
* @brief initializes uart, timer, i2c, led driver,
* then enters user mode
*
* @return 0 always
*/
int kernel_main( void ) {
  init_349(); // DO NOT REMOVE THIS LINE
  uart_init(USART_DIV);  
  i2c_master_init(I2C_CCR); 
  led_driver_init(1); 
  kernel_initialize_mutexes();

  mm_initialize();

  enter_user_mode(); 
 
  mm_disable();

  return 0;
}
