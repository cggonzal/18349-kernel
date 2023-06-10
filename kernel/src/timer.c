/**
 * @file timer.c
 *
 * @brief this file implements the systick
 * interrupt handler
 *
 * @date 10/8/2021
 *
 * @author Carlos Gonzalez (cggonalez)
 * @author MeeDm Bossard (mbossard)
 */

#include <timer.h>
#include <unistd.h>
#include <printk.h> 
#include <gpio.h>

/** @brief The SysTick Register Map **/
struct systick_reg_map {
  volatile uint32_t CTRL;  /**<Systick Control and Status Register>**/
  volatile uint32_t LOAD;  /**<Reload Value Register>**/
  volatile uint32_t VAL;   /**<Current Value Register>**/
  volatile uint32_t CALIB; /**<Calibration Value Register>**/
};

/** systick base address */
#define SYSTICK_BASE (struct systick_reg_map *) 0xE000E010

/** enable systick timer */
#define SYSTICK_TIMER_EN (1 << 0)
/** enable systick exception request */
#define SYSTICK_TICKINT_EN (1 << 1)
/** disable timer */
#define SYSTICK_TIMER_DISABLE (~(1 << 1))
/** ahb clock speed */
#define AHB_CLK_SPD 16000000


/**
* @brief starts the timer to count down at given frequency
*
* @param[in] frequency freq of timer counting down
* @return 0 always
*/
int timer_start(int frequency){

  struct systick_reg_map *systick = SYSTICK_BASE;
   
  // reload value is num clock cycles for freq 
  int reload_val = AHB_CLK_SPD/(8*frequency);

  //load reload, enable timer, enable systick interrupt
  systick->LOAD |= reload_val; 
  systick->CTRL |= SYSTICK_TIMER_EN;
  systick->CTRL |= SYSTICK_TICKINT_EN;

  return 0;
}

/**
* @brief simply stops the timer from running
*/

void timer_stop(){
  struct systick_reg_map *systick = SYSTICK_BASE;
  systick->CTRL &= SYSTICK_TIMER_DISABLE;
}

