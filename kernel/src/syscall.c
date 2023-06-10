/**
 * @file syscall.c
 *
 * @brief functions that get called when an svc # is called
 *
 * @date 10/17/2021
 *
 * @author Carlos Gonzalez (cggonzal) MeeDm Bossard (mbossard)
 */

#include <unistd.h>
#include <syscall.h>
#include <uart.h>
#include <stdio.h>
#include <printk.h>
#include <led_driver.h>
#include <arm.h>

/** heap_low from linker script*/
extern uint32_t __heap_low;
/** heap_top from linker script */
extern uint32_t __heap_top;
/** address of next empty place in heap */
uint32_t HEAP_ADDRESS = (uint32_t)&__heap_low;

/**
* @brief keeps track of whether more space can be
* made in the heap.
*
* @param[in] incr number of bytes to increase heap by
* 
* @return on success, returns pointer to heap_low casted to void*
* and on failure returns (void*) -1
*/
void *sys_sbrk(int incr){
  if(incr < 0){
    printk("ERROR: given negative value to sys_sbrk\n");
  }
  uint32_t address = HEAP_ADDRESS;
  //checks if possible
  if (address + ((uint32_t)incr) > (uint32_t)&__heap_top) {
    return (void *) -1;
  }

  //if possible returns address
  HEAP_ADDRESS += (uint32_t)incr;
  return (void *)(address);
}


/**
* @brief writes whats in pointer to standard out
*
* @param[in] file file descriptor, should always be standard out
* @param[in] ptr points to buffer containing char * to write to standard out
* @param[in] len length of buffer
* 
* @return on success, returns number of bytes written,
* and on case of failure returns -1
*/
int sys_write(int file, char *ptr, int len){
  //1 for stdout
  if (file != 1) {return -1;}  
  
  for (int i = 0; i < len; i++) {
    while(uart_put_byte(ptr[i]));
  }
  
  return len;
}


/**
* @brief reads from stdin, puts read bytes into ptr 
* up till length len, then echos back to standard out
*
* @param[in] file file descriptor, should always be standard in
* @param[in] ptr points to buffer containing char * to read from standard in
* @param[in] len length of buffer
*
* @return number of bytes read, and on failure returns -1
*/
int sys_read(int file, char *ptr, int len){
  //0 for stdin
  if (file != 0) {return -1;}
  
  int count = 0;
  for (int i = 0; i < len; i++) {
    //wait until char to get
    while (uart_get_byte(ptr + i)) {}


    int ascii_val = (int)ptr[i];
    //EOF reached
    if (ascii_val == 4) {
      return i;

    //backspace pressed, so move index in ptr, and erases from minicom
    } else if (ptr[i] == '\b' && i > 1) {
      i -= 2;
      printk("\b \b");

    //enter pressed
    } else if (ptr[i] == '\n') {
      while (uart_put_byte('\n')) {} 
      return i+1;

    //\r encountered
    } else if (ptr[i] == '\r') {
      while (uart_put_byte('\n')) {}
      return i+1;
    
    //any other character
    } else {
      while (uart_put_byte(ptr[i])) {}
    }
    count = i;

  }

  return count;
}


/**
* @brief exits process by printing status and setting led to status
* 
* @param[in] status exit status
*/
void sys_exit(int status){
  //print exit code
  printk("%d", status);
  
  //flush uart buffer
  uart_flush();
  
  //display exit code on led screen
  led_set_display(status);

  //sleep with interrupts disabled
  disable_interrupts();

  //wait for interrupt instead
  while(1){
    wait_for_interrupt();
  }

}


