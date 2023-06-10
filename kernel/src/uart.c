/**
 * @file uart.c 
 *
 * @brief an interrupt based uart implementation 
 *
 * @date 10/9/2021
 *
 * @author Carlos Gonzalez (cggonzal)
 * @author MeeDm Bossard (mbossard)
 */

#include <unistd.h>
#include <rcc.h>
#include <uart.h>
#include <nvic.h>
#include <gpio.h>
#include <arm.h>
/** @brief The UART register map. */
struct uart_reg_map {
    volatile uint32_t SR;   /**< Status Register >*/
    volatile uint32_t DR;   /**<  Data Register >*/
    volatile uint32_t BRR;  /**<  Baud Rate Register >*/
    volatile uint32_t CR1;  /**<  Control Register 1 >*/
    volatile uint32_t CR2;  /**<  Control Register 2 >*/
    volatile uint32_t CR3;  /**<  Control Register 3 >*/
    volatile uint32_t GTPR; /**<  Guard Time and Prescaler Register >*/
};


/** @brief Base address for UART2 */
#define UART2_BASE  (struct uart_reg_map *) 0x40004400

/** irq number for usart 2 */
#define UART_IRQ_NUM 38
/**enable bit for txeie interrupt */
#define UART_TXEIE_EN (1 << 7)
/**enable bit for rxneie interrupt */
#define UART_RXNEIE_EN (1 << 5)
/** @brief Enable  Bit for UART Config register */
#define UART_EN (1 << 13)
/** enable UART clock */
#define UART_CLK_EN (1 << 17)
/** UART transmit enable bit */
#define UART_TE_EN (1 << 3)
/** UART receive enable bit */
#define UART_RE_EN (1 << 2)
/** UART transmit data register empty */
#define UART_TXE (1 << 7)
/** UART read data register not empty */
#define UART_RXNE (1 << 5)
/** max size to send or receive */
#define MAX_BYTE_NUM (uint32_t) 16

/** kernel side send buffer */
extern queue_t send_buffer;
/** kernel side receive buffer */
extern queue_t receive_buffer;


/**
* @brief enqueues character c into queue q, 
* and increments length of q
*
* @param[in] queue queue to enqueue into
* @param[in] c character to enqueue
* @param[out] queue gets changed according to enqueing
* write_from gets incremented and length gets incremented
*
* @return void
*/
void enqueue (queue_t *queue, char c) {
  queue->buf[queue->write_to] = c;
  queue->write_to++;
  if (queue->write_to >= BUF_SIZE) {
    queue->write_to = 0;
  }
  queue->len++;
  return;
}

/**
* @brief dequeues character c fro queue q, 
* and decrements length of q
*
* @param[in] queue queue to dequeue from
* @param[out] queue gets changed according to dequeing
* read_from gets incremented and length gets decremented
*
* @return c character that has been dequeue
*/
char dequeue (queue_t *queue) {
  char c = queue->buf[queue->read_from];
  queue->read_from++;
  if (queue->read_from >= BUF_SIZE) {
    queue->read_from = 0;
  }
  queue->len--;
  return c; 
}

/**
* @brief initializes queue and ensures its an empty queue
*
* @param[in] queue queue to make empty
* @param[out] queue becomes empty such that the
* the length is zero, all values are 0, read_from is 0, 
* and write_from is 0
*
* @return void
*/
void queue_init(queue_t *queue) {
  queue->read_from = 0;
  queue->write_to = 0;
  queue->len = 0;
  for (int i = 0; i < BUF_SIZE; i++) {
    queue->buf[i] = 0;
  }
  return;  
}


/**
* @brief initializes uart by initializing pins,
* registers, interrupts, and queues.
*
* @param[in] baud USART_DIV macro detailed in 
* kernel.c
*
* @return void
*/
void uart_init(int baud){
  // init PA_2, TX
  gpio_init(GPIO_A, 2, MODE_ALT, OUTPUT_PUSH_PULL, OUTPUT_SPEED_LOW, PUPD_NONE, ALT7);
    
  // init PA_3, RX
  gpio_init(GPIO_A, 3, MODE_ALT, OUTPUT_OPEN_DRAIN, OUTPUT_SPEED_LOW, PUPD_NONE, ALT7);

  struct rcc_reg_map *rcc = RCC_BASE;
  rcc->apb1_enr |= UART_CLK_EN;
    
  struct uart_reg_map *uart = UART2_BASE;
  uart->CR1 |= UART_EN;
  uart->CR1 |= UART_TE_EN;
  uart->CR1 |= UART_RE_EN;
  uart->BRR |= baud; // USART_DIV

  nvic_irq(UART_IRQ_NUM, IRQ_ENABLE);

  //initialize queue to be all zeros
  queue_init(&send_buffer);
  queue_init(&receive_buffer);

  uart->CR1 |= UART_RXNEIE_EN;  
  enable_interrupts(); 
  return;

}


/**
* @brief transmits a byte over UART by
* adding character to kernel's send_buffer
*
* @param[in] c character to be sent
*
* @return 0 on success, -1 on failure
*/
int uart_put_byte(char c){
  int current_state = save_interrupt_state_and_disable();
  struct uart_reg_map *uart = UART2_BASE;
 
  if (send_buffer.len == BUF_SIZE) {
    //buffer is full
    restore_interrupt_state(current_state);
    return -1;
  }
 
  enqueue(&send_buffer, c);

  uart->CR1 |= UART_TXEIE_EN; 
  restore_interrupt_state(current_state);
  return 0;
}

/**
* @brief recieves a byte over UART by
* taking out byte from recieve buffer
*
* @param[in] c character to be sent
*
* @return 0 on success, -1 on failure
*/
int uart_get_byte(char *c){
  int current_state = save_interrupt_state_and_disable();
  if (receive_buffer.len == 0) {
    //buffer is empty
    restore_interrupt_state(current_state);
    return -1;
  }
  
  *c = dequeue(&receive_buffer); 
  restore_interrupt_state(current_state);
  return 0;

}

/**
* @brief interrupt handler for when txe register is enabled
* or rxneie register is enabled, and actively sends or recieves up to 16 bytes
* 
* no parameters
*/
void uart_irq_handler(){
  struct uart_reg_map *uart = UART2_BASE;

  //send bytes (put_byte)
  if (uart->SR & UART_TXE) {
    //if length of buffer is smaller than 16 send only those
    //otherwise send 16 bytes
    uint32_t bytes_to_send = (send_buffer.len < MAX_BYTE_NUM) ? send_buffer.len : MAX_BYTE_NUM;
    for (uint32_t i = 0; i < bytes_to_send; i++) {
      char c = dequeue(&send_buffer);
      
      uart->DR = c; 

      //check if sent and ready for next one
      while (!(uart->SR & UART_TXE)) {}
    }
  }

  if(send_buffer.len == 0){
    uart->CR1 &= ~UART_TXEIE_EN;   
  }

  //something in uart buffer to receive
  if (uart->SR & UART_RXNE) {
    enqueue(&receive_buffer, uart->DR);  
  }

  nvic_clear_pending(UART_IRQ_NUM);
}

/**
* @brief sends remaining things until kernel buffer is flushed
*/
void uart_flush(){
  struct uart_reg_map *uart = UART2_BASE; 

  for (uint32_t i = 0; i < send_buffer.len; i++) {
    while (!(uart->SR & UART_TXE)) {}
    char c = dequeue(&send_buffer);
    uart->DR = c;
  }

}
