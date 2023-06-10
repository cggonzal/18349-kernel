/**
 * @file uart.h
 *
 * @brief header file for uart.c
 *
 * @date 10/10/2021
 *
 * @author Carlos Gonzalez (cggonzal)
 * @author MeeDm Bossard (mbossard)
 */

#ifndef _UART_H_
#define _UART_H_

#define BUF_SIZE 512

/** @brief structure for a circular buffer*/
typedef struct queue {
  char buf[BUF_SIZE]; /**< buffer >*/
  uint32_t read_from; /**< index of first char to read from >*/
  uint32_t write_to; /**< index of first avail spot >*/
  uint32_t len; /**< length of buffer >*/
} queue_t;

/** kernel-side receive buffer */
queue_t receive_buffer;
/** kernel-side send buffer */
queue_t send_buffer;


void enqueue(queue_t *queue, char c);

char dequeue(queue_t *queue);

void queue_init(queue_t *queue);

void uart_init(int baud);

int uart_put_byte(char c);

int uart_get_byte(char *c);

void uart_flush();

#endif /* _UART_H_ */
