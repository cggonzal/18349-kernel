/**
 * @file   main.c
 *
 * @brief  Tests for T enforcement
 *
 * @author Benjamin Huang <zemingbh@andrew.cmu.edu>
 */

#include <349_lib.h>
#include <349_threads.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

/** @brief thread user space stack size - 1KB */
#define USR_STACK_WORDS 256
#define NUM_THREADS 1
#define NUM_MUTEXES 0
#define CLOCK_FREQUENCY 1000

/** @brief Computation time of the task */
#define THREAD_C_MS 123
/** @brief Period of the thread */
#define THREAD_T_MS 1024

/** @brief Give some leeway when testing */
#define SLACK_MS 5

void thread_0( UNUSED void *vargp ) {
  int cnt = 0;

  while( 1 ) {
    int t = get_time();

    if ( t > THREAD_T_MS * cnt + SLACK_MS ) {
      printf( "Failed. Should have woken up close to t = %d\n", THREAD_T_MS * cnt );
      printf( "Woke up at t = %d", t );
      while ( 1 );
    }

    print_num_status_cnt( 0, cnt++ );

    if ( cnt > 20 ) {
      printf( "Test passed!\n" );
      while ( 1 );
    }

    printf( "Alarm set for t = %d.\n", THREAD_T_MS * cnt );
    spin_until( THREAD_T_MS * cnt - 1 );

  }
}

void idle_thread() {
  while( 1 );
}

int main( void ) {

  ABORT_ON_ERROR( thread_init( NUM_THREADS, USR_STACK_WORDS, &idle_thread, KERNEL_ONLY, NUM_MUTEXES ) );

  ABORT_ON_ERROR( thread_create( &thread_0, 0, THREAD_C_MS, THREAD_T_MS, NULL ) );

  printf( "Successfully created threads! Starting scheduler...\n" );
  ABORT_ON_ERROR( scheduler_start( CLOCK_FREQUENCY ) );

  return 0;

}
