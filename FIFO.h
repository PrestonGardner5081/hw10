/*
 * FIFO.h
 *
 *  Created on: Feb 12, 2022
 *      Author: steveb
 */

#ifndef FIFO_H_
#define FIFO_H_

#ifndef ARRAYSIZE
#define ARRAYSIZE(X) (sizeof(X)/sizeof((X)[0]))
#endif /* ARRAYSIZE */

/* declare a FIFO
 * initialize like this:
 * struct my_FIFO_t fifo = {{}, 0, 0, PTHREAD_MUTEX_INITIALIZER};
 */
#define FIFO_TYPE(DATA_T, LENGTH, NAME_T) \
  struct NAME_T \
  { \
    volatile DATA_T   data[LENGTH]; \
    volatile uint32_t next_insert; \
    volatile uint32_t next_remove; \
    pthread_mutex_t   mutex; \
  }

/* insert and remove operations
 * check that there is enough room to insert and that there is data to remove before executing!
 */
#define FIFO_INSERT( FIFO, NEW_DATUM ) /* FIFO is a pointer, NEW_DATUM is a value */ \
    do \
    { \
      pthread_mutex_lock( &((FIFO)->mutex) ); \
      (FIFO)->data[(FIFO)->next_insert] = NEW_DATUM; \
      (FIFO)->next_insert = ((FIFO)->next_insert + 1) % ARRAYSIZE((FIFO)->data); \
      pthread_mutex_unlock( &((FIFO)->mutex) ); \
    } while (0)
#define FIFO_REMOVE( FIFO, OLD_DATUM ) /* FIFO and OLD_DATUM are pointers */ \
    do \
    { \
      pthread_mutex_lock( &((FIFO)->mutex) ); \
      *OLD_DATUM = (FIFO)->data[(FIFO)->next_remove]; \
      (FIFO)->next_remove = ((FIFO)->next_remove + 1) % ARRAYSIZE((FIFO)->data); \
      pthread_mutex_unlock( &((FIFO)->mutex) ); \
    } while (0)

/* full, empty checks and capacity calculations
 * FIFO should be locked for these operations, but they should be safe for architectures where 32-bit variable access is atomic (single-cycle)
 */
#define FIFO_EMPTY( FIFO )      ((FIFO)->next_insert == (FIFO)->next_remove)
#define FIFO_FULL( FIFO )       ((((FIFO)->next_insert + 1) % ARRAYSIZE((FIFO)->data)) == (FIFO)->next_remove)
#define FIFO_COUNT_USED( FIFO ) (((FIFO)->next_insert - (FIFO)->next_remove + ARRAYSIZE((FIFO)->data)) % ARRAYSIZE((FIFO)->data))
#define FIFO_COUNT_FREE( FIFO ) (ARRAYSIZE((FIFO)->data) - FIFO_COUNT_USED( FIFO ) - 1)

#endif /* FIFO_H_ */
