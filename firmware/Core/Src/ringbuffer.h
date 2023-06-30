#include<stdint.h>

#ifndef RB_DEF
#define RB_DEF

#define RBUF_SZ 10

typedef uint8_t rberror_t; // to show errors
typedef double rbelement_t; // element of the buffer

/*
#@
@name: struct ringbuffer aka ringbuffer_t
@brief: simple ring/circular buffer structure
@inputs: 
- uint8_t tail: index of where the element will be inserted next;
- uint8_t head: index of where the oldest element is in the buffer;
- uint8_t length: number of elements in the buffer;
- rbelement buffer[RBUF_SZ]: array that holds the elements of the buffer;
@#
*/
typedef struct ringbuffer {
    uint8_t tail;                               /* where data will be pushed */
    uint8_t head;                               /* where data will be popped */
    uint8_t length;                             /* number of elements */
    rbelement_t buffer[RBUF_SZ];                /* actual buffer */
} ringbuffer_t; /* 24 bits + RBUF_SZ*64 bits -> with RBUF_SZ = 10: 664 bits ~> 83 B of data */

rberror_t rbpush(ringbuffer_t *buffer, rbelement_t data);
rberror_t rbpop(ringbuffer_t *buffer, rbelement_t *data);
rberror_t rbpeek(ringbuffer_t *buffer, rbelement_t *data);
rberror_t rblast(ringbuffer_t *buffer, rbelement_t *data);
rberror_t rbget(ringbuffer_t *buffer, uint8_t i, rbelement_t *element);
void rbclear(ringbuffer_t *buffer);

#endif
