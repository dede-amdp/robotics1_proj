#include<stdint.h>
#include "ringbuffer.h"

/*
#@
@name: rbpush
@brief: pushes data into the ring buffer, implemented as a circular FIFO buffer;
@inputs: 
- ringbuffer_t \*buffer: buffer where the data will be pushed;
- rbelement_t data: value that will be pushed in the ring buffer;
@outputs: 
- rberror_t: whether the push operation was completed. By the nature of the circular buffer: if the buffer is full then the oldest value will be overwritten.
@#
*/
rberror_t rbpush(ringbuffer_t *buffer, rbelement_t data){
    buffer->buffer[buffer->tail] = data;
    buffer->tail++;
    buffer->tail %= RBUF_SZ; /* avoid that tail goes outside the boundaries of the buffer */
    /* the buffer can only hold RBUF_SZ elements, so old ones will be overwritten */
    if(buffer->length == RBUF_SZ){
        /* overwriting data: also move head forward */
        buffer->head++;
        buffer->head %= RBUF_SZ; /* avoid that head goes outside the boundaries of the buffer */
    }else{
        buffer->length++;
    }
    return 1;
}

/*
#@
@name: rbpop
@brief: pops the oldest value in the buffer
@inputs: 
- ringbuffer_t \*buffer: buffer from which the element will be popped;
- rbelement_t \*data: pointer to the variable that will hold the popped value;
@outputs: 
- rberror_t: whether the popping procedure was concluded successfully.
@#
*/
rberror_t rbpop(ringbuffer_t *buffer, rbelement_t *data){
    if(buffer->length == 0){
        *data = buffer->buffer[buffer->head]; /* avoids random values in data */
        return 0; /* pop operation could not be completed because the buffer is empty */
    }
    *data = buffer->buffer[buffer->head];
    buffer->head++;
    buffer->head %= RBUF_SZ;
    buffer->length--;
    return 1;
}

/*
#@
@name: rbpeek
@brief: it returns the oldest value in the buffer without removing it.
@inputs: 
- ringbuffer \*buffer: buffer from which the element will be taken;
- rbelement_t \*data: pointer to the variable that will hold the value;
@outputs: 
- rberror_t: whether the operation was concluded successfully or not;
@#
*/
rberror_t rbpeek(ringbuffer_t *buffer, rbelement_t *data){
    if(buffer->length == 0){
    	*data = buffer->buffer[buffer->head]; /* avoids having random values as output */
        return 0; /* peek operation could not be completed because the buffer is empty */
    }
    *data = buffer->buffer[buffer->head];
    return 1;
}

/*
#@
@name: rblast
@brief: returns the most recent (last) element of the buffer without removing it;
@inputs: 
- ringbuffer_t \*buffer: buffer from which the value will be taken;
- rbelement_t \*data: pointer to the variable that will be taken from the buffer;
@outputs: 
- rbelement_t: whether the operation was concluded successfully.
@#
*/
rberror_t rblast(ringbuffer_t *buffer, rbelement_t *data){
    if(buffer->length == 0){
    	*data = buffer->buffer[buffer->head]; /* avoids having random values as output */
        return 0; // operation failed
    }
    // uint8_t index = (uint8_t) ((buffer->tail-1+RBUF_SZ)%RBUF_SZ);
    int8_t index = buffer->tail-1;
    if(index < 0){
        index += RBUF_SZ;
    }
    *data = buffer->buffer[(uint8_t) index];
    return 1;
}


rberror_t rbget(ringbuffer_t *buffer, int8_t i, rbelement_t *data){
    if(i < 0 || i >= buffer->length){
        /* out of bounds */
        *data =  buffer->buffer[buffer->head];
        return 0;
    }
    uint8_t index = (uint8_t) ((buffer->head+i) % RBUF_SZ);
    *data = buffer->buffer[index];
    return 1;
}

/*
#@
@name: rbclear
@brief: method that clears the buffer (can be used also for initialization)
@inputs: 
- ringbuffer_t \*buffer: buffer to clear;
@outputs: 
- void;
@#
*/
void rbclear(ringbuffer_t *buffer){
    uint8_t i = 0;
    for(i = 0; i < RBUF_SZ; i++){
        buffer->buffer[i] = 0;
    }
    buffer->length = 0;
    buffer->head = 0;
    buffer->tail = 0;
}
