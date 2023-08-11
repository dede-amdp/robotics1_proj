# Ringbuffer


## struct ringbuffer aka ringbuffer_t
> simple ring/circular buffer structure

### Inputs
- uint8_t tail: index of where the element will be inserted next;
 - uint8_t head: index of where the oldest element is in the buffer;
 - uint8_t length: number of elements in the buffer;
 - rbelement buffer[RBUF_SZ]: array that holds the elements of the buffer;

 


---

generated with [EasyGen](http://easygen.altervista.org/) - [On Github](https://github.com/dede-amdp/easygen).