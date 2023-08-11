# Ringbuffer


## rbpush
> pushes data into the ring buffer, implemented as a circular FIFO buffer;

### Inputs
- ringbuffer_t \*buffer: buffer where the data will be pushed;
 - rbelement_t data: value that will be pushed in the ring buffer;

### Outputs
- rberror_t: whether the push operation was completed. By the nature of the circular buffer: if the buffer is full then the oldest value will be overwritten.
 



## rbpop
> pops the oldest value in the buffer

### Inputs
- ringbuffer_t \*buffer: buffer from which the element will be popped;
 - rbelement_t \*data: pointer to the variable that will hold the popped value;

### Outputs
- rberror_t: whether the popping procedure was concluded successfully.
 



## rbpeek
> it returns the oldest value in the buffer without removing it.

### Inputs
- ringbuffer \*buffer: buffer from which the element will be taken;
 - rbelement_t \*data: pointer to the variable that will hold the value;

### Outputs
- rberror_t: whether the operation was concluded successfully or not;
 



## rblast
> returns the most recent (last) element of the buffer without removing it;

### Inputs
- ringbuffer_t \*buffer: buffer from which the value will be taken;
 - rbelement_t \*data: pointer to the variable that will be taken from the buffer;

### Outputs
- rbelement_t: whether the operation was concluded successfully.
 



## rbclear
> method that clears the buffer (can be used also for initialization)

### Inputs
- ringbuffer_t \*buffer: buffer to clear;

### Outputs
- void;
 


---

generated with [EasyGen](http://easygen.altervista.org/) - [On Github](https://github.com/dede-amdp/easygen).