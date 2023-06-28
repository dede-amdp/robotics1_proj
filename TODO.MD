<style>
    *{
        font-family: 'Fira Code', monospace;
    } 
</style>

# TODO:
* [ ] add logging feature - firmware side;
  * Implement a method that sends via serial the actual motor position, speed and acceleration;
  * the message should have ALWAYS the same size, to have better control over the transmission buffer;
* [ ] add serial communication to firmware -- partially implemented
  * [x] reading from serial method TO GET REFERENCE VALUES;
  * [ ] other reading methods (other command types should be defined);
    * after specifying other possible uses for serial communication (example, starting the homing procedure), for each use a unique 3 char long ID should be defined (example: homing procedure 'hmp');
    * in the reading callback, an else-if should be added for each new command that will handle the data and populate variables / start procedures as needed;
    * other functions SHOULD NOT BE CALLED within the callback unless strictly necessary: this is to avoid the delay brought by context switching;
    * remember: callbacks should be as fast as possible;
  * [ ] writing to serial (for example -> logging and fault detection);
    * the transmission should be implemented via DMA, to be sure that execution will not be halted while the transmission is done;
    * the callback will be left alone (unless the need to use it rises);
* [ ] implement motor control method -> taken u (the speed input) returned by the controller method, use it to control the motors;
* [x] implement method to read from encoders;
  * using timers, decode the encoder output signal to find the position: from the position, also compute speed and acceleration;
  * to compute the speed, use the interpolation method that the professor showed us during the lectures which uses past values of position and time instants to find the speed;
  * a method to approximated acceleration should be found and implemented;
  * The actual position, speed and acc. should be saved in the global object 'manip' in the correct struct members (check custom.h)
* [ ] implement limit switch reading;
  * it should be connected to an interrupt, this means that it should have a callback;
  * the callback should (for now) stop everything to avoid damaging the manipulator;
* [ ] implement homing procedure;
  * both the arms should move in the same direction until they reach the limit switches (both of them);
  * the encoder readings will be set as initial offsets of the motor, to reset it;
  * both the arms should then extend to reach the 0 rad,0 rad position; 
* [ ] REMEMBER PENUP;
  * the manip object also holds data for the PENUP value, when penup is 1 the motor control method (check previous points) should raise the pen, when penup is 0 the pen should be lowered;
  * the end effector motor should not activate if the penup does not change;
* [ ] !! Refactor and documentation;

## Docs
Documenting code is very important: it helps us remember what the code does after some days without touching it and it helps the professor understand that the written code is not just a mess of letters and numbers but something meaningful: COMMENT THE CODE when needed (do not overdo it otherwise it will be messy). Use ALWAYS multiline code, not inline ones. Write a "comment header" that explains what the function does, what each input and output does and how, in the following structure

```c
/*
#@
@name: function_name
@brief: brief -> summary of what the function does
@notes: notes -> detailed description (if the function is simple, it can be completely omitted)
@inputs: 
- type input1: description;
- type input2: description;
- ...
@outputs: 
- type: description.
@#
*/
```

example:
```c
/*
#@
@name: dot
@brief: computes the dot product between two matrices, A and B, represented in vector form;
@inputs: 
- double *A: pointer to a vector of doubles of size nA*mA, which represents the first nAxmA matrix;
- uint8_t nA: number of rows of matrix A;
- uint8_t mA: number of columns of matrix A;
- double *B: pointer to a vector of doubles of size nB*mB, which represents the second nBxmB matrix;
- uint8_t nB: number of rows of matrix B;
- uint8_t mB: number of columns of matrix B;
-double *C: pointer to a vector of doubles of size nA*mB, which represents the resulting nAxmB matrix -> if the operation cannot be done, it will be NULL;
@outputs: 
- uint8_t: 0 or 1 that shows whether the operation completed successfully or not.
@#
*/
uint8_t dot(double *A, uint8_t nA, uint8_t mA, double* B, uint8_t nB, uint8_t mB, double* C){
    //...
}
```

## Refactoring
Refactoring code avoids having junk code --> variables that do not do anything or that hold values that will be changed without any use OR functions that are useless that get called when not needed or do not get called ever -> also leftovers from debug;
Refactoring removes these problems and leaves only the strictly necessary, making it easier to read and understand.
To help out with refactoring, use the "// DEBUG" commend near a line of code that was added for debug and use "// START DEBUG //END DEBUG" limiters before and after the debug section, so that by searching for "debug" in the code you will be able to find each line of code that was added for debug! 



!! Addendum: here's an easy to copy documentation comment:
```c
/*
#@
@name: name
@brief: brief
@notes: notes
@inputs: inputs
@outputs: outputs
@#
*/
```