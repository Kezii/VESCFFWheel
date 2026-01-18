# VESCFFWheel

Use an RP2040 to build an usb force feedback direct drive steering wheel for racing simulators, supporting multiple turns and everything


## How to use

1. Get a VESC controller with the means to attach a rotor angle sensor (like the as5047) (follow online tutorials for that)
1. Configure VESC (through vesc_tool) to drive the motor correctly with FOC and everything, configure / calibrate the sensor
1. You should now be able to drive the motor at a specific angle, and drive it in rotation setting the duty cycle
1. Flash this project on an RP2040
1. Connect the RP2040 and the VESC with the uart, TX = PIN4, RX = PIN5
1. Connect the usb port
1. It should work as a ffb controller driving the vesc
