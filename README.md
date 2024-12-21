# Source code of the balancer bot

WARNING: the code is quite outdated, includes old libraries and depends on old
         Arduino platform. 

## Hardware
   - Raspberry Pi Pico
   - MPU6050 6-axis motion sensor
   - L298N DC motors controller
   - JDY-31 bluetooth module

## ** Requires deprecated Arduino MBED OS boards support
 
## Requires libraries installation in Arduino IDE
   - Use MPU6050 lib to get hardware DMP readings from MPU6050
   - Use RP2040_PWM lib to leverage PiPico hardware PWM generators

## Includes third party software modules
   - MPU6050 by Jeff Rowberg <jeff@rowberg.net>
   - I2Cdev lib by Jeff Rowberg <jeff@rowberg.net>
   - RP2040_PWM by Khoi Hoang
   - PID based on PID by Bradley J. Snyder <snyder.bradleyj@gmail.com>

## Remote control
   - Uses the Android app as a bluetooth remote, sources can be found here: 
   - https:github.com/poconoco/bt-remote/