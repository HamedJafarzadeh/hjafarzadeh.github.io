---
layout: post
title:  Isolated battery voltage measurements
categories: robotics
---

# Isolated battery voltage measurements

## High percision accuracy solutions
### First solution
Broadcom ACPL-C87x **Precision Optically Isolated Voltage Sensor**
![](https://github.com/HamedJafarzadeh/HamedJafarzadeh.github.io/blob/master/assets/ACPL-C87x_Circuits.png?raw=true)

- ACPL-87A or ACPL-87B or ACPL-870 are not different in functioning

### Second solution
- Use a microcontroller on one side and measure the voltage and send the data through a isolated communication link such as CAN or RS485

## Low percision accuracy solution
### Using Voltage level indicator ICs
We can use these Voltage level indicators as follows

![](https://www.electroschematics.com/wp-content/uploads/2014/07/battery-voltage-level-indicator.png)

and then instead of normal LEDs, we can connect them to optocouplers and get the voltage level on the other side.

### Using Zener and Optocouplers
We can use Zeners as follows for determining the voltage level
![](http://i0.wp.com/zaidpirwani.com/wp-content/uploads/2011/01/indicator-15v.png)

and then instead of normal LEDs, we can connect them to optocouplers and get the voltage level on the other side.
