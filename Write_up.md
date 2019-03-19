# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
## Overview
### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic

## Project video
![](img/pic1.png)

## Project Specification
### Compilation
1. The updated file: 'main.cpp'
2. Folder: 'build'

### Valid Trajectories
1. The car is able to drive at least 4.32 miles without incident. (o)
2. The car drives according to the speed limit. (o)
   1. I set _'ref_vel = 49.5'_ to control the fastest speed. 
   2. If the front car is too close (<30), the car should slow down.
3. Max Acceleration and Jerk are not Exceeded. (o)
4. Car does not have collisions.(o)
   1. In previous experiment, the crash sometimes happened in lane changing. I set  _'if(((check_car_s_nearby-car_s)<20) && ((check_car_s_nearby-car_s)>-20))'_,_'{lane_safe = false;}'_ for preventing that the car hit the car behind it.  
5. The car stays in its lane, except for the time between changing lanes.(o)
   1. 
6. The car is able to change lanes(o)

