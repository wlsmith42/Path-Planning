#CarND-Path-Planning-Project
####Self-Driving Car Engineer Nanodegree Program
---

[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

The goals / steps of this project are the following:

* Safely navigate a vehicle around a highway with other traffic driving +/-10 mph of the 50 mph speed limit.
* The vehicle should maintain a speed as close as possible to the 50 mph speed limit, which includes passing slower moving vehicles
* The vehicle should also remain within the marked road lanes at all times and not collide with another vehicle.
* Finally, the car must complete 1 loop around the 6,946 m highway without experiencing a total acceleration over 10 m/s^s and jerk that is greater than 10 m/s^3.

[//]: # (Image References)

[image1]: ./images/boxed_in1.gif "box1"
[image2]: ./images/boxed_in2.gif "box2"
[image3]: ./images/car_ahead_lane_switch.gif "switch"
[image4]: ./images/pass.gif "pass"

---


   
### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab] (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).  

To run the simulator on Mac/Linux, first make the binary file executable with the following command:
```shell
sudo chmod u+x {simulator_file_name}
```

 
#### The map of the highway is in data/highway_map.txt
The car's localization and sensor fusion data will be provided along with a sparse map list of waypoints around the highway. Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Run the build script using `./build.sh`.
4. Run the program: `./run.sh`.
5. Open the simulator and select "Project 1: Path Planning"

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Details

1. The car uses a perfect controller and will visit every (x,y) point it receives in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner receives should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually it's not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Results

The following gifs highlight certain scenarios and how my implementation of the path planner project handles those challenges. For more information on how the path planner works, check out the comments in `main.cpp`.

---

This first gif shows a clean pass, illustrating the path planners ability to keep the vehicle in the lane lines as well as intelligently navigate around other vehicles.

![alt text][image4]

---

This next gif shows how the vehicle handles a lane change when the car in front of it also makes a lane change.

![alt text][image3]

---
These last two gifs show how the vehicle reacts when it is boxed in and cannot pass the slower vehicle. Ultimately, it is forced to wait, but when there is enough space the pass is completed.

![alt text][image1]

![alt text][image2]

---

## Limitations

Although this path planner works relatively well, there is still a lot of room for improvement in the future. While completing this project I learned the following shortcomings of my path planner design.

* When boxed in, the vehicle constantly accelerates then decelerates resulting in a jumpy motion. To solve this, I would calculate the speed of the vehicle in front and set my vehicles speed to match rather than braking when there is an obstacle and accelerating to full speed otherwise.

* The car will always pass to the left if available, but in some situations a pass to the right could avoid being blocked by slower vehicles. While passing on the right is not the best practice, it is still worth looking into so that the vehicle can generate multiple paths that reach further out so avoid such circumstances.




## Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)


