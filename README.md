# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program.

A C++ implementation of a PID controller
![Alt Text](./images/sample_drive.gif)

---
## Description
PID = Proportional, integral, Derivative. This is a control system designed to control a parameter in a system based on a measureable outcome.
The implementation of the controller is quite straight forward. There are three parts to the cotroller:
* P - Proportional: change the input parameter in proportion to the output error (correction = -Kp * Cte). The controller reacts to the measured error and try to minimize it. This part of the controller is the part whis reacts in the fastest way to new and unexpected changes when they occur (such as sharp turns), but it also can have the tendency to overshoot the target.
* I - Intgral: change the input parameter in proportion to the sum of all measured errors (correction = -Ki * sum(Cte)). This part will take care of biases in our system that create long term errors. It can take some time to stabilize in some cases.
* D - Derivative: change the input parameter in proportion to the derivative of the output error (correction = -Kd * d(Cte)/dt). prevents the controller from overshooting when reducing the error, and also reduces the system oscilations.

all the parts of the controller are used to determine the input parameter (input=Tau*(P + I + D)).

The implementation of the controller is pretty simple the tricky part was to find good controller coefficients (Kp, Ki, Kd). I did it in a good old trial and error process :) 

I have implemented 2 controlles: 
* Steering controller - based on the error measured between current car location, and the desired path (given). I have found that when the speed increases, the controller becomes more sensitive, and the car starts to wobble. so, I made the total proportion parameter (Tau) to be a function of the speed. and it become smaller when the speed increases, to avoid situation when the car does movements which are too sharp.
* Speed controller - based on the difference between current speed, and the desired speed. pretty straight forward.

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

Fellow students have put together a guide to Windows set-up for the project [here](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/files/Kidnapped_Vehicle_Windows_Setup.pdf) if the environment you have set up for the Sensor Fusion projects does not work for this project. There's also an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3).

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)
