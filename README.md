# Model Predictive Control Project
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

Overview
---
This repository contains my implementation of the odel Predictive Control project (Term 2 - Project 5) in Udacity's Self-Driving Car Nanodegree Program.
The goal of the project is to implement a PID controller to race around the lake track in the driving simulator.

## Implementation 

One PID controller is used to control the throttle of the vehicle in order to achieve a constant speed of 20 km/s. 
The parameters of the throttle PID controller are set to: Kp=0.05, Ki = 0.001 and Kd = 0.

Another PID controller is used to control the steering. 
Manual parameter tuning was used to get a feeling for the impact of the single parameters. 
Afterwards, a twiddle algorithm was used to automatically fine tune the parameters. 
The algorithm is implemented similar to the version showin in the class. 
For every set of parameters, the simulator is first reset and then the code is run for the first 500 timestamps of the simulator.
Using this procedure, the total error of more than 200 parameter configurations was recorded.
The parameter-set with the minimum error was: Kp=0.813491, Ki = 0.00215393 and Kd = 3.24184.
Thus, this combination was used for the submission of the project. 

The implementation of the twiddle algorithm can be found in `src/main_twiddle.cpp`

## Demo

The chosen parameter configuration resulted in the following output: 

<img src="./media/video.gif" />  

The vehicle stays on the track during the whole circuit. 


## Running the Code
This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and intall uWebSocketIO for either Linux or Mac systems. For windows you can use either Docker, VMware, or even Windows 10 Bash on Ubuntu to install uWebSocketIO.

Once the install for uWebSocketIO is complete, the main program can be built and ran by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./mpc


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
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.



