# Unscented Kalman Filter Project Starter Code
Self-Driving Car Engineer Nanodegree Program

In this project utilize an Unscented Kalman Filter to estimate the state of a moving object of interest with noisy lidar and radar measurements. Passing the project requires obtaining RMSE values that are lower that the tolerance outlined in the project rubric.

## Installation / Requirements
This project can be either used with *.csv text files or with a visualization in a simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases).

This repository includes two files that can be used to set up and intall [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO. Please see [this concept in the classroom](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/16cf4a78-4fc7-49e1-8621-3450ca938b77) for the required version and installation scripts.

### Other Important Dependencies
* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./UnscentedKF` Previous versions use i/o from text files.  The current state uses i/o
from the simulator.


## Running the program

### Program documentation
```
Help:
  --verbose        <0|1>:    Turn on verbose output, default: 0
  --use_laser      <0|1>:    Turn on or off laser measurements, default: 1
  --use_radar      <0|1>:    Turn on or off radar measurements, default: 1
  --std_a          <num>:    Standard deviation for linear acceleration noise, default: 0.5
  --std_yawdd      <num>:    Standard deviation for angular acceleration noise, default: 0.2
  --use_simulator  <0|1>:    Use simulator for input and output or instead an input output csv file, default: 1
  --input_file     <path>:   Path to input csv file (only possible when simulator mode is not set), default: ../data/obj_pose-laser-radar-synthetic-input.txt
  --output_file    <path>:   Path to output csv file (only possible when simulator mode is not set), default: ../data/obj_pose-fused-output.txt
  --help:                    Show help
```

The program supports two modes: simulator backend (`--use_simulator=1`) and data files as in- and output (`--use_simulator=0`)

### Simulator mode
When using the simulator mode the unity based udacity simulator mentioned above has to be started and the first menu option selected. Then the UKF can be started by invoking:

    ./UnscentedKF --use_simulator=1


### Data file mode

Start the program using udacity provided `data/obj_pose-laser-radar-synthetic-input.txt` file as input and storing the filtered states, rmse and ground truth for evaluation and visualization in `data/obj_pose-fused-output-all.txt`:

    ./UnscentedKF --use_simulator=0 --input_file=../data/obj_pose-laser-radar-synthetic-input.txt --output_file=../data/obj_pose-fused-output-all.txt

Using the same input and output files but deactivating the laser scanner and only using the radar data for filtering (can't be called sensor fusion anymore, I guess):

    ./UnscentedKF --use_simulator=0 --use_laser=0 --input_file=../data/obj_pose-laser-radar-synthetic-input.txt --output_file=../data/obj_pose-fused-output-all.txt


## Results

The following plot shows how the UFK filter compares using data from both lidar and radar sensor and having ony either as input.

![](figures/position.png)

As cartesian positions are directly observable by the lidar sensor the accuracy is pretty good.
It will be more interesting to see how well the filter performes at predicting hidden states such as cartesian velocities. In the following plot we look at how well cartesian velocities are being predicted, just in the same manner as above, one plot is with both sensor inputs and in the other plots only either sensor being used.

![](figures/velocity.png)


In this plot we see the RMSE in cartesian position and velocity over time. It's interesting to see how the error in the velocities seems to have asymptotic behaviour towards the end of the dataset.

![](figures/state_rmse.png)
