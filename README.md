# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

## Implementation
---
### Description
This project contains my solution to implementation the Model Predictive Control (MPC) to drive the car around the track.

[![ScreenShot](./build/2017-10-03-11.29.32.png)](https://youtu.be/EKf2ZoMHnL0)

### The Model
- Student describes their model in detail. This includes the state, actuators and update equations.

MPC model is simulating actuator inputs to predict trajectory and select trajectory according to minimum cost. The model used in this project is Kinematic Bicycle Model which consists of following states:

`x`: The x position of the vehicle.<br>
`y`: The y position of the vehicle.<br>
`psi`: The orientation of the vehicle.<br>
`v`: The current velocity.<br>
`cte`: The Cross Track Error.<br>
`epsi`: The error in orientation.<br>

The update equations are:
```
fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
fg[1 + psi_start + t] = psi1 - (psi0 - v0/Lf * delta * dt);
fg[1 + v_start + t] = v1 - (v0 + a * dt);
fg[1 + cte_start + t] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
fg[1 + epsi_start + t] = epsi1 - ((psi0 - psides0) - v0/Lf * delta * dt);
```
### Timestep Length and Elapsed Duration(N & dt)
- Student discusses the reasoning behind the chosen N (timestep length) and dt (elapsed duration between timesteps) values. Additionally the student details the previous values tried.

Firstly, according to the learnt from lesson, I chosen with N=25 and dt=0.05. The car was swinging sharply and out of the road very fast. So I choose N=20 and dt=0.05, the car was still swinging and out of the road,it's easy to miscalculate, I chosen N=10, dt=0.05, decrease the control input helps the car drivers longer than before, but still swinging was not controlled. I chosen to increase the dt=0.2, N=10, this reduced the car drive swinging, but the calculation is a little slower, finally, I chose the N = 10, dt = 0.15 to calculate the trajectory. The car reached max speed of 100 MPH on the road. 
Here are the conclusion:
- Smaller dt has finer resolution but require high N for given horizon (N*dt).
- Larger N than N=10 takes longer computational time which effectively increase the latency.
- Smaller value thant N=10 is not enough to caculate the trajectory.

### Polynomial Fitting and MPC Preprocessing
- A polynomial is fitted to waypoints.

The waypoints from simulator are global coordinate system, but we have to transform them because all computations are performed in the vehicle coordinate system.
```
double x = (ptsx[i] - px) * cos(psi) + (ptsy[i] - py) * sin(psi);
double y = -(ptsx[i] - px) * sin(psi) + (ptsy[i] - py) * cos(psi);
```
### Latency
- The student implements Model Predictive Control that handles a 100 millisecond latency. Student provides details on how they deal with latency.

I chosen to update the car's state after polynomial fitting with 2 steps, first to caculate the current time t=0, here according to car's coordinate px=0, py=0, psi=0, and caculate cte and epsi in step1, in step2 I caculate the prediction of all states for t=lantency.
The state is predicted with latency 100ms and then process the Solver, the latency in the module is proccessed as below:
```
// Step1: At current time t=0, the car's state are px=0, py=0, psi=0
auto coeffs = polyfit(waypoints_x, waypoints_y, 3);
// cte = desired_y - actual_y
//     = polyeval(coeffs, px) - py
double cte = polyeval(coeffs, 0);
// epsi = actual_psi-desired_psi
//      = psi - atan(coeffs[1] + 2*coeffs[2]*px + 3*coeffs[3]*px*px)
double epsi = -atan(coeffs[1]);
          
double latency = 0.1;
double Lf = 2.67;

double delta = j[1]["steering_angle"];
double a = j[1]["throttle"];
          
// Step2: Predict states for t = latency, due to car coordinate system, 
// px0=py0=psi0=0
// Optional to convert miles per hour to meter per second
v *= 0.44704;
px = 0 + v * cos(-delta) * latency; 
py = 0 + v * sin(-delta) * latency;
psi = 0 - v * delta * latency / Lf;
v = v + a * latency;
cte = polyeval(coeffs, px) - 0;  // since py0=0
epsi = atan(coeffs[1]+2*coeffs[2]*px + 3*coeffs[3]*px*px);

Eigen::VectorXd state_vector(6);
state_vector << px, py, psi, v, cte, epsi;

auto results = mpc.Solve(state_vector, coeffs);
```

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
* Fortran Compiler
  * Mac: `brew install gcc` (might not be required)
  * Linux: `sudo apt-get install gfortran`. Additionall you have also have to install gcc and g++, `sudo apt-get install gcc g++`. Look in [this Dockerfile](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/Dockerfile) for more info.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * If challenges to installation are encountered (install script fails).  Please review this thread for tips on installing Ipopt.
  * Mac: `brew install ipopt`
       +  Some Mac users have experienced the following error:
       ```
       Listening to port 4567
       Connected!!!
       mpc(4561,0x7ffff1eed3c0) malloc: *** error for object 0x7f911e007600: incorrect checksum for freed object
       - object was probably modified after being freed.
       *** set a breakpoint in malloc_error_break to debug
       ```
       This error has been resolved by updrading ipopt with
       ```brew upgrade ipopt --with-openblas```
       per this [forum post](https://discussions.udacity.com/t/incorrect-checksum-for-freed-object/313433/19).
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/).
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `sudo bash install_ipopt.sh Ipopt-3.12.1`. 
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions


1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.)
4.  Tips for setting up your environment are available [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).

