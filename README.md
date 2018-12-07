# CarND-Controls-MPC
*By Aravind Pillarisetti*

## The Model
The kinematic model is used to model the vehicle dynamics. The model has the following actuator inputs
```
steer - steering angle (also called delta)
throttle - acceleration/brake of the vehicle (also called a)
```

The model maintains the following states
```
x - the x position of the vehicle in car co-ordinates
y - the y position of the vehicle in car co-ordinates
psi - the orientation of the vehicle in radians
v - the velocity of the vehicle
```

The update equations of the state for a elapsed time of dT ate

```
x[t + 1] = x[t] + v[t]*cos(psi[t])*dT
y[t + 1] = y[t] + v[t]*sin(psi[t])*dT
psi[t+1] = psi[t] + v[t]*delta[t]*dT/Lf
v[t + 1] = v[t] + a[t]*dT;
```

In addition, there are 2 errors - cross track error (cte) and orientation error (epsi).
The errors are also captured as states. The update equations for the error states are
```
cte[t+1] = cte[t] + v[t]*sin(epsi[t])*dT
where
cte[t] = f(x[t]) - y[t]
and
epsi[t+1] = epsi[t] + v[t]*delta[t]*dT/Lf
where
epsi[t] = psi[t] - psides[t]
```

## Cost function and optimization
The objective is to find an optimal steering angle and acceleration such that it minimizes the cost function of states.
As per the lecture, the cost function consists of following parts
1. Squares of cte and epsi. 
2. Squares of the actuator inputs - steering angle and throttle
3. Squares of difference between consecutive actuator inputs
Each squares need to be appropriately multiplied by a factor/weight to get best performance.
I started with just the factor of 500 from the lecture but this led to the simulator vehicle wobbling and crashing.
I experimented with various weights and arrived at the best after multiple iterations. The iterations also helped me
understand how each of the cost function contributors impact the MPC performance.
My final weights are reflected in the code (in the FG_eval class operator() function)

## N & dT
The number of points (N) and elapsed time (dT) define the prediction horizon (T). As such, N & dT impacts the MPC performance. 
It is recommended that N and T should be maximized for best performance. At the same time, a much larger N slows down the
solver performance. I started with N=10 and dT=0.1 to give a 1 second horizon. This worked ok when driving at slower speeds.
However, at higher speeds, I had to increase it to N=20 and dT=0.05 to give the same 1 second horizon.
My N and dT are represented as globals in MPC.cpp

## Waypoints and polynomials
The points provided by the simulator to the main are transformed to points in the car co-ordinate system.
Also, the velocity is converted from mph to m/s (SI).

To handle latency, the points are updated before the transformation to the car system.
I used a latency of 150 ms based on review suggestion. Because of the latency all the waypoints were updated. The calculation was based on the following update

```
px  += v*cos(psi)*latency;
py  += v*sin(psi)*latency;
psi -= v*latency*delta/2.67;
v   += accel*latency;
```


With the waypoints, a 3rd order polynomial is used to compute the co-efficients.
The coefficients are used to calculate the cte and epsi states.

This code is reflected in main.cpp onMessage function.

## Model predictive control

With the computed cte, and epsi as well as the transformed points (with latency), we set the states

```
Eigen::VectorXd mpcStates(6);
mpcStates << 0.0, 0.0, 0.0, v, cte, epsi;
```

and with the coefficients are used to update the MPC
```
// Do model predictive control and get vars
mpcVars = mpc.Solve(mpcStates, coeffs);
```

---

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


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Build with Docker-Compose
The docker-compose can run the project into a container
and exposes the port required by the simulator to run.

1. Clone this repo.
2. Build image: `docker-compose build`
3. Run Container: `docker-compose up`
4. On code changes repeat steps 2 and 3.

## Tips

1. The MPC is recommended to be tested on examples to see if implementation behaves as desired. One possible example
is the vehicle offset of a straight line (reference). If the MPC implementation is correct, it tracks the reference line after some timesteps(not too many).
2. The `lake_track_waypoints.csv` file has waypoints of the lake track. This could fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.)
4.  Tips for setting up your environment are available [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)
5. **VM Latency:** Some students have reported differences in behavior using VM's ostensibly a result of latency.  Please let us know if issues arise as a result of a VM environment.

## Editor Settings

We have kept editor configuration files out of this repo to
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
agnostic as possible. We omitted IDE profiles to ensure
students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. Most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio and develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).
