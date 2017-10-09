# CarND-Controls-MPC


**The Model:**

Model State variables:

- x position of car
- y position of car
- Orientation of car (psi)
- Velocity of car
- Crosstrack error 
- Orientation error

Actuators:
- Vehicle Throttle
- Vehicle Steering

Update equations:
- Standard kinematic motion model is used and updated in the simulation

x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt

y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt

psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt

v_[t+1] = v[t] + a[t] * dt

cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt

epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt

**Timestep Len and Elapsed Duration (N & dt)**

- I experimentally chose the timestep length (N) and (dt) to balance future estimation vs computational load. The values I chose provided enough future-expectation while also allowing the simulation to run in real time. 

Increasing the dt while holding N constant increased the resolution of controller but inversely decreased the reaction time of the controller because of increased computational overhead.

Decreasing dt while holding N constant increased the look ahead amount. Having this too far ahead resulted in erratic performance and poor short term performance. 

Increasing N to a high value while holding dt constant (at .1) resulted in extremely erratic behaviour. It seems that when the foresight distance gets too long the controller has a lot of trouble performing smoothly.

Decreasing N while keeping dt constant at (0.1) resulted in the controller having enough foresight. Thus when encountering a turn the vehicle had difficulty remaining on the road. 

Thus the solution was to find the sweetspot of resolution and foresight where the controller could deal with tight turns but also be reactive in terms of computational load. 

N=20 and dt = 0.1 was found to work based on empirical analysis.

**Polynomial fitting and Preprocessing**

- Velocity was converted from MPH to meters per second
- World coordinate frame measurements were converted to car coordinate frame.

**Latency**

- Latency is handled by setting initial state of car as predicted state 100ms ahead. 

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

# P5-MPC
