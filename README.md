# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

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


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Model  
I used a kinematic model.  

State:  
x,y -> coordinates of the car  
v -> velocity  
cte -> crosstrack error  
psi -> orientation angle  
epsi -> orientation angle error  

Actuator:  
a -> acceleration, between -1 and 1. A negative value means breaking, positive value is accellerating.  
delta -> steering angle, between -25 and 25.  

Equations:  
px_1 = px + v * cos(psi) * dt  
py_1 = py + v * sin(psi) ( dt)  
psi_1 = psi + v / Lf * (-delta) * dt  
v_1 = v + a * dt  
cte = cte - v * sin(epsi) * dt  
epsi = epsi +  v / Lf * (-delta) * dt  

Concerning the weights I use in the cost function, I found these by trying. I started from no weight at all, the car left the road. I then tried to put more weight on the elements depending on delta, it went way better but still was not able to finish a lap. I doubled the values and it worked :)  

## Timestep Length and Elapsed Duration (N & dt)

I choosed N = 10 and dt = 0.1. I went for these values because predicting for the next second seemed a good idea to me. I tried to use a higher N but my computer was too slow (so a lower dt is not a possibility for me). I tried with N = 15 and dt = 0.15, the tires left the road from time to time but the car was able to finish a lap.
  
  
## Polynomial Fitting and MPC Preprocessing

The waypoints are transformed in the vehicle perspective (line 117-122 main.cpp), it makes the vehicle position to be 0,0 and the orientation angle too. This makes the polynomial fitting easier.

## Model Predictive Control with Latency

To deal with the latency, I added it to each state (line 108-112) term and used these new values to make predictions.
