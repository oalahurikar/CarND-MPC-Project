## Model Predictive Control
The goal of Model Predictive Control in this projet is to optimize the control inputs: Steering angle and throttle. Model predictive controllers rely on dynamic models of the process. In this project used kinematic bicycle model shown below.

      x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
      y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
      psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
      v_[t+1] = v[t] + a[t] * dt
      cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
      epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt

where:
x_,y_ is position of the vehicle.
psi is orientation of the vehicle.
v is vehicle velocity.
**delta** and **a** are actuators like steering angle and throttle; 
Lf is the distance between the front of the vehicle and its center of gravity;
cte is cross-track error (the difference between the center of road line and the current vehicle position y).
epsi is the orientation error.

The main advantage of MPC is it allows the controls variable in current timeslot to be optimized, while keeping future timeslots in account.

## Latency
A contributing factor to latency is actuator dynamics. Latency in controls is time elapsed between when you command a steering angle to when that angle is actually achieved. Thus, MPC can deal with latency much more effectively, by explicitly taking it into account, than a PID controller. In this model MPC handles 100 miliseconds latency without running car out off road.

## Cross-track error `(CTE)`, orientation error `(EPSI)` and  Number of Timesteps `(T)`
MPC control receives the trajectory from simulator as an array of waypoints `ptsx` and `ptsy` in the World (map) coordinate space. `(CTE)` and the `(EPSI)` are calculated in the vehicle coordinate space. To do this, the waypoints are transformed to the vehicle coordinate space, then MPC control approximates the trajectory with 3rd order polynomial and does prediction of `N` states with `N-1` actuator changes of the car using a prediction horizon `T`, that is a duration over which future predictions are made. `T` is the product of two other variables, `N` and `dt`, where `N` is the number of timesteps in the horizon and `dt` is how much time elapses between actuations.
I used timestep length N = 15, timestep frequency dt = 50 ms which resulted in succesful track completion. I experimented with N, dt values some showed erratic car behaviour and for some values car went off the road..

## A polynomial is fitted to waypoints.
MPC control receives the trajectory from simulator as an array of waypoints ptsx and ptsy in the World (map) coordinate space. The cross-track error (CTE) and the orientation error (EPSI) are calculated in the vehicle coordinate space. In order to transform the waypoints to the vehicle coordinate space from map coordinate space. The MPC control approximates the vehicle trajectory with 3rd order polynomial and does prediction of N states with N-1 actuator changes of the car using a prediction horizon T, that is a duration over which future predictions are made.
## Results
Curve Road | Straight Road
------------ | -------------
![alt_text-1](https://github.com/oalahurikar/CarND-MPC-Project/blob/master/MPC%20Pic/MPC%201.PNG) | ![alt_text-2](https://github.com/oalahurikar/CarND-MPC-Project/blob/master/MPC%20Pic/MPC%203.PNG)

