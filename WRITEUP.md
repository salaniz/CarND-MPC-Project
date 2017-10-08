# Reflection of Model Predictive Control Project

## The Model
The model's state is composed of  
* `px`: the vehicle's x-coordinate in its own coordinate system (vehicle's facing direction),
* `py`: the vehicle's y-coordinate in its own coordinate system (left from vehicle's point of view),
* `psi`: the vehicle's orientation in its own coordinate system (initially 0),
* `v`: the vehicle's velocity,
* `cte`: the cross-track error of the vehicle and
* `epsi`: the orientation error of the vehicle.

The actuators are  
* the steering angle `delta_psi` of the vehicle and 
* the acceleration `a` of the vehicle.

The model is updated using the following equations:
```
        px[t+1] = px[t] + v[t] * cos(psi[t]) * dt
        py[t+1] = py[t] + v[t] * sin(psi[t]) * dt
        psi[t+1] = psi[t] + v[t] / Lf * delta_psi[t] * dt
        v[t+1] = v[t] + a[t] * dt
        cte[t+1] = f(dx[t]) - dy[t] + v[t] * sin(epsi[t]) * dt
        epsi[t+1] = psi[t] - psi_des[t] + v[t] / Lf * delta[t] * dt
```
where  
* `t` is the time step,
* `dt` is the elapsed time between update steps,
* `Lf` is the length from the vehicle's front to its center of gravity,
* `f(x)` is the desired y-position of the car (ideally the center of the road) and
* `psi_des` is the desired orientation of the car (ideally the road's curvature).


## Timestep Length and Elapsed Duration (N & dt)
The implementation uses a time step length `N` of `10` and a step duration `dt` of `0.1` (i.e., 100ms). 

I started out with the parameters suggested in the exercise, namely `N=25` and `dt=0.05`. The prediction horizon seemed fine, but reducing the number of steps while increasing the step duration to 0.1 made the vehicle a little bit more responsive while roughly maintaining the time horizon.

The smallest value with good results was chosen for `N` so that computation is reduced. A value of 0.1 for `dt` also made it convenient to deal with the latency of 100ms which now corresponds to exactly one time step.

## Polynomial Fitting and MPC Preprocessing
The original waypoints in the map's coordinate system were transformed via translation and rotation in the vehicle's coordinate system (see system's description above). This simplifies all further calculations.
A third-order was fitted to the waypoints to get a function of the road from which the desired y-position and orientation can be obtained.

## Model Predictive Control with Latency
MPC minimizes a cost function consisting of:  
* a `cte` term to stay close to the center of the road,
* a `epsi` term to position the vehicle to the right direction,
* a `v` term to maintain a velocity close to a reference velocity,
* two `delta_psi` terms, one to regularize the use of large steering angles and one regularizes drastic changes in steering angles, and
* two `a` terms, equivalently to `delta_psi`.

The reference velocity is dynamically changed according the current curvature of the road. The gradient of the fitted polynomial is used as an indicator where the reference velocity is high with gradients close to 0 and becomes smaller when the absolute value of gradient increases during turns on the road.

The latency of 100ms is dealt with in two steps:  
1. Instead of using the predicted actuators at the current time step `t=0`, the ones at `t=1` are sent to the simulator as next actions since the duration of one time step equals the latency of 100ms. Hence, they are executed exactly at the time they are predicted for.
2. The first predicted actuator at `t=0` are constrained to be the actuators that were sent to the simulator in the previous MPC step. This ensures that the first predicted actuators (which cannot be used due to the latency) do not influence the subsequent predictions. Instead the vehicle will be predicted to continue on its current path for the first time step (the 100ms that we are not able to influence anymore).

## Further Improvements
The vehicle safely maneuvers around the track, but there is still room for improvement. The latency also affects the waypoints and the vehicle's relative position to these. Especially during turns, the predicted center of the road is not aligned with the actual road because of that and, thus, the vehicle drives slightly off center. To improve this, the latency also has to be considered in the vehicles position with respect to the waypoints.
