
---

** Model Predictive Control Project**

The goal of this project are the following:

Implement the Model Predictive Controller (MPC) and control the simulator car such that the car drives through the track without leaving it.

[//]: # (Image References)
[image1]: ./examples/cte8.png
[image8]: ./examples/curve1.png
[image9]: ./examples/curve2.png
[image10]: ./examples/straight1.png
[video1]: ,/examples/mpc2.mp4

## [Rubric Points](https://review.udacity.com/#!/rubrics/824/view)
All the code for this project has been derived from the example code in the course and is in this directory.
[Here](https://github.com/gvp-study/CarND-MPC-Project.git)

# MPC Controller
## Implementing the MPC Controller

I implemented the MPC control based on the lessons and looking at links to the implementations suggested by David Silver.
This is a revision of the implementation based on the suggestions  by the first reviewer to take care of the latency better. The reviewer also corrected my velocity unit conversion factor and the sign in the delta in the constraint equations. The result was a better controller. I have updated this document with the video and graphs.

 (https://medium.com/self-driving-cars/five-different-udacity-student-controllers-4b13cc8f2d0f)

Unlike the PID controller, the MPC explicitly uses the kinematic model of the car and its control actuators to predict its state. More importantly, it incorporates the non linear system solver which can optimize a very general cost function over a large number of future states. This lets us optimize for various driving parameters such as smoothness, cross track error, orientation error, change in the actuators etc.

## Model
The state of the car at every instant of time is defined by a six element vector X = (x, y, ψ​, v, cte, eψ​). The first three (x, y, ψ​) are the coordinates of the car pose in global coordinates. (v) is the velocity. The last two (cte, eψ​) are the error in distance and angle from the desired pose in the road.
The control inputs to the car are its steering angle and throttle u = (δ​, a).
The Kinematic equations used for updating vehicle state is based on the following equation based on the kinematics defined by **A** and the control **b**.
```
Xt+1 = A*Xt + b*ut
```
The distance Lf between the front of the car and the center of gravity of the car which decides the effect of the steering on the actual turning of the car.
The state transition equation based on the kinematics of the system is as shown below.
```
x​t+1​​ = x​t ​​+ v​t ​​∗ cos(ψ​t​​) ∗ dt
y​t+1​​ = y​t​​ + v​t​​ ∗ sin(ψ​t​​) ∗ dt
ψ​t+1​​ = ψ​t​​ +​ L​f​​​​ * v​t​​​​ ∗ δ ∗ dt
v​t+1​​ = v​t​​ + a​t ​​∗ dt
cte​t+1​​ = f(x​t​​) − y​t ​​+ (v​t​​ ∗ sin(eψ​t​​) ∗ dt)
eψ​t+1​​ = eψ​t​​ +​ L​f ​* ​​​v​t​​​​ ∗ δ​t​​ ∗ dt
```
The model defines the state of the car for N steps ahead of its current state. It does this by augmenting its six state elements with the N*6 future state elements. It also augments the state further by adding the (N-1) control inputs that are used in between the N states. The result is a (6N + 2N - 2) state vector. For N=10 this will result in a 78 element state vector.
The state elements corresponding to the time t+1 of this large state vector is related to its previous time t as shown in the kinematic equation above. The time interval dt between t+1 and t is the time between actuations.
## Timestep Length and Elapsed Duration (N & dt)
The values of N and dt was decided by trial and error. I started out with N=10 and t=0.1 sec which worked well. I then increased N=20 and t=0.1 and the car seemed to be tracking the road better for the gentle turns and the straights but it became unstable at large turns and beyond. The compuation time also doubled from the N=10 case. I also tried to use N=10 and t=0.05 and it caused the car to weave about the center line which I think is due to the fact that the action is only 0.05 seconds ahead and does not cover the 0.1 time cycle of the system. I also noted that the N*dt = 1second of path horizon which translates to about 44 meters in front of the car for a max speed of 100mph. Based on my experiments I finally settled on N=10 and t=0.1.
## Constraints
The ipopt solver requires the user to supply the upper and lower limits of all the state elements explicitly. The system defining the car has constraints on both the actuator inputs u and the state vector X.
The constraints on the state vector X is defined exactly by the state equations given above. This is done by constraining the error from the computed Xt+1 from the equations and the observed Xt+1 to zero.
The constraints on the actuator inputs (δ​, a) are as follows.
```
-25degs < (δ​) < +25degs
-1 < (a) < +1
```
## Cost Function
The cost function for the system based on the augmented state vector and actuator is as follows.
```
    // The cost is stored is the first element of `fg`.
    // Any additions to the cost should be added to `fg[0]`.
    fg[0] = 0;

    // Cost function
    // TODO: Define the cost related the reference state and
    // any anything you think may be beneficial.

    // The part of the cost based on the reference state.
    for (int t = 0; t < N; t++) {
      fg[0] += cost_cte * CppAD::pow(vars[cte_start + t], 2);
      fg[0] += cost_epsi * CppAD::pow(vars[epsi_start + t], 2);
      fg[0] += cost_v * CppAD::pow(vars[v_start + t] - ref_v, 2);
    }

    // Minimize the use of actuators.
    for (int t = 0; t < N - 1; t++) {
      fg[0] += cost_delta * CppAD::pow(vars[delta_start + t], 2);
      fg[0] += cost_a * CppAD::pow(vars[a_start + t], 2);
      fg[0] += cost_deltav * CppAD::pow(vars[delta_start + t]*vars[v_start + t], 2);
    }

    // Minimize the value gap between sequential actuations.
    for (int t = 0; t < N - 2; t++) {
      fg[0] += cost_deltadot * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
      fg[0] += cost_adot * CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
    }
```
The cost function that we are trying to minimize increases with the error (cte, epsi, v-ref_v).
The cost also increases with the magnitude of the actuation (δ​, a, δ​ * v). The first two elements minimizes the use of the actuators. The third element ​(δ​ * v) penalizes high velocities during large turns by using their product.
The final set of costs are related to the change in the actuator values between cycles. This allows the driving be smooth.
## Fitting the 3rd Order Polynomial
The desired path of the car is defined by a series of (x, y) points marking the center line of the road in front of the car. To allow the solver to be able to compute the solution, I first transformed the global way points to the local car coordinates. I then fitted a third order polynomial to these points using the polyfit function. The four coefficients returned by the polyfit function is then used by the ipopt solver.
## Latency
The simulator allows for the simulation of a delay for the actuator to show up on the physical system. This makes it mimic real life systems where the delay is unavoidable. This latency can be modeled into the state equations. I initially put them in the constraints on the actuator state variables u as shown below. But, after feedback from the reviewer, I decided to precompute the change in the state in main.cpp by interpolating the velocity and steering for the duration of the delay in the actions to appear in the car as shown in the code below.
```
    //
	  // Add the latency to the state.
	  //
	  double latency = 0.1;
	  double delta = -steer_value;
	  double a = throttle_value;
	  //
	  // New local pose based on last control inputs.
	  //
	  if(v > 20*0.44704)
	    psi = delta;
	  else
	    psi = 0.0;
	  px = 0.0 + v * cos(psi) * latency;
	  py = 0.0 + v * sin(psi) * latency;
	  cte = cte + v * sin(epsi) * latency;
	  epsi = epsi + v/Lf * delta * latency;
	  psi = psi + v/Lf * delta * latency;
	  v = v + a * latency;

```
The result of this change was dramatic in that the car was able to track the path at higher velocities. I was able to raise the max speed to 100mph. I found that there was instability at the start when the velocity is low so I put in a threshold for 20mph for the steer angle to affect the pose.
## Solving the Non-Linear System of Equations
The optimal solution of the resulting model of N states and N-1 actuations is computed using a Ipopt solver. The solver uses the state equations, the constraints, and the polyfit coefficients that fit the path. It uses all these to compute the optimal solution within a given time limit.
The larger the value of N, the greater the computation needed to solve the nonlinear optimization problem. I used N=10 in my system to keep the computation well within the 100msec cycle time of the control loop.
I computed the time taken for the Solve method and found that it was approximately 0.01seconds for N=10 and dt=0.1. The compute time increased almost linearly with N and was 0.02 for N=20.
## Command line arguments
I made the mpc program to be able to take in arguments that could be used to set the weights for the eight objective functions.
The mpc program when called without arguments use these default arguments for these eight objective function weights. They are as follows (The default values are in parenthesis)
* CTE cost factor (100)
* ErrorPsi cost factor (1000)
* Velocity cost factor (1)
* Steering command cost factor (1)
* Throttle command cost factor (1)
* Velocity times Steering command cost factor (100)
* Steering command delta cost factor (100)
* Throttle command delta cost factor (10)

The user can change the default values by typing the desired values in order in the command line like so.

mpc 100 1000 1 1 1 100 100 10

## Reflections on the cost factors
To find the appropriate values of the cost factors, I conducted experiments by driving the simulator through a lap with each set of parameters. I then wrote out the (cte, epsi, steering, velocity) at each cycle of the control loop into a file. I then used a python matplotlib library to plot the values of these normalized parameters for each run. The results are shown below.
Note that the velocity plot in red is normalized to a maximum velocity of 100mph for all the plots below.

### mpc 100 1000 1 1 1 100 100 10
![alt text][image1]

Notice that the final default parameter set I chose shown above allows the simulator to maintain the high speed through most of the curves and the straights. I let the CTE and EPSI in this case to be much larger than for the previous sets of parameters as long as it kept the car within the road. I also found that the relaxation of the CTE errors allows the car to cut corners to maintain the speed similar to the techniques used by race car drivers when they take corners.
I let the simulator run several laps over time to make sure that the chosen parameters resulted in a stable control system.

Some of the still images of the run are shown below.
![alt text][image8]
![alt text][image9]
![alt text][image10]
### Output
The movie of the simulator controlled by the MPC controller is shown below.[link to my video](./examples/mpc2.mp4)
![alt text][video1]

## References
https://medium.com/self-driving-cars/five-different-udacity-student-controllers-4b13cc8f2d0f
