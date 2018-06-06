
---

** Model Predictive Control Project**

The goal of this project are the following:

Implement the Model Predictive Controller (MPC) and control the simulator car such that the car drives through the track without leaving it.

[//]: # (Image References)
[image1]: ./examples/cte1.png
[image2]: ./examples/cte2.png
[image3]: ./examples/cte3.png
[image4]: ./examples/cte4.png
[image5]: ./examples/cte5.png
[image6]: ./examples/cte6.png
[image7]: ./examples/cte7.png
[image8]: ./examples/curve1.png
[image9]: ./examples/curve2.png
[image10]: ./examples/straight1.png
[video1]: ,/examples/mpc.mp4

## [Rubric Points](https://review.udacity.com/#!/rubrics/824/view)
All the code for this project has been derived from the example code in the course and is in this directory.
[Here](https://github.com/gvp-study/CarND-MPC-Project.git)

# MPC Controller
## Implementing the MPC Controller

I implemented the MPC control based on the lessons and looking at links to the implementations suggested by David Silver. (https://medium.com/self-driving-cars/five-different-udacity-student-controllers-4b13cc8f2d0f)

Unlike the PID controller, the MPC explicitly uses the kinematic model of the car and its control actuators to predict its state. More importantly, it incorporates the non linear system solver which can optimize a very general cost function. This lets us optimize for various driving parameters such as smoothness, cross track error, orientation error, change in the actuators etc.

## Model
The state of the car at every instant of time is defined by a six element vector X = (x, y, ψ​, v, cte, eψ​). (x, y, ψ​) are the coordinates of the car pose in global coordinates. (v) is the velocity. (cte, eψ​) are the error in distance and angle from the desired pose in the road.
The control inputs to the car are its steering angle and throttle u = (δ​, a).
The Kinematic equations used for updating vehicle state are as follows. Lf is the distance between the front of the car and the center of gravity of the car which decides the effect of the steering on the actual turning of the car.
The state transition equation based on the kinematics of the system is as shown below. This Xt+1 = A*Xt + b*ut
```
x​t+1​​ = x​t ​​+ v​t ​​∗ cos(ψ​t​​) ∗ dt
y​t+1​​ = y​t​​ + v​t​​ ∗ sin(ψ​t​​) ∗ dt
ψ​t+1​​ = ψ​t​​ +​ L​f​​​​ * v​t​​​​ ∗ δ ∗ dt
v​t+1​​ = v​t​​ + a​t ​​∗ dt
cte​t+1​​ = f(x​t​​) − y​t ​​+ (v​t​​ ∗ sin(eψ​t​​) ∗ dt)
eψ​t+1​​ = eψ​t​​ +​ L​f ​* ​​​v​t​​​​ ∗ δ​t​​ ∗ dt
```
The model defines the state of the car N steps ahead of its current state. It does this by augmenting its six states with the N*6 future states.  It also augments the state further by adding the (N-1) control inputs that are used in between the N states. The result is a (6N + 2N - 2) state vector. For N=10 this will result in a 78 element state vector.
The state elements corresponding to the time t+1 of this large state vector is related to its previous time t as shown in the kinematic equation above. The time interval dt between t+1 and t is the time between actuations.
## Timestep Length and Elapsed Duration (N & dt)
The values of N and dt was decided by trial and error. I started out with N=10 and t=0.1 sec. I then increased N=20 and t=0.1 and the car seemed to be tracking the road very well for the gentle turns and the straights but it became unstable at the big third turn and beyond. I also tried to use N=10 and t=0.05 and it caused the car to weave about the center line. So I finally settled on N=10 and t=0.1.
## Constraints
The system defining the car has constraints on both the actuator inputs u and the state vector X.
The constraints on the state vector X is defined exactly by the state equations given above.
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
The cost also increases with the magnitude of the actuation (δ​, a, δ​ * v). This minimizes the use of the actuators and also penalizes high velocities during large turns by using their product (δ​ * v).
The final set of costs are related to the change in the actuator values between cycles. This allows the driving be smooth.
## Fitting the 3rd Order Polynomial
The desired path of the car is defined by a series of (x, y) points marking the center line of the road in front of the car. To allow the solver to be able to compute the solution, I first transformed the global way points to the local car coordinates. I then fitted a third order polynomial to these points using the polyfit function. The four coefficients returned by the polyfit function is then used by the ipopt solver.
## Latency
The simulator allows for the simulation of a delay for the actuator to show up on the physical system. This makes it mimic real life systems where the delay is unavoidable. This latency can be modeled into the state equations. I put them in the constraints on the actuator state variables u as shown below.
```
      //
      // Account for delay in seeing result of action to be 1 cycle behind.
      //
      if (t > 1)
      {   
        a = vars[a_start + t - 2];
        delta = vars[delta_start + t - 2];
      }
```
I did try an alternative place to impose the latency by setting the current state to be the result of the state plus the result of the previous actuator values. This did not seem to work as well. So I commented that code out.
## Solving the Non-Linear System of Equations
The optimal solution of the resulting model of N states and N-1 actuations is computed using a Ipopt solver. The solver uses the state equations, the constraints, and the polyfit coefficients to compute the optimal solution within a given time limit.
The larger the value of N, the greater the computation needed to solve the nonlinear optimization problem. I used N=10 in my system to keep the computation well within the 100msec cycle time of the control loop.
## Command line arguments
I made the mpc program to be able to take in arguments that could be used to set the weights for the eight objective functions.
The mpc program can be run with default arguments for these eight objective function weights. They are as follows (The default values are in parenthesis)
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

### mpc 3000 3000 1 5 5 700 200 10
![alt text][image1]
### mpc 100 100 1 1 1 100 10 10
![alt text][image2]
### mpc 1000 100 1 1 1 100 10 10
![alt text][image3]
### mpc 100 10 1 1 1 100 10 10
![alt text][image4]
### mpc 100 100 1 1 1 100 100 100
![alt text][image5]
### mpc 100 100 1 1 1 100 100 10
![alt text][image6]
### mpc 100 1000 1 1 1 100 100 10
![alt text][image7]

Notice that the final default parameter set I chose shown above allows the simulator to maintain the steady 78mph speed through most of the curves and the straights. The CTE and EPSI in this case is much larger than for the previous sets of parameters. I also found that the relaxation of the CTE errors allows the car to cut corners to maintain the speed similar to the techniques used by race car drivers when they take corners.
I let the simulator run several laps over time to make sure that the chosen parameters resulted in a stable control system.

Some of the still images of the run are shown below.
![alt text][image8]
![alt text][image9]
![alt text][image10]
### Output
The movie of the simulator controlled by the MPC controller is shown below.[link to my video](./examples/mpc.mp4)
![alt text][video1]

## References
https://medium.com/self-driving-cars/five-different-udacity-student-controllers-4b13cc8f2d0f
