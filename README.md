# Atmospheric-Process-Estimation

In this project, I focused on designing an optimal control problem for estimating the parameters of an atmospheric dispersion process model based on measurement collected by autonomous mobile robots. Locally optimal waypoint and sensor trajectories are computed by maximizing trace of information matrix formed by considering past and estimated future locations. The robots follow these waypoints to maximize the information gain of the acquired measurements and estimate the spatiotemporal field. Simulations show the effectiveness of the proposed scheme in reducing the error between the estimated and the true dispersion model parameters.

Methodology
1. This application starts with a consideration of the underlying mobility of the sensing agents in the environment, which hereby is referred as the robot dynamics. The motion constraints of robots are modelled as a point mass with double integrator dynamics π₯ΜΜ=π’

2. The underlying atmospheric process is modelled by a 3D advection-diffusion equation as follows; πΏπΆπΏπ‘=ββ(πΆπ£π€βπΎβπΆ). Here π£π€ is wind or water velocity in [m/s].

3. Parameter Estimation
Measurements are taken with on-board sensors and integrated with current best estimate of the spatiotemporal field, as represented by the physical model. All of this happens in the observer block. This assimilation of data is done collaboratively with other robots. To estimate the parameter vector π, following non-linear least square problem is solved: minΞΈ12||π(π£βπΆ(π))||22 π€βπππ π=ππππ(π1β1,π2β1,β¦,ππβ1)

4. Optimal Control
The observerβs output to guide the collection of subsequent measurements (i.e., adaptive sampling). The controller block scores candidate sampling trajectories and selects the one that maximizes information gain. This is done by maximizing the π‘ππππ{π»ππ»+π½πΆπ½πΆ+ππΌ}. Here, the first term corresponds to the information due to the previous collected measurements by the robots, while the second term represents the expected information that will be contributed by the future measurements scheduled to be collected by the robots that have already determined the optimal locations for them to go to. π»,π½π are Jacobians computed as; πΆ(π)=πΆ(πΜ)+π½π(πβπΜ) π½π=(βπΆ1(πΜ,π1),βπΆ2(πΜ,π2),β¦,βπΆπ(πΜ,π2))π
For the final step, controller gives control inputs to the robot and finishes the loop. Finally, sampling tasks are allocated to each of the N robots.

## Algorithm
  1.  Collect M concentration measurements
  2. Solve estimation problem (1) for all available measurement data based on initial/last estimate to obtain new estimator πΜ
  3. Generate information matrix π½πππ½π for πΜ based on last M measurement locations
  4. Solve mixed non-linear integer program to select path with highest score 
  5. Apply model predictive control for cooperative sampling until measurements locations are processes.
  6. (Back to step 2).
