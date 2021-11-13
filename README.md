# Atmospheric-Process-Estimation

In this project, I focused on designing an optimal control problem for estimating the parameters of an atmospheric dispersion process model based on measurement collected by autonomous mobile robots. Locally optimal waypoint and sensor trajectories are computed by maximizing trace of information matrix formed by considering past and estimated future locations. The robots follow these waypoints to maximize the information gain of the acquired measurements and estimate the spatiotemporal field. Simulations show the effectiveness of the proposed scheme in reducing the error between the estimated and the true dispersion model parameters.

Methodology
1. This application starts with a consideration of the underlying mobility of the sensing agents in the environment, which hereby is referred as the robot dynamics. The motion constraints of robots are modelled as a point mass with double integrator dynamics 𝑥̇̇=𝑢

2. The underlying atmospheric process is modelled by a 3D advection-diffusion equation as follows; 𝛿𝐶𝛿𝑡=−∇(𝐶𝑣𝑤−𝐾∇𝐶). Here 𝑣𝑤 is wind or water velocity in [m/s].

3. Parameter Estimation
Measurements are taken with on-board sensors and integrated with current best estimate of the spatiotemporal field, as represented by the physical model. All of this happens in the observer block. This assimilation of data is done collaboratively with other robots. To estimate the parameter vector 𝜃, following non-linear least square problem is solved: minθ12||𝑊(𝑣−𝐶(𝜃))||22 𝑤ℎ𝑒𝑟𝑒 𝑊=𝑑𝑖𝑎𝑔(𝜎1−1,𝜎2−1,…,𝜎𝑚−1)

4. Optimal Control
The observer’s output to guide the collection of subsequent measurements (i.e., adaptive sampling). The controller block scores candidate sampling trajectories and selects the one that maximizes information gain. This is done by maximizing the 𝑡𝑟𝑎𝑐𝑒{𝐻𝑇𝐻+𝐽𝐶𝐽𝐶+𝜇𝐼}. Here, the first term corresponds to the information due to the previous collected measurements by the robots, while the second term represents the expected information that will be contributed by the future measurements scheduled to be collected by the robots that have already determined the optimal locations for them to go to. 𝐻,𝐽𝑐 are Jacobians computed as; 𝐶(𝜃)=𝐶(𝜃̅)+𝐽𝑐(𝜃−𝜃̅) 𝐽𝑐=(∇𝐶1(𝜃̅,𝑝1),∇𝐶2(𝜃̅,𝑝2),…,∇𝐶𝑚(𝜃̅,𝑝2))𝑇
For the final step, controller gives control inputs to the robot and finishes the loop. Finally, sampling tasks are allocated to each of the N robots.

5. Algorithm
  1.  Collect M concentration measurements
  2. Solve estimation problem (1) for all available measurement data based on initial/last estimate to obtain new estimator 𝜃̂
  3. Generate information matrix 𝐽𝑐𝑇𝐽𝑐 for 𝜃̂ based on last M measurement locations
  4. Solve mixed non-linear integer program to select path with highest score 
  5. Apply model predictive control for cooperative sampling until measurements locations are processes.
  6. (Back to step 2).
