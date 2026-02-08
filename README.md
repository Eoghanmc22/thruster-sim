# Thruster Configuration Optimizer for Underwater Robotics

This uses a gradient ascent based algorithm to optimize the placement of
thrusters on an underwater robot.

Currently, the visualizer optimizes configurations of six thrusters symmetric
about the yz plane. The backend, however, supports optimizing arbitrary
thruster configurations. In the web demo, two other optimization types by going
to the "Optimization Arena" drop down, and clicking on "Optimization Type".

The math for this project is reused from the *motor_math* crate in my ROV
software stack at
[Eoghanmc22/robocode](https://github.com/Eoghanmc22/robocode/)

