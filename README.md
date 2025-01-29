# Thruster Configuration Optimizer for Underwater Robotics

This uses a gradient ascent based algorithm to optimize the placement of thrusters on an underwater robot.

Currently, the visualizer optimizes configurations of six thrusters symmetric about the yz plane. The backend, however, supports optimizing arbitrary thruster configurations. This functionality can be used by modifying the "ascent" binary.

The math for this project is reused from the *motor_math* crate in [Eoghanmc22/mate-rov-2025](https://github.com/Eoghanmc22/mate-rov-2025/)

