# Thruster Configuration Optimizer for Underwater Robotics

This used gradient ascent to optimize the placement of thrusters to maximize a heuristic function.

Currently, the visualizer only supports symmetrical thruster configurations with one thruster on each vertex of a box. The backend, however, supports optimizing arbitrary thruster configurations. This functionality can be used by modifying the "ascent" binary.

The math for this project is reused from the *motor_math* crate in [Eoghanmc22/mate-rov-2025](https://github.com/Eoghanmc22/mate-rov-2025/)

