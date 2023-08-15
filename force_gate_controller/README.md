# force_gate_controllers package

The package implements controllers to interpolate joint's trajectory with the addition of a Force / Torque safety threshold.

It will be periodically updated with changes from the upstream [joint_trajectory_controller](https://github.com/ros-controls/ros2_controllers).

For detailed documentation check the `docs` folder or [ros2_control documentation](https://control.ros.org/).

The only additional ROS2 parameters are under the `wrench_threshold` namespace. Look in `joint_trajectory_controller_parameters.yaml` for documentation.
