# Ground Vehicles C++ Project

## Overview

This project develops a high-level management system for the Polaris GEM e2 simulator, handling various sensor inputs to manage the robot's state and coordinate navigation tasks. The system dynamically reacts to sensor data, including battery level, temperature, GPS accuracy, internet signal strength, and emergency stop signals, to navigate and manage the robot's state efficiently.

## Installation Instructions

Follow these steps to install the simulator and run the simulation:

1. Clone the POLARIS_GEM_e2 simulator from GitLab:

   ```
   git clone <POLARIS_GEM_e2 repository URL>
   ```

2. Build the workspace following the instructions provided in the simulator's README.

3. Run the simulation with the following command:

   ```
   roslaunch gem_gazebo gem_gazebo_rviz.launch velodyne_points:="true"
   ```

You should see a car-like robot model simulated, equipped with a front laser, sonar, 3D Velodyne lidar, single camera, GPS, and IMU.

## Project Structure

The project is structured as follows within the `catkin_ws`:

```
catkin_ws/
└── src/
    ├── POLARIS_GEM_e2/
    │   ├── polaris_gem_drivers_sim/
    │   │   ├── gem_navigation/
    │   │   │   └── src/
    │   │   │       └── planner.cpp
    │   │   └── gem_pure_pursuit_sim/
    │   │       └── scripts/
    │   │           └── pure_pursuit_sim.py
    └── sensor_data_management_system/
        ├── src/
        │   └── sensor_data_manager.cpp
        └── test/
            ├── integration.test
            ├── test_battery_sensor.cpp
            ├── test_emerg_stop.cpp
            ├── test_gps_sensor.cpp
            ├── test_internet.cpp
            └── test_temperature_sensor.cpp
```

- `planner.cpp`: Path-tracking controller.
- `pure_pursuit_sim.py`: Simulation script for pure pursuit path tracking.
- `sensor_data_manager.cpp`: Manages robot states based on sensor inputs.
- `test_*.cpp` & `integration.test`: Unit and integration tests for sensor inputs and system functionality.

## Docker Setup

A Dockerfile is provided to encapsulate the environment and dependencies required to run the code and the simulation. Use the following steps to build and run the Docker container:

1. Build the Docker image:

   ```
   docker build -t ground_vehicles_cpp_project .
   ```

2. Run the Docker container:

   ```
   docker run -it --rm ground_vehicles_cpp_project
   ```

This Docker setup ensures that all dependencies are met for running the simulation and the management system code.

## Design and Implementation

The system is designed with modularity, scalability, and real-time operation in mind. It includes:

- A sensor data management system to dynamically manage robot states based on sensor inputs.
- A high-level planner for determining navigation tasks based on the robot's current state.
- Integration of a path-tracking controller node for waypoint navigation.

### Challenges and Solutions

- **Sensor Integration**: Mock sensors were developed to simulate real sensor data, allowing for comprehensive testing.
- **State Management**: A state machine was implemented to manage transitions between IDLE, RUNNING, and ERROR states based on sensor inputs.

## Testing Scenarios

The system was tested in various scenarios to ensure robustness and reliability. These include battery level variations, temperature fluctuations, GPS accuracy changes, internet signal strength variations, and emergency stop activation.

## Conclusion

This project showcases the development of a high-level management system for a simulated robot vehicle. It emphasizes modularity, scalability, and real-time operation, providing a robust framework for future enhancements and integration with real-world applications.
