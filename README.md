# Ground Vehicles C++ Project

## Overview

This project develops a high-level management system for the Polaris GEM e2 simulator, handling various sensor inputs to manage the robot's state and coordinate navigation tasks. The system dynamically reacts to sensor data, including battery level, temperature, GPS accuracy, internet signal strength, and emergency stop signals, to navigate and manage the robot's state efficiently.

## Installation Instructions

Follow these steps to install the simulator and run the simulation:

1. Clone the POLARIS_GEM_e2 simulator from GitLab:

   ```
   git clone https://gitlab.engr.illinois.edu/gemillins/POLARIS_GEM_e2.git
   ```

2. Build the workspace following the instructions provided in the simulator's [README.](https://gitlab.engr.illinois.edu/gemillins/POLARIS_GEM_e2/-/blob/main/README.md?ref_type=heads)


## Project Structure

The project is structured as follows within the `catkin_ws`:

~~~
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
    ├── sensor_data_management_system/
    │   ├── src/
    │   │   └── sensor_data_manager.cpp
    │   └── test/
    │       ├── integration.test
    │       ├── test_battery_sensor.cpp
    │       ├── test_emerg_stop.cpp
    │       ├── test_gps_sensor.cpp
    │       ├── test_internet.cpp
    │       └── test_temperature_sensor.cpp
    └── mock_sensor_data/
        └── src/
            ├── mock_battery_data.cpp
            ├── mock_gps_data.cpp
            ├── mock_internet_data.cpp
            └── mock_temp_data.cpp
~~~


- `planner.cpp`: Behaviour Planner that enables the navigation through the waypoints parsed from the .csv file.
- `pure_pursuit_sim.py`: Simulation script for pure pursuit path tracking.
- `sensor_data_manager.cpp`: Manages robot states based on sensor inputs.
- `test_*.cpp` & `integration.test`: Unit and integration tests for sensor inputs and system functionality.
- `mock_*.cpp` : Mock sensor data scripts generating various synthetic sensor readings for sensor data manager to process robot states (IDLE, RUNNING, ERROR). 

## Docker Setup and Run

A Dockerfile is provided to encapsulate the environment and dependencies required to run the code and the simulation. Use the following steps to build and run the Docker container:

1. Build the Docker image:

   ```
   docker build -t ground_vehicles_cpp_project .
   ```

2. Run the Docker container:

   ```
   docker run -it --net=host -e ROS_HOSTNAME=localhost     --env="DISPLAY"     --env="QT_X11_NO_MITSHM=1"     --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw"     ground_vehicles_cpp_project     bash
   ```

3. Open new terminal and allow Docker to use the host machine’s X11 socket:
 
   ```
   xhost +
   ```

4. Use docker exec to get a shell inside the container:

   ```
   docker exec -it <container_id_or_name> bash
   ```

5. Run the simulation with the following command:

   ```
   roslaunch gem_gazebo gem_gazebo_rviz.launch velodyne_points:="true"
   ```

You should see a car-like robot model simulated, equipped with a front laser, sonar, 3D Velodyne lidar, single camera, GPS, and IMU.

6. Run sensor information aggregator with the following command:

   ```
   roslaunch gem_gazebo gem_sensor_info.launch
   ```
7. Run Pure Pursuit tracker that controls the vehicle based on behaviour planner output with the following command:
   ```
   rosrun gem_pure_pursuit_sim pure_pursuit_sim.py
   ```
8. Run behvaiour planner node, which enables path tracker to start moving the vehicle, with the following: 
   ```
   rosrun gem_navigation gem_navigation_node
   ```
9. Run sensor data manager to bridge the raw sensor data and the planner to perform various scenarios with the following:
   ```
    rosrun sensor_data_management_system sensor_data_management_system_node 
   ```
10. Run individual scenarios (battery level, temperature, gps, internet, and emergency):
      - Run battery level unit test with: 
         ```
         rosrun sensor_data_management_system sensor_data_management_system_battery_test
         ```
      - Run temperature variation unit test with: 
         ```
         rosrun sensor_data_management_system sensor_data_management_system_temp_test
         ```
      - Run GPS accuracy fluctuation unit test with: 
         ```
         rosrun sensor_data_management_system sensor_data_management_system_gps_test
         ```
      - Run signal strength unit test with: 
         ```
         rosrun sensor_data_management_system sensor_data_management_system_internet_test
         ```
      - Run emergency button press unit test with: 
         ```
         rosrun sensor_data_management_system sensor_data_management_system_emerg_test
         ```
11. Run Integration test with the following:
    ```
    rostest sensor_data_management_system integration.test
    ```
This Docker setup ensures that all dependencies are met for running the simulation and the management system code.

## Design and Implementation

The autonomous vehicle system is architected to ensure high reliability, real-time responsiveness, and adaptability to dynamic environmental conditions. It encompasses several key components:

- **Sensor Data Management System**: Centralizes sensor data processing and decision-making, facilitating a coherent response to sensory inputs.
- **Mock Sensors**: Simulate real-world sensor data for the battery, GPS, temperature, internet connectivity, and emergency stop scenarios, enabling thorough testing in a controlled environment.
- **High-Level Planner**: Determines navigation strategies based on current vehicle status and environmental data.
- **Path-Tracking Controller**: Implements the pure pursuit algorithm for accurate waypoint following.

### Challenges and Solutions

Implementing a robust and scalable autonomous vehicle system posed numerous challenges:

- **Real-Time Sensor Data Processing**: Achieving minimal latency in processing sensor data and executing corresponding actions.
- **State Management**: Designing an efficient state machine to handle various vehicle states (e.g., IDLE, RUNNING, ERROR) based on sensor inputs.
- **Sensor Data Simulation**: Creating realistic mock sensor data sources for comprehensive testing without the need for physical sensors.
- **Integration Testing**: Ensuring seamless integration and interaction between system components under various scenarios.

## Testing Scenarios and Unit Tests

To validate the system's performance and reliability, extensive testing scenarios were devised, focusing on unit tests for each sensor type and integration tests for overall system behavior.

### Battery Level Monitoring

- **Design and Implementation**: Incorporates mock battery data to simulate discharge patterns and trigger system state transitions at critical levels.
- **Challenges**: Included simulating realistic battery discharge patterns and ensuring accurate system state transitions under edge conditions.
- **Test Fixture**: Simulates scenarios where battery level gradually decreases to critical thresholds, testing the system's response to transition into appropriate states.

#### Test Fixture: TestBatterySensor

This fixture prepares the environment for battery sensor testing, including publishing simulated battery levels and subscribing to the system's status messages.

```cpp
class TestBatterySensor : public ::testing::Test {
protected:
    ros::NodeHandle nh;
    ros::Publisher battery_pub;
    ros::Subscriber status_sub;
    gem_navigation::StringStamped::ConstPtr last_status_msg;

    virtual void SetUp() {
        battery_pub = nh.advertise<std_msgs::Int32>("/battery_level", 1);
        status_sub = nh.subscribe("/car_status", 1, &TestBatterySensor::statusCallback, this);
    }

    void publishBattery(int battery) {
        std_msgs::Int32 msg;
        msg.data = battery;
        battery_pub.publish(msg);
    }

    void statusCallback(const gem_navigation::StringStamped::ConstPtr& msg) {
        last_status_msg = msg;
    }

    void waitForStatus(const std::string& expected_status, const ros::Duration& timeout) {
        auto start = ros::Time::now();
        while (ros::Time::now() - start < timeout && ros::ok()) {
            ros::spinOnce();
            if (last_status_msg && last_status_msg->data == expected_status) {
                return;
            }
        }
        ASSERT_TRUE(last_status_msg != nullptr);
        EXPECT_EQ(last_status_msg->data, expected_status);
    }
};
```

**Key Experiments:**

- **BatteryErrorState**: This test assesses the system's response to varying battery levels, particularly focusing on thresholds that should trigger different states like ERROR or RUNNING.

```cpp
TEST_F(TestBatterySensor, BatteryErrorState) {
    // Decrease battery from 100 to 51 percent and expect RUNNING state
    for (int battery = 100; battery >= 51; --battery) {
        publishBattery(battery);
        waitForStatus("RUNNING", ros::Duration(2));
    }

    // Test edge cases for battery levels and expected system responses
    std::vector<std::pair<int, std::string>> test_cases = {
        {49, "ERROR"},  // Under 50% triggers ERROR
        {55, "RUNNING"}, // Above 50% remains RUNNING
        {-1, "ERROR"},  // Invalid level triggers ERROR
        {101, "ERROR"}  // Above 100% triggers ERROR
    };

    for (const auto& test_case : test_cases) {
        publishBattery(test_case.first);
        waitForStatus(test_case.second, ros::Duration(2));
    }
}
```

#### Conclusion

Testing the battery sensor ensures the vehicle's power management system reliably monitors and responds to battery levels. Through these tests, we validate that the system appropriately transitions between operational states based on battery charge level, enhancing the vehicle's safety and reliability.
    
### Temperature Monitoring

- **Design and Implementation**: Simulates temperature fluctuations to test the system's ability to maintain operational integrity under extreme conditions.
- **Challenges**: Involved accurately simulating environmental temperature changes and triggering system responses to prevent overheating or freezing.
- **Test Fixture**: Tests include exposing the system to sudden temperature spikes and drops, verifying correct state transitions to protect hardware and maintain safety.

The temperature monitoring functionality is crucial for maintaining the operational integrity of the vehicle's systems, ensuring they operate within safe temperature ranges. The `test_temperature_sensor.cpp` script plays a vital role in validating the vehicle's ability to monitor and respond to temperature changes accurately.

The battery sensor is critical for monitoring the vehicle's power supply and ensuring that it operates within safe parameters. The `test_battery_sensor.cpp` script is designed to validate the functionality of the battery sensor, focusing on its accuracy in reporting battery levels and the system's response to various battery states.

This fixture prepares the testing environment by setting up ROS publishers and subscribers to simulate temperature changes and monitor the vehicle's status response.

```cpp
class TestTempSensor : public ::testing::Test {
protected:
    ros::NodeHandle nh;
    ros::Publisher temp_pub;
    ros::Subscriber status_sub;
    gem_navigation::StringStamped::ConstPtr last_status_msg;

    virtual void SetUp() override {
        temp_pub = nh.advertise<std_msgs::Int32>("/temp_level", 1);
        status_sub = nh.subscribe("/car_status", 1, &TestTempSensor::statusCallback, this);
    }

    void publishTemp(int temp) {
        std_msgs::Int32 msg;
        msg.data = temp;
        temp_pub.publish(msg);
        ROS_WARN_STREAM("Published Temperature: " << msg.data);
    }

    void statusCallback(const gem_navigation::StringStamped::ConstPtr& msg) {
        last_status_msg = msg;
        ROS_WARN_STREAM("Received Car Status: " << last_status_msg->data);
    }
};
```

**Experiment: TemperatureErrorState**

This test simulates a gradual increase in temperature, monitoring the system's transition to an ERROR state at critical temperature thresholds. It also verifies the system's ability to return to a normal operational state once temperatures are back within safe limits.

```cpp
TEST_F(TestTempSensor, TemperatureErrorState) {
    // Gradually increase temperature from 30 to 55 over 30 seconds
    simulateTemperatureChange(30, 55, 30);
    // Check for ERROR state at 55° C
    waitForStatus("ERROR", ros::Duration(1));
    // Spike temperature to 60° C and verify ERROR state is maintained
    publishTemp(60);
    ros::Duration(1).sleep(); // Allow time for message processing
    waitForStatus("ERROR", ros::Duration(5));
    // Drop temperature back to 30° C and verify system returns to RUNNING state
    publishTemp(30);
    waitForStatus("RUNNING", ros::Duration(1));
}
```

#### Conclusion

This temperature sensor test scenario is crucial for validating the vehicle's ability to detect and respond to hazardous temperature levels. By simulating realistic temperature fluctuations, it ensures that the vehicle can maintain safe operating conditions, highlighting the effectiveness of the sensor data management system in responding to environmental changes.


### GPS Sensor Accuracy

- **Design and Implementation**: Mocks GPS data to test navigation accuracy.
- **Challenges**: Ensured realistic simulation of GPS signal variability test system resilience.
- **Test Fixture**: Includes scenarios of GPS signal loss, assessing the system's capacity to maintain operational awareness.

The GPS sensor's accuracy is vital for the navigation and operational safety of the vehicle. The `test_gps_sensor.cpp` script is designed to validate the vehicle's GPS sensor accuracy and the system's ability to handle varying levels of GPS accuracy.

#### Test Fixture: TestGPSSensor

This fixture sets up the necessary environment to simulate different GPS accuracy levels and monitor the system's response. It aims to ensure that the vehicle can correctly handle GPS data under various conditions.

```cpp
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include "gem_navigation/StringStamped.h"

class TestGPSSensor : public ::testing::Test {
protected:
    ros::NodeHandle nh;
    ros::Publisher gps_pub;
    ros::Subscriber status_sub;
    gem_navigation::StringStamped::ConstPtr last_status_msg;

    virtual void SetUp() override {
        gps_pub = nh.advertise<std_msgs::Int32>("/gps_accuracy", 1);
        status_sub = nh.subscribe("/car_status", 1, &TestGPSSensor::statusCallback, this);
    }

    void publishGPSAccuracy(int accuracy) {
        std_msgs::Int32 msg;
        msg.data = accuracy;
        gps_pub.publish(msg);
    }

    void statusCallback(const gem_navigation::StringStamped::ConstPtr& msg) {
        last_status_msg = msg;
    }
};
```

**Key Experiments:**

- **GPSErrorState**: This test evaluates the system's response to changes in GPS accuracy, particularly focusing on the transition between normal and error states based on the duration and extremes of GPS accuracy levels.

```cpp
TEST_F(TestGPSSensor, GPSErrorState) {
    // Simulate various GPS accuracy scenarios and evaluate system response
    simulateGPSAccuracyChange(100, 300, 14); // High accuracy for 14 seconds, expect RUNNING state
    waitForStatus("RUNNING", ros::Duration(1));

    simulateGPSAccuracyChange(100, 300, 16); // High accuracy for 16 seconds, expect ERROR state
    waitForStatus("ERROR", ros::Duration(1));

    // Test values outside the valid range
    publishGPSAccuracy(-1); // Below valid range, expect ERROR state
    waitForStatus("ERROR", ros::Duration(1));

    publishGPSAccuracy(1001); // Above valid range, expect ERROR state
    waitForStatus("ERROR", ros::Duration(1));
}
```

#### Conclusion

The GPS accuracy tests are designed to ensure that the vehicle's navigation system can reliably interpret GPS data under various conditions and maintain operational safety by responding appropriately to data accuracy issues. Through simulating different levels of GPS accuracy, this testing framework verifies the robustness of the GPS sensor integration and the system's ability to adapt to changes in environmental conditions affecting GPS reliability.
    

### Internet Connectivity

- **Design and Implementation**: Tests the vehicle's ability to maintain a stable internet connection under various conditions.
- **Challenges**: Ensures the system can handle variations in signal strength and connectivity interruptions gracefully.
- **Test Fixture**: Simulates scenarios of fluctuating internet signal strength, assessing the system's capability to manage connectivity and maintain communication.

Internet connectivity is crucial for the operational efficiency and safety of the vehicle. The `test_internet_connection.cpp` script is designed to validate the vehicle's ability to sustain a stable internet connection and how the system responds to changes in connectivity.

#### Test Fixture: TestInternetConnection

This fixture sets up the necessary environment to simulate different levels of internet connectivity and monitor the system's response. It aims to ensure that the vehicle can manage its internet connection effectively under various conditions.

```cpp
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include "gem_navigation/StringStamped.h"

class TestInternetConnection : public ::testing::Test {
protected:
    ros::NodeHandle nh;
    ros::Publisher internet_pub;
    ros::Subscriber status_sub;
    gem_navigation::StringStamped::ConstPtr last_status_msg;

    virtual void SetUp() override {
        internet_pub = nh.advertise<std_msgs::Int32>("/internet_signal_strength", 1);
        status_sub = nh.subscribe("/car_status", 1, &TestInternetConnection::statusCallback, this);
    }

    void publishInternetSignalStrength(int strength) {
        std_msgs::Int32 msg;
        msg.data = strength;
        internet_pub.publish(msg);
    }

    void statusCallback(const gem_navigation::StringStamped::ConstPtr& msg) {
        last_status_msg = msg;
    }
};
```

**Key Experiments:**

- **InternetConnectivityErrorState**: This test evaluates the system's response to changes in internet connectivity, focusing particularly on how it manages transitions between connected and disconnected states based on the signal strength.

```cpp
TEST_F(TestInternetConnection, InternetConnectivityErrorState) {
    // Simulate various internet connectivity scenarios and evaluate system response
    simulateInternetSignalStrengthChange(100, 14); // Strong signal for 14 seconds, expect ONLINE state
    waitForStatus("ONLINE", ros::Duration(1));

    simulateInternetSignalStrengthChange(10, 16); // Weak signal for 16 seconds, expect OFFLINE state
    waitForStatus("OFFLINE", ros::Duration(1));

    // Test values representing connectivity interruptions
    publishInternetSignalStrength(-1); // Signal loss, expect OFFLINE state
    waitForStatus("OFFLINE", ros::Duration(1));
}
```

#### Conclusion

The internet connectivity tests are designed to ensure that the vehicle's communication system can effectively manage its internet connection under varying conditions. By simulating different levels of signal strength, this testing framework verifies the robustness of the vehicle's internet connectivity and its ability to maintain essential communication functions, crucial for operational safety and efficiency.







### Emergency Stop Functionality

- **Design and Implementation**: Implements an emergency stop mechanism that can be triggered manually or by other system components under critical conditions.
- **Challenges**: Ensuring the immediate and safe cessation of vehicle movement upon emergency stop signal reception.
- **Test Fixture**: Evaluates the system's response time and effectiveness in executing an emergency stop under various operational scenarios.

### Emergency Stop Signal

The emergency stop functionality is a critical safety feature, allowing the vehicle to halt immediately under dangerous conditions. The `test_emerg_stop.cpp` script is designed to verify this functionality thoroughly.

#### Test Fixture: TestEmergencyStop

This fixture sets up the testing environment by initializing ROS publishers and subscribers for simulating and monitoring the emergency stop signal's effect on the vehicle's operational state.

```cpp
class TestEmergencyStop : public ::testing::Test {
protected:
    ros::NodeHandle nh;
    ros::Publisher emergency_stop_pub;
    ros::Subscriber status_sub;
    gem_navigation::StringStamped::ConstPtr last_status_msg;

    virtual void SetUp() override {
        emergency_stop_pub = nh.advertise<std_msgs::Bool>("emergency_stop", 1);
        status_sub = nh.subscribe("/car_status", 1, &TestEmergencyStop::statusCallback, this);
    }

    void triggerEmergencyStop(bool activated) {
        std_msgs::Bool msg;
        msg.data = activated;
        emergency_stop_pub.publish(msg);
    }

    void statusCallback(const gem_navigation::StringStamped::ConstPtr& msg) {
        last_status_msg = msg;
    }
};
```

**Key Experiment: EmergencyStopActivation**

This experiment simulates the activation and subsequent deactivation of the emergency stop signal. It specifically tests the system's ability to transition to an ERROR state upon activation and return to a RUNNING state once the signal is deactivated.

```cpp
TEST_F(TestEmergencyStop, EmergencyStopActivation) {
    // Deactivate emergency stop
    triggerEmergencyStop(false);
    // Wait for system to return to RUNNING state 
    waitForStatus("RUNNING", ros::Duration(5));

    // Trigger emergency stop
    triggerEmergencyStop(true);
    // Wait for ERROR state after emergency stop is triggered
    waitForStatus("ERROR", ros::Duration(5));
}
```

This test ensures that the vehicle's sensor data management system can correctly interpret emergency stop signals, prioritizing safety by immediately transitioning to an error state. This direct integration with the sensor manager file demonstrates the cohesive operation of the vehicle's safety mechanisms, ensuring that the system can respond promptly to critical situations.

#### Conclusion

The emergency stop test scenario emphasizes the system's capability to prioritize safety through rapid state transitions in response to emergency signals. It showcases the robustness of the sensor data management system in handling critical safety features effectively.




### Integration and Performance

The integration of the sensor data management system with the planner node and path-tracking controller demonstrates the system's capability to make informed decisions and execute precise navigation tasks based on real-time sensor inputs and environmental conditions. Testing under simulated real-world scenarios has proven the system's robustness, adaptability, and reliability in ensuring the autonomous vehicle's safety and operational efficiency.

1. Battery test scenario vehicle performance:

   **Response Time:** 0.011 seconds

   **Recovery Time:** 0.023 seconds

   **Robustness.** Sucessful unit tests indicate robustness to failure scenarios:
      - Sensor data manager output throughout unit test:
         ```bash
         [ERROR] [1709070052.001338431, 95.627000000]: Battery Level -  49.Battery is less than or equal to 50./nERROR STATE
         [ERROR] [1709070052.276803182, 95.830000000]: Battery Level -  -1.The Battery charge range is outside the sensor range [0, 100]./nERROR STATE
         [ERROR] [1709070052.412165396, 95.931000000]: Battery Level -  101.The Battery charge range is outside the sensor range [0, 100]./nERROR STATE
         ```
      - Unit test output:
         ```bash
         [==========] Running 1 test from 1 test suite.
         [----------] Global test environment set-up.
         [----------] 1 test from TestBatterySensor
         [ RUN      ] TestBatterySensor.BatteryErrorState
         [       OK ] TestBatterySensor.BatteryErrorState (50402 ms)
         [----------] 1 test from TestBatterySensor (50402 ms total)
         [----------] Global test environment tear-down
         [==========] 1 test from 1 test suite ran. (50402 ms total)
         [  PASSED  ] 1 test.
         ```

2. Temperature test scenario vehicle performance:

   **Response Time:** 0.035 seconds

   **Recovery Time:** 0.010 seconds

   **Robustness.** Sucessful unit tests indicate robustness to failure scenarios:
      - Sensor data manager output throughout unit test:
         ```bash
         [ERROR] [1709070750.959811128, 602.334000000]: Temperature -  55C.Temperature is greater than or equal to 55./n ERROR STATE
         [ERROR] [1709070751.097359637, 602.433000000]: Temperature -  60C.Temperature is greater than or equal to 55./n ERROR STATE
         [ WARN] [1709070752.500410553, 603.434000000]: Temperature -  30C.Temperature is less than 55./n RUNNING STATE
         ```
      - Unit test output:
         ```bash
         [==========] Running 1 test from 1 test suite.
         [----------] Global test environment set-up.
         [----------] 1 test from TestTempSensor
         [ RUN      ] TestTempSensor.TemperatureErrorState
         [       OK ] TestTempSensor.TemperatureErrorState (45735 ms)
         [----------] 1 test from TestTempSensor (45735 ms total)
         [----------] Global test environment tear-down
         [==========] 1 test from 1 test suite ran. (45735 ms total)
         [  PASSED  ] 1 test.
         ```
3. GPS accuracy test scenario vehicle performance:

   **Response Time:** 0.075 seconds

   **Recovery Time:** 0.068 seconds

   **Robustness.** Sucessful unit tests indicate robustness to failure scenarios:
      - Sensor data manager output throughout unit test:
         ```bash
         [ERROR] [1709072148.552685305, 1617.952000000]: GPS ACCURACY -  -1.The GPS accuracy is outside the sensor range [0, 1000]./n ERROR STATE
         [ERROR] [1709072148.552870834, 1617.952000000]: GPS ACCURACY more than 200 for 15s./nERROR STATE
         [ERROR] [1709072148.553045918, 1617.952000000]: GPS ACCURACY -  1001.The GPS accuracy is outside the sensor range [0, 1000]./n ERROR STATE
         ```
      - Unit test output:
         ```bash
         [==========] Running 1 test from 1 test suite.
         [----------] Global test environment set-up.
         [----------] 1 test from TestGPSSensor
         [ RUN      ] TestGPSSensor.GPSErrorState
         [       OK ] TestGPSSensor.GPSErrorState (42916 ms)
         [----------] 1 test from TestGPSSensor (42916 ms total)

         [----------] Global test environment tear-down
         [==========] 1 test from 1 test suite ran. (42916 ms total)
         [  PASSED  ] 1 test.
         ```
4. Signal strength test scenario vehicle performance:

   **Response Time:** 0.002 seconds

   **Recovery Time:** 0.002 seconds

   **Robustness.** Sucessful unit tests indicate robustness to failure scenarios:
      - Sensor data manager output throughout unit test:
         ```bash
         [ WARN] [1709072409.826065645, 1812.933000000]: Stable connection./nRUNNING STATE
         [ERROR] [1709072502.550117712, 1879.732000000]: Internet not connected for 10s./n ERROR STATE
         [ERROR] [1709072547.808507865, 1912.749000000]: Internet low for 20s./n ERROR STATE
         ```
      - Unit test output:
         ```bash
         [==========] Running 1 test from 1 test suite.
         [----------] Global test environment set-up.
         [----------] 1 test from TestInternetConnection
         [ RUN      ] TestInternetConnection.InternetConnectivityErrorState
         [       OK ] TestInternetConnection.InternetConnectivityErrorState (74433 ms)
         [----------] 1 test from TestInternetConnection (74433 ms total)

         [----------] Global test environment tear-down
         [==========] 1 test from 1 test suite ran. (74434 ms total)
         [  PASSED  ] 1 test.

         ```
5. Emergency button press test scenario vehicle performance:

   **Response Time:** 0.002 seconds

   **Recovery Time:** 0.003 seconds

   **Robustness.** Sucessful unit tests indicate robustness to failure scenarios:
      - Sensor data manager output throughout unit test:
         ```bash
         [ERROR] [1709073114.142565331, 2324.309000000]: Emergency status -  1. Emergency button is pressed./nERROR STATE
         ```
      - Unit test output:
         ```bash
         [==========] Running 1 test from 1 test suite.
         [----------] Global test environment set-up.
         [----------] 1 test from TestEmergencyStop
         [ RUN      ] TestEmergencyStop.EmergencyStopActivation
         [       OK ] TestEmergencyStop.EmergencyStopActivation (7141 ms)
         [----------] 1 test from TestEmergencyStop (7141 ms total)
         [----------] Global test environment tear-down
         [==========] 1 test from 1 test suite ran. (7141 ms total)
         [  PASSED  ] 1 test.

         ```

### Conclusion

This comprehensive testing and development approach ensures that the autonomous vehicle system is well-equipped to handle real-world challenges, providing a solid foundation for further development and real-world deployment. The system's architecture allows for easy expansion and integration of additional sensors and functionalities, paving the way for advanced autonomous vehicle capabilities.


## Evaluation

This project showcases the development of a high-level management system for a simulated robot vehicle. It emphasizes modularity, scalability, and real-time operation, providing a robust framework for future enhancements and integration with real-world applications.
