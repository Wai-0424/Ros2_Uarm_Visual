# Running the Swift Pro Robot Arm with ROS 2

I have prepared the project to run with the real Swift Pro robot arm.

## Prerequisites

1.  **Connect the Robot**: Ensure the robot is connected via USB and powered on.
2.  **Serial Port**: The robot should be detected as `/dev/ttyACM0`.

## Steps to Run

1.  **Grant Serial Port Permissions**:
    You need to grant read/write permissions to the serial port. Run this command in your terminal:
    ```bash
    sudo chmod 666 /dev/ttyACM0
    ```

2.  **Source the Workspace**:
    Load the ROS 2 environment for this workspace:
    ```bash
    source install/setup.bash
    ```

3.  **Launch the Real Robot Control**:
    I have created a new launch file `real_robot.launch.py` that starts:
    -   Hardware interface nodes (`swiftpro_write_node`, etc.)
    -   MoveIt Motion Planning Framework
    -   RViz for visualization and control

    Run the launch file:
    ```bash
    ros2 launch swiftpro real_robot.launch.py
    ```

## Usage

-   **RViz**: You should see the robot model in RViz.
-   **Motion Planning**: You can use the MoveIt MotionPlanning plugin in RViz to drag the end-effector to a desired pose and click "Plan and Execute" to move the real robot.

## Troubleshooting

-   If the robot doesn't move, check if the power adapter is connected and the emergency stop is not engaged.
-   If you see permission errors, make sure you ran the `chmod` command.
