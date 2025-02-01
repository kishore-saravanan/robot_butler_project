![image](https://github.com/user-attachments/assets/96d88ac2-146a-43a0-a0a4-1c2d2f37da8a)# Robot Butler Project

## Overview

The **Robot Butler Project** is an autonomous solution designed for busy café environments. In this project, a TurtleBot3-based robot operates in a custom café world in Gazebo using the Nav2 navigation stack. The robot performs order delivery tasks by autonomously moving from its home base to the kitchen and then to customer tables. Order requests and cancellations are handled by dedicated buttons in the GUI, and the robot’s behavior is managed by an internal Finite State Machine (FSM).

## Repository Structure

robot_butler_project/ 

![image](https://github.com/user-attachments/assets/471ba929-4bc9-4d5f-b5ca-b60e9dfdfa79)


- **`robot_butler/launch/tb3_cafe_navigation.launch.py`**  
  Launches the Gazebo simulation using a custom café environment and starts the navigation system configured for TurtleBot3.

- **`robot_butler/robot_butler/butler_control_node.py`**  
  Contains the main behavior and control node. This node implements an FSM to manage navigation between key locations (home, kitchen, and tables) and provides a GUI that includes buttons for:
  - Confirming arrival at the kitchen and at tables.
  - Placing orders for each table.
  - Canceling orders for each table.
  
- **`robot_butler_interfaces`**  
  Contains custom message and service definitions used by the project.

## Features

The Robot Butler system performs the following tasks:

1. **Single Order Delivery**  
   - When an order is received with a table number, the robot moves from its home position to the kitchen, then to the specified table for delivery, and finally returns to home.
   - No confirmation is required at the kitchen or table unless specified.

2. **Waiting for Confirmation**  
   - If an order is received and no one attends the robot, it waits for confirmation (either at the kitchen or at the table) and returns home after a timeout.

3. **Kitchen and Table Confirmation with Timeouts**  
   - The robot waits at the kitchen for confirmation (if the food is not immediately available, for example) and will return home if the confirmation is not received within a set timeout.
   - Similarly, if the robot reaches a table and no confirmation is received, it will return to the kitchen (or home) following a timeout.

4. **Task Cancellation and Preemption**  
   - If a task is canceled while en route to a table, the robot returns to the kitchen first and then goes home.
   - If a task is canceled while en route to the kitchen, the robot immediately returns home.

5. **Handling Multiple Orders**  
   - When multiple orders are received, the robot first goes to the kitchen to collect the orders and then visits each table sequentially.
   - If no confirmation is received for one table, the robot skips that table and continues to the next.
   - If an order for a specific table is canceled, the robot skips that table and delivers to the remaining tables.
   - After the final delivery, the robot returns to the kitchen and then home.

## Finite State Machine (FSM) Approach

The robot’s behavior is managed by a Finite State Machine (FSM) implemented within the `butler_control_node.py`. The FSM transitions between various states based on events and timeouts. The primary states include:

- **IDLE**:  
  The robot waits for new orders.

- **GO_TO_KITCHEN**:  
  Upon receiving an order, the robot moves from home to the kitchen.

- **WAIT_KITCHEN_CONFIRM**:  
  Once at the kitchen, the robot waits for a confirmation that the food is and placed on robot (e.g., the food is prepared). If confirmation is not received within a set timeout, the robot returns home.

- **CHECK_ORDERS_QUEUE**:  
  After receiving confirmation, the robot checks the orders queue. If orders remain, it proceeds to deliver them one by one.

- **GO_TO_TABLE**:  
  The robot navigates from the kitchen to the designated table for delivery.

- **WAIT_TABLE_CONFIRM**:  
  At the table, the robot waits for confirmation of delivery. If no confirmation is received within the timeout, the robot returns to the kitchen (or home) as per the logic.

- **RETURN_TO_KITCHEN** and **RETURN_TO_HOME**:  
  These states manage the robot’s return journey, either after completing deliveries or when a task is canceled.

Preemption is handled during navigation by periodically checking a command queue. This allows the robot to interrupt its current navigation if, for example, an order is canceled or a new order arrives.

## GUI Button Functionalities

### Confirmation Buttons
- **Kitchen Confirm**  
  - **Functionality**: Click this button to confirm that the food is placed on the robot.
  - **Effect**: The robot’s FSM uses this confirmation to proceed from waiting at the kitchen to checking the orders queue, after which it will navigate to the next delivery or return home.

- **Table Confirm**  
  - **Functionality**: Click this button to confirm that the robot has delivered the order to a table.
  - **Effect**: The FSM marks the current table order as complete, removes it from the orders queue, and then either moves to the next order or returns home if all orders are finished.

### Remote Order and Cancellation Buttons (for Testing)
*Note: These buttons are also provided in the GUI for testing remote order placement and cancellation; however, in a production scenario, orders and cancellations might be handled automatically by other system components.*

- **Table Order Buttons** (for Table 1, Table 2, Table 3)  
  - **Functionality**: Simulate placing an order for a specific table.
  - **Effect**: Adds the specified table to the orders queue. The robot will then perform its delivery routine based on the current FSM logic.

- **Table Cancel Buttons** (for Table 1, Table 2, Table 3)  
  - **Functionality**: Simulate canceling an order for a specific table.
  - **Effect**: Removes the specified table from the orders queue, and if the robot is currently en route to that table, it preempts the navigation and handles the cancellation accordingly.

## Screenshots

### Environment Screenshot

![gazebo cafe world](https://github.com/user-attachments/assets/6edd1e4d-93d1-41e1-b250-ca0fc2471399)


### GUI Screenshot

![Screenshot from 2025-02-01 08-01-39](https://github.com/user-attachments/assets/7fb1c0d4-37c8-4222-a00e-007b6f7af504)

### NAvigation with GUI Input Screenshot

![image](https://github.com/user-attachments/assets/28726e05-c2c8-4357-a1e4-d7f71696a12c)


## Installation

### Prerequisites

- **ROS2** (e.g., ROS2 Humble)
- **Gazebo** for simulation (Custom Cafe World required)
- **TurtleBot3 Packages**
- **Nav2 Navigation Stack**
- **nav2_simple_commander**

### Building the Workspace

Clone the repository into your ROS2 workspace (e.g., `~/ros2_ws/src/`):

`cd ~/ros2_ws/src
git clone https://github.com/yourusername/robot_butler_project.git`

Build your workspace:

`cd ~/ros2_ws
colcon build --packages-select robot_butler robot_butler_interfaces`

Source your workspace:

`source install/setup.bash`


## Usage

### Launching the Simulation and Navigation

Launch the simulation (Gazebo with the custom café environment, TurtleBot3, and Nav2) with:

`ros2 launch robot_butler tb3_cafe_navigation.launch.py`

Running the Butler Behavior and Control GUI

In a new terminal (after sourcing your workspace), run:

`ros2 run robot_butler butler_control_node`

This will start the butler behavior node and display the GUI with confirmation buttons (as well as remote order and cancel buttons for testing).

(Note: If Gazebo is not launching properly run `pkill -9 gzserver && pkill -9 gzclient` and relaunch gazebo)

### Configuration and Customization

Custom Café Environment: Modify the Gazebo world file as needed.

Robot and Navigation Parameters: Adjust parameters in the launch file (tb3_cafe_navigation.launch.py) to customize starting positions and navigation settings.

GUI Behavior: The GUI is implemented in butler_control_node.py using Tkinter and can be customized by editing its layout and callback functions.
