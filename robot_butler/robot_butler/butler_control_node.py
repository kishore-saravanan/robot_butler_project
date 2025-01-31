#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose

# Custom interfaces
from robot_butler_interfaces.msg import Order
from robot_butler_interfaces.srv import Cancel

from geometry_msgs.msg import Quaternion
from tf_transformations import quaternion_from_euler

###############################
#      Config and Helpers     #
###############################

TIMEOUT_CONFIRMATION = 5.0  # seconds to wait at kitchen/table for "confirmation"
STATE_MACHINE_PERIOD = 0.5   # seconds (timer callback frequency)

def euler_to_quaternion(yaw):
    """
    Convert a yaw (in radians) to a Quaternion for 2D navigation.
    We assume roll = pitch = 0.
    """
    q = quaternion_from_euler(0.0, 0.0, yaw)
    return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

# Named locations in your cafe or simulation:
NAMED_LOCATIONS = {
    "home":    (0.0,  0.0,  0.0),   # x=0, y=0, yaw=0
    "kitchen": (2.5,  3.0,  1.57),  # Just an example
    "table1":  (4.0,  3.2,  0.0),
    "table2":  (4.0, -1.0,  3.14),
    "table3":  (1.0, -2.0,  1.57),
    # Add or adjust as needed
}


class ButlerControlNode(Node):
    """
    A single node that implements a simple state machine for the robot butler:
      - Receives multiple orders from /order_topic (Order.msg)
      - Maintains a queue of tables to deliver
      - Waits for confirmations at kitchen/table
      - Handles cancellations via /cancel_topic (Cancel.srv)
    """

    def __init__(self):
        super().__init__('butler_control_node')
        self.get_logger().info("ButlerControlNode started.")

        # Internal data
        self.state = "IDLE"
        self.orders_queue = []   # list of table IDs to deliver
        self.current_table = None
        self.cancel_flag = False

        # We store whether we "timed out" at table or kitchen
        self.timeout_occurred = False

        # Nav2 action client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Subscribe to /order_topic
        self.create_subscription(Order, '/order_topic', self.order_callback, 10)

        # Service to handle cancellation
        self.create_service(Cancel, '/cancel_topic', self.cancel_callback)

        # State machine step timer
        self.create_timer(STATE_MACHINE_PERIOD, self.state_machine_step)

    ###############################
    #   Subscriptions / Services  #
    ###############################

    def order_callback(self, msg: Order):
        """
        Append new table orders to the queue.
        Example: msg.table_ids = ["table1", "table3"]
        """
        self.get_logger().info(f"Received new order(s): {msg.table_ids}")
        for table_id in msg.table_ids:
            self.orders_queue.append(table_id)

    def cancel_callback(self, request: Cancel, response):
        """
        Cancel service. We can handle partial or full cancellation.
          - If request.cancel_all == True, clear all deliveries.
          - Else, remove the specified table_id from the queue.
        """
        if request.cancel_all:
            self.get_logger().warn("Cancel ALL deliveries request received.")
            self.orders_queue.clear()
            # If currently en route, let's set a global cancel to break the state.
            self.cancel_flag = True
            response.success = True
        else:
            # Cancel only for the specified table
            if request.table_id:
                self.get_logger().warn(f"Cancel request for {request.table_id}")
                # Remove from queue if present
                if request.table_id in self.orders_queue:
                    self.orders_queue.remove(request.table_id)
                    self.get_logger().info(f"Removed {request.table_id} from the queue.")
                    response.success = True
                else:
                    self.get_logger().warn(f"{request.table_id} is not in the queue. Maybe already delivered or invalid.")
                    response.success = False
            else:
                # If neither table_id nor cancel_all is set, do nothing
                self.get_logger().warn("Cancel service called with no table_id and cancel_all=false.")
                response.success = False

        return response

    ###############################
    #      STATE MACHINE LOOP     #
    ###############################

    def state_machine_step(self):
        """
        Called periodically (every 0.5s by default) to step through states.
        Check for global cancel at the top-level; if canceled, handle scenario #4 logic.
        """
        # 1) Global Cancel Check
        if self.cancel_flag and self.state not in ["IDLE", "RETURN_TO_HOME"]:
            # We only handle immediate global cancel if not already heading home or idle
            self.handle_global_cancellation()
            return

        # 2) Switch on current state
        if self.state == "IDLE":
            self.do_idle()
        elif self.state == "GO_TO_KITCHEN":
            self.do_go_to_kitchen()
        elif self.state == "WAIT_KITCHEN_CONFIRM":
            self.do_wait_kitchen_confirm()
        elif self.state == "CHECK_ORDERS_QUEUE":
            self.do_check_orders_queue()
        elif self.state == "GO_TO_TABLE":
            self.do_go_to_table()
        elif self.state == "WAIT_TABLE_CONFIRM":
            self.do_wait_table_confirm()
        elif self.state == "RETURN_TO_KITCHEN":
            self.do_return_to_kitchen()
        elif self.state == "RETURN_TO_HOME":
            self.do_return_to_home()
        else:
            # Unknown or unimplemented state
            pass

    ###############################
    #      STATE IMPLEMENTATION   #
    ###############################

    def do_idle(self):
        """
        IDLE: Robot is at Home, doing nothing. We check if there's an order in queue.
        """
        if len(self.orders_queue) > 0:
            self.get_logger().info("[IDLE] Orders found, going to KITCHEN.")
            self.state = "GO_TO_KITCHEN"
        else:
            # Stay IDLE
            pass

    def do_go_to_kitchen(self):
        """
        Travel from Home (or anywhere) to the Kitchen.
        We do a synchronous Nav2 call. 
        """
        self.get_logger().info("[GO_TO_KITCHEN] Navigating to KITCHEN.")
        success = self.go_to_pose("kitchen")
        if not success:
            # If navigation fails, or canceled
            self.get_logger().warn("[GO_TO_KITCHEN] Navigation failed, returning home.")
            self.state = "RETURN_TO_HOME"
        else:
            self.state = "WAIT_KITCHEN_CONFIRM"

    def do_wait_kitchen_confirm(self):
        """
        WAIT at the Kitchen for confirmation or time out (Scenario #2, #3).
        If timed out, go HOME. Otherwise, proceed to check orders.
        """
        self.get_logger().info("[WAIT_KITCHEN_CONFIRM] Simulating waiting for kitchen confirmation.")
        start_time = time.time()
        self.timeout_occurred = True  # assume we'll time out unless we "simulate" early confirm

        # We'll poll for half TIMEOUT, then "pretend" we got confirmation.
        while (time.time() - start_time) < TIMEOUT_CONFIRMATION:
            # If a cancel flag triggers mid-wait:
            if self.cancel_flag:
                self.get_logger().warn("[WAIT_KITCHEN_CONFIRM] Canceled while waiting at kitchen.")
                self.handle_global_cancellation()
                return
            elapsed = time.time() - start_time
            if elapsed > (TIMEOUT_CONFIRMATION / 2.0):
                self.get_logger().info("[WAIT_KITCHEN_CONFIRM] Kitchen gave confirmation!")
                self.timeout_occurred = False
                break
            time.sleep(0.2)

        if self.timeout_occurred:
            self.get_logger().warn("[WAIT_KITCHEN_CONFIRM] Timed out => returning HOME.")
            self.state = "RETURN_TO_HOME"
        else:
            self.state = "CHECK_ORDERS_QUEUE"

    def do_check_orders_queue(self):
        """
        Multiple orders scenario (#5). 
        If no tables left, go HOME. Else pop next table and deliver.
        """
        if len(self.orders_queue) == 0:
            self.get_logger().info("[CHECK_ORDERS_QUEUE] No more tables. Returning HOME.")
            self.state = "RETURN_TO_HOME"
            return

        # Grab next table from queue
        self.current_table = self.orders_queue.pop(0)
        self.get_logger().info(f"[CHECK_ORDERS_QUEUE] Next table: {self.current_table}")
        self.state = "GO_TO_TABLE"

    def do_go_to_table(self):
        """
        Travel from kitchen to the table.
        If canceled en route => scenario #4: go to kitchen then home
        If nav fails => go home
        """
        if self.cancel_flag:
            self.get_logger().warn("[GO_TO_TABLE] We were canceled en route => handle global cancel.")
            self.handle_global_cancellation()
            return

        self.get_logger().info(f"[GO_TO_TABLE] Navigating to {self.current_table}.")
        success = self.go_to_pose(self.current_table)
        if not success:
            self.get_logger().warn("[GO_TO_TABLE] Navigation failed, returning HOME.")
            self.state = "RETURN_TO_HOME"
        else:
            self.state = "WAIT_TABLE_CONFIRM"

    def do_wait_table_confirm(self):
        """
        Wait at table for confirmation (Scenario #3).
         - If timed out => scenario #6: skip this table, proceed to next
         - Or scenario #3(b): if no confirm, go to kitchen before home
        For demonstration:
         - We assume if we "time out," we skip this table and go to RETURN_TO_KITCHEN.
         - If confirmed, we move on to next table (CHECK_ORDERS_QUEUE).
        """
        self.get_logger().info(f"[WAIT_TABLE_CONFIRM] Waiting at {self.current_table} for confirmation.")
        start_time = time.time()
        self.timeout_occurred = True

        while (time.time() - start_time) < TIMEOUT_CONFIRMATION:
            if self.cancel_flag:
                self.get_logger().warn("[WAIT_TABLE_CONFIRM] Cancelled while waiting at table.")
                self.handle_global_cancellation()
                return

            elapsed = time.time() - start_time
            if elapsed > (TIMEOUT_CONFIRMATION / 2.0):
                self.get_logger().info("[WAIT_TABLE_CONFIRM] Table confirmed receipt.")
                self.timeout_occurred = False
                break
            time.sleep(0.2)

        if self.timeout_occurred:
            self.get_logger().warn(f"[WAIT_TABLE_CONFIRM] Timed out at {self.current_table}. Skipping it.")
            # Scenario #3(b) or #6 => go to kitchen first
            self.state = "RETURN_TO_KITCHEN"
        else:
            # Confirmed => go see if there's more tables
            self.state = "CHECK_ORDERS_QUEUE"

    def do_return_to_kitchen(self):
        """
        Return from table to Kitchen, typically after a fail or no confirmation.
        Then from Kitchen => eventually we go home (Scenario #3(b) or #6).
        """
        self.get_logger().info("[RETURN_TO_KITCHEN] Going back to Kitchen.")
        success = self.go_to_pose("kitchen")
        if not success:
            self.get_logger().warn("[RETURN_TO_KITCHEN] Nav failed. Returning HOME anyway.")
        self.state = "RETURN_TO_HOME"

    def do_return_to_home(self):
        """
        Finally, go back to HOME. Clear any cancel flags and reset state to IDLE.
        """
        self.get_logger().info("[RETURN_TO_HOME] Navigating HOME.")
        success = self.go_to_pose("home")
        if success:
            self.get_logger().info("[RETURN_TO_HOME] Arrived HOME successfully.")

        # Reset data
        self.cancel_flag = False
        self.current_table = None
        self.state = "IDLE"

    ###############################
    #   Global Cancel Handling    #
    ###############################

    def handle_global_cancellation(self):
        """
        Scenario #4:
          - If canceled while going to Kitchen => go HOME
          - If canceled while going to Table => go KITCHEN then HOME
          - Or if canceled while waiting, handle accordingly
        """
        # Clear partial deliveries if we want to. 
        # Or you can keep the queue if it's a partial cancel, 
        # but here we interpret global cancel as "Stop everything."
        self.orders_queue.clear()

        if self.state == "GO_TO_KITCHEN":
            self.get_logger().warn("[GlobalCancel] Canceled en route to Kitchen => returning HOME.")
            self.state = "RETURN_TO_HOME"
        elif self.state == "GO_TO_TABLE":
            self.get_logger().warn("[GlobalCancel] Canceled en route to Table => go to KITCHEN => then HOME.")
            self.state = "RETURN_TO_KITCHEN"
        else:
            self.get_logger().warn("[GlobalCancel] Transitioning to HOME.")
            self.state = "RETURN_TO_HOME"

        self.cancel_flag = False

    ###############################
    #       Nav2 Integration      #
    ###############################

    def go_to_pose(self, location_name: str) -> bool:
        """
        Send a NavigateToPose goal to Nav2 for the named location.
        Blocks until the action completes or fails.
        Returns True if succeeded, False if canceled/failure.
        """
        if location_name not in NAMED_LOCATIONS:
            self.get_logger().error(f"Unknown location: {location_name}")
            return False

        (x, y, yaw) = NAMED_LOCATIONS[location_name]
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation = euler_to_quaternion(yaw)

        # Wait for Nav2 action server
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Nav2 Action Server not available!")
            return False

        self.get_logger().info(f"Sending Nav2 goal to {location_name}: (x={x}, y={y}, yaw={yaw})")
        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Nav2 goal was rejected.")
            return False

        self.get_logger().info("Nav2 goal accepted. Waiting for result...")
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)
        result_status = get_result_future.result().status

        # 4 => SUCCEEDED per action_msgs/GoalStatus
        if result_status == 4:
            self.get_logger().info("Navigation SUCCEEDED!")
            return True
        else:
            self.get_logger().warn(f"Navigation failed with status={result_status}")
            return False


def main(args=None):
    rclpy.init(args=args)
    node = ButlerControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
