#!/usr/bin/env python3

import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler
import tkinter as tk

# Nav2 Simple Commander
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

# Custom messages/services
from robot_butler_interfaces.msg import Order
from robot_butler_interfaces.srv import Cancel

############################################
#            Global Config & Vars
############################################
TIMEOUT_CONFIRMATION = 8.0   # 8s wait at kitchen/table
STATE_MACHINE_PERIOD = 0.5   # State machine iteration rate

SINGLE_TABLE_MODE = False
MULTIPLE_TABLE_MODE = False
SKIPPED_OR_CANCELED_ANY_TABLE = False  # If any table is skipped/canceled => final trip to kitchen

NAMED_LOCATIONS = {
    "home":    (-4.0,  1.5,  0.0),
    "kitchen": (-3.0,  6.0,  1.57),
    "table1":  (-3.2, -1.2,  -1.57),
    "table2":  (-0.8, -2.32,  0.0),
    "table3":  ( 2.57, -1.62, 1.57),
}

def create_pose_stamped(x, y, yaw, stamp):
    ps = PoseStamped()
    ps.header.frame_id = 'map'
    ps.header.stamp = stamp
    q = quaternion_from_euler(0.0, 0.0, yaw)
    ps.pose.position.x = x
    ps.pose.position.y = y
    ps.pose.orientation.x = q[0]
    ps.pose.orientation.y = q[1]
    ps.pose.orientation.z = q[2]
    ps.pose.orientation.w = q[3]
    return ps

class ButlerControlNode(Node):
    """
    Covers scenarios #3,#4,#5,#6,#7 with final logic:
      - If ALL tables are confirmed => direct home after final table (#5).
      - If ANY table is skipped/canceled => final trip to kitchen => wait => home (#6,#7).
      - "No confirm => home" at kitchen. 
    """

    def __init__(self):
        super().__init__('butler_control_node')
        self.get_logger().info("Butler Control Node Started.")

        self.state = "IDLE"

        self.orders_queue = []
        self.current_table = None

        self.cancel_flag = False
        self.kitchen_confirmed = False
        self.table_confirmed   = False

        self.kitchen_timer = None
        self.table_timer   = None
        self.kitchen_wait_initiated = False
        self.table_wait_initiated   = False

        global SKIPPED_OR_CANCELED_ANY_TABLE
        SKIPPED_OR_CANCELED_ANY_TABLE = False

        self.navigator = BasicNavigator()

        # Subscriptions/Services
        self.create_subscription(Order, '/order_topic', self.order_callback, 10)
        self.create_service(Cancel, '/cancel_topic', self.cancel_callback)

        # State machine stepping
        self.create_timer(STATE_MACHINE_PERIOD, self.state_machine_step)

        # Launch Tkinter in separate thread
        self.gui_thread = threading.Thread(target=self.start_gui, daemon=True)
        self.gui_thread.start()

    ###############################
    #   Order & Cancel
    ###############################
    def order_callback(self, msg: Order):
        new_tables = msg.table_ids
        self.get_logger().info(f"Received Order(s): {new_tables}")

        global SINGLE_TABLE_MODE, MULTIPLE_TABLE_MODE

        for t in new_tables:
            if t not in self.orders_queue:
                self.orders_queue.append(t)

        if len(self.orders_queue) == 1:
            SINGLE_TABLE_MODE = True
            MULTIPLE_TABLE_MODE = False
        elif len(self.orders_queue) > 1:
            SINGLE_TABLE_MODE = False
            MULTIPLE_TABLE_MODE = True

        self.get_logger().info(f"(DEBUG) SINGLE_TABLE_MODE={SINGLE_TABLE_MODE}, MULTIPLE_TABLE_MODE={MULTIPLE_TABLE_MODE}")
        self.get_logger().info(f"(DEBUG) Updated queue => {self.orders_queue}")

    def cancel_callback(self, request: Cancel, response):
        global SKIPPED_OR_CANCELED_ANY_TABLE
        if request.cancel_all:
            self.get_logger().warn("CANCEL ALL deliveries.")
            self.orders_queue.clear()
            self.cancel_flag = True
            SKIPPED_OR_CANCELED_ANY_TABLE = True
            response.success = True
        elif request.table_id:
            if request.table_id in self.orders_queue:
                self.get_logger().warn(f"Cancel table: {request.table_id}")
                self.orders_queue.remove(request.table_id)
                SKIPPED_OR_CANCELED_ANY_TABLE = True
                response.success = True
            else:
                self.get_logger().warn(f"⚠️ Table {request.table_id} not in queue => can't cancel.")
                response.success = False
        else:
            self.get_logger().warn("⚠️ Cancel request invalid (no table_id, no cancel_all).")
            response.success = False
        return response

    ###############################
    #         State Machine
    ###############################
    def state_machine_step(self):
        if self.cancel_flag and self.state not in ["IDLE", "RETURN_TO_HOME"]:
            self.handle_global_cancellation()
            return

        state_methods = {
            "IDLE": self.do_idle,
            "GO_TO_KITCHEN": self.do_go_to_kitchen,
            "WAIT_KITCHEN_CONFIRM": self.do_wait_kitchen_confirm,
            "CHECK_ORDERS_QUEUE": self.do_check_orders_queue,
            "GO_TO_TABLE": self.do_go_to_table,
            "WAIT_TABLE_CONFIRM": self.do_wait_table_confirm,
            "RETURN_TO_KITCHEN": self.do_return_to_kitchen,
            "RETURN_TO_HOME": self.do_return_to_home,
        }

        state_method = state_methods.get(self.state, self.do_idle)
        state_method()

    ###############################
    #   State Implementations
    ###############################
    def do_idle(self):
        if len(self.orders_queue) > 0:
            self.get_logger().info("(DEBUG) IDLE => GO_TO_KITCHEN")
            self.state = "GO_TO_KITCHEN"

    def do_go_to_kitchen(self):
        self.get_logger().info("(DEBUG) do_go_to_kitchen => reset kitchen_wait_initiated.")
        self.kitchen_wait_initiated = False

        # If there are NO remaining orders, go home instead of the kitchen
        if not self.orders_queue:
            self.get_logger().warn("(DEBUG) No orders left after cancel => Going HOME instead of kitchen.")
            self.state = "RETURN_TO_HOME"
            return

        x, y, yaw = NAMED_LOCATIONS["kitchen"]
        success, interrupt_state = self.go_to_pose(x, y, yaw, 'kitchen')
        if not success:
            self.get_logger().warn("(DEBUG) Nav to Kitchen failed => HOME.")
            self.state = "RETURN_TO_HOME"
            return
        if not interrupt_state:
            self.state = "WAIT_KITCHEN_CONFIRM"
        else:
            self.state = interrupt_state
            self.kitchen_confirmed = True

    def do_wait_kitchen_confirm(self):
        """No confirm => home, always."""
        if not self.kitchen_wait_initiated:
            self.get_logger().info("(DEBUG) WAIT_KITCHEN_CONFIRM => create 8s timer (no confirm => home).")
            self.kitchen_wait_initiated = True
            self.kitchen_confirmed = False
            self.kitchen_timer = self.create_timer(
                TIMEOUT_CONFIRMATION,
                self.kitchen_timeout_callback
            )
        else:
            self.get_logger().info("(DEBUG) Already in WAIT_KITCHEN_CONFIRM => no new timer.")
            print("Please press Confirmation Button")

    def kitchen_timeout_callback(self):
        global SKIPPED_OR_CANCELED_ANY_TABLE

        if self.kitchen_timer:
            self.kitchen_timer.cancel()
            self.kitchen_timer = None
        self.kitchen_wait_initiated = False

        self.get_logger().info("(DEBUG) Kitchen Timeout")

        if not self.kitchen_confirmed:
            self.get_logger().warn("(DEBUG) Kitchen confirm timed out => HOME.")
            # If this was final wait, we reset skip/cancel so we don't loop
            SKIPPED_OR_CANCELED_ANY_TABLE = False
            self.orders_queue.clear()
            self.state = "RETURN_TO_HOME"
        else:
            self.get_logger().info("(DEBUG) Kitchen Confirmed => CHECK_ORDERS_QUEUE")
            # If final wait, also reset skip/cancel
            SKIPPED_OR_CANCELED_ANY_TABLE = False
            self.kitchen_confirmed = False
            if len(self.orders_queue) > 0:
                self.state = "CHECK_ORDERS_QUEUE"

    def do_check_orders_queue(self):
        global SKIPPED_OR_CANCELED_ANY_TABLE

        self.get_logger().info(f"(DEBUG) MULTIPLE_TABLE_MODE={MULTIPLE_TABLE_MODE}, SKIPPED_OR_CANCELED_ANY_TABLE={SKIPPED_OR_CANCELED_ANY_TABLE}")

        if len(self.orders_queue) == 0:
            if MULTIPLE_TABLE_MODE and SKIPPED_OR_CANCELED_ANY_TABLE:
                self.get_logger().info("(DEBUG) No tables => multi skip/cancel => final => RETURN_TO_KITCHEN => WAIT => home.")
                self.state = "RETURN_TO_KITCHEN"
            else:
                self.get_logger().info("(DEBUG) All delivered => home.")
                self.state = "RETURN_TO_HOME"
        else:
            self.current_table = self.orders_queue[0]
            self.get_logger().info(f"(DEBUG) Next table => {self.current_table} => GO_TO_TABLE")
            self.state = "GO_TO_TABLE"

    def do_go_to_table(self):
        self.get_logger().info("(DEBUG) do_go_to_table => resetting table_wait_initiated.")
        self.table_wait_initiated = False

        if not self.current_table:
            self.get_logger().warn("(DEBUG) No current_table => HOME.")
            self.state = "RETURN_TO_HOME"
            return

        x, y, yaw = NAMED_LOCATIONS[self.current_table]
        success, interrupt_state = self.go_to_pose(x, y, yaw, self.current_table)
        if not success:
            self.get_logger().warn("(DEBUG) Nav to table failed => HOME.")
            self.state = "RETURN_TO_HOME"
            return
        if not interrupt_state:
            self.state = "WAIT_TABLE_CONFIRM"
        else:
            self.state = interrupt_state

    def do_wait_table_confirm(self):
        global SKIPPED_OR_CANCELED_ANY_TABLE

        if not self.table_wait_initiated:
            self.get_logger().info(f"(DEBUG) WAIT_TABLE_CONFIRM => create 8s timer for {self.current_table}")
            self.table_wait_initiated = True
            self.table_confirmed = False
            self.table_timer = self.create_timer(
                TIMEOUT_CONFIRMATION,
                self.table_timeout_callback
            )
        else:
            self.get_logger().info("(DEBUG) Already waiting table confirm => no new timer.")
            print("Please press Confirmation Button")

    def table_timeout_callback(self):
        global SKIPPED_OR_CANCELED_ANY_TABLE

        if self.table_timer:
            self.table_timer.cancel()
            self.table_timer = None
        self.table_wait_initiated = False

        if not self.table_confirmed:
            self.get_logger().warn(f"(DEBUG) Table {self.current_table} => no confirm => skipping.")
            if self.current_table in self.orders_queue:
                self.orders_queue.remove(self.current_table)
            SKIPPED_OR_CANCELED_ANY_TABLE = True

            if SINGLE_TABLE_MODE:
                self.state = "RETURN_TO_KITCHEN"
            else:
                self.state = "CHECK_ORDERS_QUEUE"
            self.current_table = None

        else:
            self.get_logger().info(f"(DEBUG) Table {self.current_table} confirmed => delivered.")
            if self.current_table in self.orders_queue:
                self.orders_queue.remove(self.current_table)
            self.current_table = None
            self.state = "CHECK_ORDERS_QUEUE"

    def do_return_to_kitchen(self):
        self.get_logger().info("(DEBUG) RETURN_TO_KITCHEN => new wait => then HOME.")
        # reset guard => fresh 8s
        self.kitchen_wait_initiated = False

        x, y, yaw = NAMED_LOCATIONS["kitchen"]
        success, interrupt_state = self.go_to_pose(x, y, yaw, 'kitchen_return')

        if not success:
            self.get_logger().warn("(DEBUG) Nav to kitchen failed => HOME.")
            self.state = "RETURN_TO_HOME"
            return

        self.state = "WAIT_KITCHEN_CONFIRM"

    def do_return_to_home(self):
        self.get_logger().info("(DEBUG) RETURN_TO_HOME => final => IDLE.")
        x, y, yaw = NAMED_LOCATIONS["home"]
        success, interrupt_state = self.go_to_pose(x, y, yaw, 'home')

        if not success:
            self.get_logger().warn("(DEBUG) Nav to home failed => Staying in RETURN_TO_HOME.")
            self.state = "RETURN_TO_HOME"
            return

        if not interrupt_state:
            self.state = "IDLE"
        else:
            self.state = interrupt_state

    ############################################
    #       Tkinter GUI for Confirm
    ############################################
    def start_gui(self):
        root = tk.Tk()
        root.title("Butler Node (Final Fix)")

        tk.Label(root, text="Robot Buttons", font=("Arial", 14, "bold")).pack(pady=10)
        tk.Label(root, text="Pick Place Buttons", font=("Arial", 14)).pack(pady=10)

        tk.Button(root, text="Kitchen Confirm", font=("Arial", 14), command=self.kitchen_confirm).pack(pady=10)
        tk.Button(root, text="Table Confirm",   font=("Arial", 14), command=self.table_confirm).pack(pady=10)

        tk.Label(root, text="Remote Buttons", font=("Arial", 14, "bold")).pack(pady=10)

        for table_num in range(1, 4):
            table_id = f"table{table_num}"
            tk.Label(root, text=f"Table {table_num}", font=("Arial", 14)).pack(pady=10)

            tk.Button(root, text=f"Table {table_num} Order", font=("Arial", 14),
                      command=getattr(self, f"table_place_order_{table_num}")).pack(pady=10)
            tk.Button(root, text=f"Table {table_num} Cancel", font=("Arial", 14),
                      command=getattr(self, f"table_cancel_order_{table_num}")).pack(pady=10)

        root.mainloop()

    def kitchen_confirm(self):
        global SKIPPED_OR_CANCELED_ANY_TABLE

        self.get_logger().info("(DEBUG) Kitchen Confirmed via Button!")
        self.kitchen_confirmed = True

        if self.kitchen_timer:
            self.kitchen_timer.cancel()
            self.kitchen_timer = None
        self.kitchen_wait_initiated = False

        # If final kitchen wait => reset skip/cancel so we don't loop
        SKIPPED_OR_CANCELED_ANY_TABLE = False
        self.state = "CHECK_ORDERS_QUEUE"

    def table_confirm(self):
        self.get_logger().info("(DEBUG) Table Confirmed via Button!")
        self.table_confirmed = True

        if self.table_timer:
            self.table_timer.cancel()
            self.table_timer = None
        self.table_wait_initiated = False

        if self.current_table in self.orders_queue:
            self.get_logger().info(f"(DEBUG) Removing {self.current_table} from queue => {self.orders_queue}")
            self.orders_queue.remove(self.current_table)
            self.get_logger().info(f"(DEBUG) Table {self.current_table} confirmed => delivered.")
        self.current_table = None
        self.state = "CHECK_ORDERS_QUEUE"

    def table_place_order_1(self):
        self.get_logger().info("(DEBUG) Table 1 Order Received!")
        new_tables = ['table1']
        self.get_logger().info(f"Received Order(s): {new_tables}")

        global SINGLE_TABLE_MODE, MULTIPLE_TABLE_MODE

        for t in new_tables:
            if t not in self.orders_queue:
                self.orders_queue.append(t)

        if len(self.orders_queue) == 1:
            SINGLE_TABLE_MODE = True
            MULTIPLE_TABLE_MODE = False
        elif len(self.orders_queue) > 1:
            SINGLE_TABLE_MODE = False
            MULTIPLE_TABLE_MODE = True

        self.get_logger().info(f"(DEBUG) SINGLE_TABLE_MODE={SINGLE_TABLE_MODE}, MULTIPLE_TABLE_MODE={MULTIPLE_TABLE_MODE}")
        self.get_logger().info(f"(DEBUG) Updated queue => {self.orders_queue}")

    def table_cancel_order_1(self):
        global SKIPPED_OR_CANCELED_ANY_TABLE
        self.get_logger().info("(DEBUG) Table 1 Order Cancelled!")
        table_id = 'table1'
        if table_id in self.orders_queue:
            self.get_logger().warn(f"Cancel table: {table_id}")
            self.orders_queue.remove(table_id)
            SKIPPED_OR_CANCELED_ANY_TABLE = True
        else:
            self.get_logger().warn(f"Table {table_id} not in queue => can't cancel.")

    def table_place_order_2(self):
        self.get_logger().info("(DEBUG) Table 2 Order Received!")
        new_tables = ['table2']
        self.get_logger().info(f"Received Order(s): {new_tables}")

        global SINGLE_TABLE_MODE, MULTIPLE_TABLE_MODE

        for t in new_tables:
            if t not in self.orders_queue:
                self.orders_queue.append(t)

        if len(self.orders_queue) == 1:
            SINGLE_TABLE_MODE = True
            MULTIPLE_TABLE_MODE = False
        elif len(self.orders_queue) > 1:
            SINGLE_TABLE_MODE = False
            MULTIPLE_TABLE_MODE = True

        self.get_logger().info(f"(DEBUG) SINGLE_TABLE_MODE={SINGLE_TABLE_MODE}, MULTIPLE_TABLE_MODE={MULTIPLE_TABLE_MODE}")
        self.get_logger().info(f"(DEBUG) Updated queue => {self.orders_queue}")

    def table_cancel_order_2(self):
        global SKIPPED_OR_CANCELED_ANY_TABLE
        self.get_logger().info("(DEBUG) Table 2 Order Cancelled!")
        table_id = 'table2'
        if table_id in self.orders_queue:
            self.get_logger().warn(f"Cancel table: {table_id}")
            self.orders_queue.remove(table_id)
            SKIPPED_OR_CANCELED_ANY_TABLE = True
        else:
            self.get_logger().warn(f"Table {table_id} not in queue => can't cancel.")

    def table_place_order_3(self):
        self.get_logger().info("(DEBUG) Table 3 Order Received!")
        new_tables = ['table3']
        self.get_logger().info(f"Received Order(s): {new_tables}")

        global SINGLE_TABLE_MODE, MULTIPLE_TABLE_MODE

        for t in new_tables:
            if t not in self.orders_queue:
                self.orders_queue.append(t)

        if len(self.orders_queue) == 1:
            SINGLE_TABLE_MODE = True
            MULTIPLE_TABLE_MODE = False
        elif len(self.orders_queue) > 1:
            SINGLE_TABLE_MODE = False
            MULTIPLE_TABLE_MODE = True

        self.get_logger().info(f"(DEBUG) SINGLE_TABLE_MODE={SINGLE_TABLE_MODE}, MULTIPLE_TABLE_MODE={MULTIPLE_TABLE_MODE}")
        self.get_logger().info(f"(DEBUG) Updated queue => {self.orders_queue}")

    def table_cancel_order_3(self):
        global SKIPPED_OR_CANCELED_ANY_TABLE
        self.get_logger().info("(DEBUG) Table 3 Order Cancelled!")
        table_id = 'table3'
        if table_id in self.orders_queue:
            self.get_logger().warn(f"Cancel table: {table_id}")
            self.orders_queue.remove(table_id)
            SKIPPED_OR_CANCELED_ANY_TABLE = True
        else:
            self.get_logger().warn(f"Table {table_id} not in queue => can't cancel.")

    ############################################
    #       Nav2 Simple Commander
    ############################################
    def go_to_pose(self, x, y, yaw, destination):
        """
        Move robot to (x, y, yaw) using Nav2 Simple Commander.

        Returns:
        - success (bool): Whether navigation was successful
        - interrupt_state (str | None): Next state if interrupted
        """

        ps = create_pose_stamped(x, y, yaw, self.get_clock().now().to_msg())
        self.navigator.goToPose(ps)

        self.get_logger().info(f"(DEBUG) Navigating to {destination} => x={x}, y={y}, yaw={yaw}")

        if self.orders_queue:
            first_table = self.orders_queue[0]
        else:
            first_table = None  # Explicitly set to None instead of string

        # Monitor Navigation Task
        while not self.navigator.isTaskComplete():
            if destination in ["home", "kitchen_return"] and not self.orders_queue:
                continue  # Keep waiting in loop

            if not self.orders_queue or first_table != self.orders_queue[0]:  # Order change detected
                self.get_logger().warn("(DEBUG) Orders changed mid-navigation! Cancelling task.")
                self.navigator.cancelTask()
                # Handle different destinations properly
                if destination == "kitchen" and not self.orders_queue:
                    return True, "RETURN_TO_HOME"
                elif destination == "home":
                    self.get_logger().info("(DEBUG) Going to Kitchen")
                    return True, "GO_TO_KITCHEN"
                elif destination == "kitchen_return":
                    self.get_logger().info("(DEBUG) Going to Home from Kitchen")
                    return True, "RETURN_TO_HOME"
                elif not self.orders_queue:
                    return True, "RETURN_TO_KITCHEN"
                else:
                    return True, "GO_TO_TABLE"

        # Handle final navigation state
        result = self.navigator.getResult()

        if result == TaskResult.SUCCEEDED:
            self.get_logger().info("(DEBUG) Navigation SUCCEEDED!")
            return True, None

        elif result == TaskResult.CANCELED:
            self.get_logger().warn("(DEBUG) Navigation CANCELED!")
            return False, "RETURN_TO_HOME"

        elif result == TaskResult.FAILED:
            self.get_logger().warn("(DEBUG) Navigation FAILED!")
            return False, None  # Stay in current state

        else:
            self.get_logger().warn("(DEBUG) Unknown navigation result!")
            return False, None

    ############################################
    #         Handle Global Cancel (#4)
    ############################################
    def handle_global_cancellation(self):
        global SKIPPED_OR_CANCELED_ANY_TABLE
        SKIPPED_OR_CANCELED_ANY_TABLE = True

        self.orders_queue.clear()

        if self.state == "GO_TO_KITCHEN":
            self.get_logger().warn("(DEBUG) Cancel en route kitchen => HOME.")
            self.state = "RETURN_TO_HOME"
        elif self.state == "GO_TO_TABLE":
            self.get_logger().warn("(DEBUG) Cancel en route table => KITCHEN => HOME.")
            self.state = "RETURN_TO_KITCHEN"
        else:
            self.get_logger().warn("(DEBUG) Global Cancel => HOME.")
            self.state = "RETURN_TO_HOME"

        self.cancel_flag = False

def main():
    rclpy.init()
    node = ButlerControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
