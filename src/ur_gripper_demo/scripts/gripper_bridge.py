#!/usr/bin/env python3
"""
Gripper Bridge Node
===================
Acts as a GripperCommand action SERVER on /gripper_action_controller/gripper_cmd.
Translates goals into Float64MultiArray commands on /finger_width_controller/commands.

WHY THIS EXISTS
---------------
On the real robot the gripper_action_controller is not configured / does not start
(the OnRobot RG driver exposes a raw position controller instead).  This bridge
presents the standard GripperCommand action interface to MoveIt while doing its
own completion detection via /joint_states.

GRASP STRATEGY
--------------
Send the command, wait GRASP_SETTLE_S seconds, read the current finger width,
command that width back (hold), return stalled=True.

No direction detection, no goal-reaching checks.  The reactive pick loop in
motion_controller_reactive handles the "was anything actually grasped?" question
— if the object wasn't picked it stays on the platform and perception sees it
again next iteration.

NOTE: Do NOT run this node in simulation — in sim the gripper_action_controller
is properly configured, and two action servers on the same topic will conflict.
Gate with condition=IfCondition(PythonExpression(["'", sim, "' == 'false'"]))
in the launch file (already done in ur_moveit.launch.py).
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup

from control_msgs.action import GripperCommand
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

import threading
import time

# ── tunables ──────────────────────────────────────────────────────────────────
GRASP_SETTLE_S = 2.0    # s  – wait after command before holding
POLL_INTERVAL  = 0.05   # s  – joint state poll interval
GRIPPER_JOINT  = "finger_width"
GRASP_CLOSED_THRESH = 0.010
# ──────────────────────────────────────────────────────────────────────────────


class GripperBridge(Node):
    def __init__(self):
        super().__init__("gripper_bridge")

        self._cb_group = ReentrantCallbackGroup()

        self._cmd_pub = self.create_publisher(
            Float64MultiArray,
            "/finger_width_controller/commands",
            10,
        )

        self._current_pos = None
        self._js_lock     = threading.Lock()
        self.create_subscription(
            JointState,
            "/joint_states",
            self._joint_state_cb,
            10,
            callback_group=self._cb_group,
        )

        self._action_server = ActionServer(
            self,
            GripperCommand,
            "/gripper_action_controller/gripper_cmd",
            execute_callback=self._execute_cb,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
            callback_group=self._cb_group,
        )

        self.get_logger().info(
            "Gripper bridge ready — serving /gripper_action_controller/gripper_cmd"
        )

    # ── joint state callback ──────────────────────────────────────────────────
    def _joint_state_cb(self, msg: JointState):
        try:
            idx = msg.name.index(GRIPPER_JOINT)
            with self._js_lock:
                if idx < len(msg.position):
                    p = msg.position[idx]
                    if not math.isnan(p):
                        self._current_pos = p
        except ValueError:
            pass

    def _get_pos(self):
        with self._js_lock:
            return self._current_pos

    # ── action callbacks ──────────────────────────────────────────────────────
    def _goal_cb(self, _goal_request):
        return GoalResponse.ACCEPT

    def _cancel_cb(self, _goal_handle):
        return CancelResponse.ACCEPT

    def _execute_cb(self, goal_handle):
        target = goal_handle.request.command.position
        effort = goal_handle.request.command.max_effort

        self.get_logger().info(
            f"Gripper goal: target={target:.4f} m  effort={effort:.1f} N"
        )

        self._send_command(target)

        # Wait for the gripper to settle (reach stall or target position)
        deadline = time.time() + GRASP_SETTLE_S
        feedback = GripperCommand.Feedback()

        while time.time() < deadline:
            if goal_handle.is_cancel_requested:
                cur = self._get_pos()
                result = GripperCommand.Result()
                result.stalled      = False
                result.reached_goal = False
                result.position     = cur if cur is not None else target
                result.effort       = effort
                goal_handle.canceled()
                return result

            time.sleep(POLL_INTERVAL)
            cur = self._get_pos()
            if cur is not None:
                feedback.position     = cur
                feedback.stalled      = False
                feedback.reached_goal = False
                goal_handle.publish_feedback(feedback)

        # Settle complete — hold at current width so controller stops fighting  # safe/no verification 
        # cur = self._get_pos()
        # hold_pos = cur if cur is not None else target
        # self.get_logger().info(
        #     f"Settle complete — holding at {hold_pos:.4f} m"
        # )
        # self._send_command(hold_pos)

        # result = GripperCommand.Result()
        # result.stalled      = True
        # result.reached_goal = False
        # result.position     = hold_pos
        # result.effort       = effort
        # goal_handle.succeed()
        # return result


        # Settle complete — hold at current width so controller stops fighting
        cur      = self._get_pos()
        hold_pos = cur if cur is not None else target
        
        is_close_command = target < GRASP_CLOSED_THRESH
        grasped          = hold_pos > GRASP_CLOSED_THRESH

        self.get_logger().info(
            f"Settle complete — hold={hold_pos:.4f} m  "
            f"close_cmd={is_close_command}  grasped={grasped}"
        )
        self._send_command(hold_pos)

        result              = GripperCommand.Result()
        result.position     = hold_pos
        result.effort       = effort
        result.stalled      = grasped if is_close_command else False
        result.reached_goal = (not grasped) if is_close_command else True
        goal_handle.succeed()
        return result

    def _send_command(self, position: float):
        msg = Float64MultiArray()
        msg.data = [position]
        self._cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GripperBridge()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()