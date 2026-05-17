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
own stall/completion detection via /joint_states.

STALL / GRIP DETECTION
-----------------------
The OnRobot RG driver on this setup publishes NaN for both the effort and
velocity fields of the finger_width joint, so neither can be used for grip
detection.  Instead the bridge uses POSITION-DELTA stall detection:

  If the finger_width position changes by less than MOTION_TOL over
  STALL_POLLS consecutive 50 ms polls, the gripper has physically stopped.

  If stopped short of the target while closing → object in gripper
    → result: stalled=True,  reached_goal=False
    → immediately hold at current position to silence the clicking

  |pos - target| < REACHED_TOL → reached target (open or free-space close)
    → result: stalled=False, reached_goal=True

  Exceeded TIMEOUT with neither condition met
    → result: stalled=True,  reached_goal=False  (conservative fallback)

  A SETTLE_POLLS grace period at the start prevents false stall detection
  before the fingers have begun to move.

OPEN vs CLOSE
-------------
Stall detection is only active when CLOSING (target < start_pos).
When OPENING, the bridge simply waits for the position to reach the target
and ignores any transient stall signal (fingers may briefly slow while
releasing a held object).

NOTE: Do NOT run this node in simulation — in sim the gripper_action_controller
is properly configured, and two action servers on the same topic will conflict.
Gate this node with   condition=IfCondition(PythonExpression(["'", sim, "' == 'false'"]))
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
REACHED_TOL   = 0.001   # m  – position within this of target → reached_goal
MOTION_TOL    = 0.0005  # m  – minimum per-poll displacement to count as "moving"
STALL_POLLS   = 6       # consecutive still polls before declaring stall (~300 ms)
SETTLE_POLLS  = 8       # ignore stall detection for first N polls (~400 ms grace)
POLL_INTERVAL = 0.05    # s  – how often to check joint states
TIMEOUT       = 5.0     # s  – absolute fallback; RG2 full stroke is ~3 s
GRIPPER_JOINT = "finger_width"
# ──────────────────────────────────────────────────────────────────────────────


class GripperBridge(Node):
    def __init__(self):
        super().__init__("gripper_bridge")

        self._cb_group = ReentrantCallbackGroup()

        # Publisher to the actual position controller
        self._cmd_pub = self.create_publisher(
            Float64MultiArray,
            "/finger_width_controller/commands",
            10,
        )

        # Subscriber for joint states
        self._current_pos = None
        self._js_lock     = threading.Lock()
        self.create_subscription(
            JointState,
            "/joint_states",
            self._joint_state_cb,
            10,
            callback_group=self._cb_group,
        )

        # Action server
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
            pass  # GRIPPER_JOINT not present in this message — ignore

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
            f"Gripper goal received: target={target:.4f} m, effort={effort:.1f}"
        )

        # Send the position command once; the controller handles the trajectory.
        self._send_command(target)

        result   = GripperCommand.Result()
        feedback = GripperCommand.Feedback()
        deadline = time.time() + TIMEOUT

        is_closing:  bool | None = None   # latched on first valid reading
        prev_pos:    float | None = None  # position at previous poll
        stall_count: int = 0              # consecutive still polls
        poll_count:  int = 0              # total polls for settle grace period

        while time.time() < deadline:

            # ── cancellation check ────────────────────────────────────────────
            if goal_handle.is_cancel_requested:
                self.get_logger().info("Gripper goal cancelled")
                cur = self._get_pos()
                result.stalled      = False
                result.reached_goal = False
                result.position     = cur if cur is not None else target
                result.effort       = effort
                goal_handle.canceled()
                return result

            time.sleep(POLL_INTERVAL)
            cur = self._get_pos()

            if cur is None:
                continue  # no joint state received yet

            poll_count += 1

            # Latch direction on first valid reading
            if is_closing is None:
                is_closing = target < cur
                self.get_logger().info(
                    f"Gripper direction: {'CLOSING' if is_closing else 'OPENING'} "
                    f"(start={cur:.4f} m → target={target:.4f} m)"
                )

            # ── reached target (open or free-space close) ─────────────────────
            if abs(cur - target) < REACHED_TOL:
                self.get_logger().info(
                    f"Gripper reached target: pos={cur:.4f} m"
                )
                result.stalled      = False
                result.reached_goal = True
                result.position     = cur
                result.effort       = effort
                goal_handle.succeed()
                return result

            # ── position-delta stall detection (closing only) ─────────────────
            # Not checked during the settle window or when opening.
            if is_closing and poll_count > SETTLE_POLLS:
                if prev_pos is not None:
                    delta = abs(cur - prev_pos)
                    if delta < MOTION_TOL:
                        stall_count += 1
                    else:
                        stall_count = 0  # still moving — reset counter

                    if stall_count >= STALL_POLLS:
                        self.get_logger().info(
                            f"Position-delta stall detected at pos={cur:.4f} m "
                            f"(target={target:.4f} m, delta={delta*1000:.2f} mm over last poll)"
                        )
                        # Hold here so the controller stops fighting the object.
                        self._send_command(cur)
                        result.stalled      = True
                        result.reached_goal = False
                        result.position     = cur
                        result.effort       = effort
                        goal_handle.succeed()
                        return result

            # Publish live feedback
            feedback.position     = cur
            feedback.stalled      = (stall_count >= STALL_POLLS)
            feedback.reached_goal = False
            goal_handle.publish_feedback(feedback)

            prev_pos = cur

        # ── timeout — absolute fallback ───────────────────────────────────────
        cur = self._get_pos()
        self.get_logger().warn(
            f"Gripper action timed out at pos={cur} "
            f"(target={target:.4f} m) — treating as stalled"
        )
        # Hold at current position to silence the clicking.
        if cur is not None:
            self._send_command(cur)
        result.stalled      = True
        result.reached_goal = False
        result.position     = cur if cur is not None else target
        result.effort       = effort
        goal_handle.succeed()
        return result

    # ── helpers ───────────────────────────────────────────────────────────────
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
