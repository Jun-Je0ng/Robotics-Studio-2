#!/usr/bin/env python3
"""
plastic_detections_translator_calibrated.py
============================================
Drop-in replacement for plastic_detections_translator that adds an optional
XY affine correction layer fitted from real robot measurements.

NORMAL OPERATION
----------------
If  config/xy_correction.json  exists next to this script (or at
XY_CORRECTION_FILE below), the 6-parameter affine correction is applied to
every detected XY position before publishing.  If the file is absent the node
behaves identically to the plain translator.

CALIBRATION PROCEDURE  (9-point grid)
--------------------------------------
1. Launch the system as normal (real robot, freedrive enabled, OBB node running).
2. Choose a flat reference object — a 600 ml water bottle LYING FLAT is ideal:
   - Representative of actual sorted objects.
   - Large enough for consistent OBB detection.
   - Lying flat keeps object height out of the error (≤5 cm rule).
3. Place the bottle at position 1 on the tray (see grid below).
4. Wait for the OBB to report a stable detection (watch the terminal).
5. Use freedrive to hover the gripper_tcp DIRECTLY above the bottle centre.
6. From a second terminal, call:
       ros2 service call /xy_correction/capture std_srvs/srv/Trigger {}
   The node records: OBB-reported XY (latest detection) + EEF XY (from TF2).
7. Move the bottle to the next position and repeat steps 4-6 for all 9 points.
8. Compute and save the correction:
       ros2 service call /xy_correction/finish std_srvs/srv/Trigger {}
   The node fits an affine correction, prints per-point residuals, saves the
   file, and starts applying the correction immediately (no restart needed).

Suggested 9-point grid (values in robot base frame — adjust to your tray):

    (X_MIN, Y_MAX)   (  0  , Y_MAX)   (X_MAX, Y_MAX)   ← far  row
    (X_MIN,  mid )   (  0  ,  mid )   (X_MAX,  mid )   ← mid  row
    (X_MIN, Y_MIN)   (  0  , Y_MIN)   (X_MAX, Y_MIN)   ← near row

Other service calls:
    /xy_correction/reset   — discard all collected points and start over
    /xy_correction/status  — print how many points collected so far

CORRECTION MODEL
----------------
6-parameter affine per axis (handles offset, scale, skew, and XY coupling):

    x_real = a11·x_obb + a12·y_obb + b1
    y_real = a21·x_obb + a22·y_obb + b2

Fitted by least squares.  Needs ≥3 non-collinear points; 9 gives a stable fit.
"""

import os
import json
import numpy as np
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from std_srvs.srv import Trigger

from tf2_ros import Buffer, TransformListener, LookupException, \
                   ConnectivityException, ExtrapolationException

from object_msgs.msg import Object, ObjectArray

# ──────────────────────────────────────────────────────────────────────────────
# CONFIG — mirror of plain translator
# ──────────────────────────────────────────────────────────────────────────────

MIN_CONFIDENCE = 0.50
ROBOT_FRAME    = 'base'
EEF_FRAME      = 'gripper_tcp'   # TF2 frame for end-effector
MM_TO_M        = 1e-3

GRIPPER_TCP_OFFSET_M = 0.190

TRAY_X_MIN =  -0.220
TRAY_X_MAX =   0.220
TRAY_Y_MIN =  -0.440
TRAY_Y_MAX =  -0.160

# Path where the fitted correction is saved / loaded
XY_CORRECTION_FILE = os.path.join(
    os.path.dirname(__file__),
    '..', 'config', 'xy_correction.json'
)

# ──────────────────────────────────────────────────────────────────────────────


def mm_to_m(v: float) -> float:
    return v * MM_TO_M


def _apply_affine(coeffs_x, coeffs_y, x: float, y: float):
    """Apply fitted affine correction.  coeffs = [a1, a2, b] per axis."""
    xc = coeffs_x[0] * x + coeffs_x[1] * y + coeffs_x[2]
    yc = coeffs_y[0] * x + coeffs_y[1] * y + coeffs_y[2]
    return xc, yc


def _fit_affine(obb_xy, real_xy):
    """
    Fit 6-parameter affine from (obb_x, obb_y) → (real_x, real_y).
    Returns (coeffs_x, coeffs_y) where each is [a1, a2, b].
    """
    obb  = np.array(obb_xy)   # (N, 2)
    real = np.array(real_xy)  # (N, 2)
    A    = np.column_stack([obb[:, 0], obb[:, 1], np.ones(len(obb))])
    cx, _, _, _ = np.linalg.lstsq(A, real[:, 0], rcond=None)
    cy, _, _, _ = np.linalg.lstsq(A, real[:, 1], rcond=None)
    return cx.tolist(), cy.tolist()


# ──────────────────────────────────────────────────────────────────────────────

class CalibratedTranslator(Node):

    def __init__(self):
        super().__init__('plastic_detections_translator')

        # ── TF2 for EEF lookup ────────────────────────────────────────────────
        self._tf_buffer   = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # ── Latest OBB reading (metres, robot frame, pre-correction) ──────────
        self._latest_obb_lock = threading.Lock()
        self._latest_obb_xy: dict | None = None   # {cls: (x, y)}

        # ── Calibration state ─────────────────────────────────────────────────
        self._cal_lock  = threading.Lock()
        self._obb_pts   = []   # list of (x_obb_m, y_obb_m)
        self._real_pts  = []   # list of (x_eef_m, y_eef_m)

        # ── Correction coefficients (loaded from file or None) ─────────────────
        self._corr_x: list | None = None   # [a11, a12, b1]
        self._corr_y: list | None = None   # [a21, a22, b2]
        self._load_correction()

        # ── ROS interfaces ────────────────────────────────────────────────────
        self.sub = self.create_subscription(
            String, 'plastic_detections', self._detection_cb, 10)

        self.pub = self.create_publisher(
            ObjectArray, 'perception/objects', 10)

        self.create_service(
            Trigger, '/xy_correction/capture', self._svc_capture)
        self.create_service(
            Trigger, '/xy_correction/finish',  self._svc_finish)
        self.create_service(
            Trigger, '/xy_correction/reset',   self._svc_reset)
        self.create_service(
            Trigger, '/xy_correction/status',  self._svc_status)

        status = (
            f'correction loaded ({XY_CORRECTION_FILE})'
            if self._corr_x is not None else 'no correction file — pass-through mode'
        )
        self.get_logger().info(
            f'Calibrated translator ready — {status}\n'
            '  Services: /xy_correction/capture  /finish  /reset  /status'
        )

    # ── Correction file ───────────────────────────────────────────────────────

    def _load_correction(self):
        path = os.path.abspath(XY_CORRECTION_FILE)
        if not os.path.exists(path):
            self.get_logger().info(f'No correction file at {path} — running uncorrected.')
            return
        try:
            with open(path) as f:
                data = json.load(f)
            self._corr_x = data['coeffs_x']
            self._corr_y = data['coeffs_y']
            self.get_logger().info(
                f'XY correction loaded from {path}  '
                f'(fitted from {data.get("n_points","?")} points, '
                f'avg residual {data.get("avg_residual_mm","?"):.1f} mm)'
            )
        except Exception as e:
            self.get_logger().error(f'Failed to load correction: {e}')

    def _save_correction(self, coeffs_x, coeffs_y, obb_pts, real_pts):
        # Compute residuals for the report
        residuals = []
        for (ox, oy), (rx, ry) in zip(obb_pts, real_pts):
            px, py = _apply_affine(coeffs_x, coeffs_y, ox, oy)
            residuals.append(np.hypot(px - rx, py - ry) * 1000)  # mm

        data = {
            'coeffs_x':          coeffs_x,
            'coeffs_y':          coeffs_y,
            'n_points':          len(obb_pts),
            'avg_residual_mm':   float(np.mean(residuals)),
            'max_residual_mm':   float(np.max(residuals)),
            'calibration_points': [
                {'obb_x': ox, 'obb_y': oy, 'real_x': rx, 'real_y': ry,
                 'residual_mm': round(r, 2)}
                for (ox, oy), (rx, ry), r in zip(obb_pts, real_pts, residuals)
            ],
        }

        path = os.path.abspath(XY_CORRECTION_FILE)
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, 'w') as f:
            json.dump(data, f, indent=2)
        return data, residuals

    # ── Detection callback ────────────────────────────────────────────────────

    def _detection_cb(self, msg: String):
        try:
            detections = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f'JSON parse error: {e}')
            return

        stamp     = self.get_clock().now().to_msg()
        obj_array = ObjectArray()
        obj_array.header.frame_id = ROBOT_FRAME
        obj_array.header.stamp    = stamp

        # Collect per-class OBB XY (pre-correction, metres) for calibration capture
        latest_obb = {}

        for i, det in enumerate(detections):
            try:
                obj, raw_x, raw_y = self._convert(det, stamp)
            except (KeyError, TypeError) as e:
                self.get_logger().warn(f'Skipping detection [{i}]: {e}')
                continue

            if obj is None:
                continue

            cls = det['classification']['class']
            latest_obb[cls] = (raw_x, raw_y)
            obj_array.objects.append(obj)

        with self._latest_obb_lock:
            self._latest_obb_xy = latest_obb if latest_obb else None

        self.pub.publish(obj_array)
        if obj_array.objects:
            self.get_logger().debug(
                f'Published {len(obj_array.objects)} object(s)')
        else:
            self.get_logger().debug('Published empty ObjectArray (platform clear)')

    def _convert(self, det: dict, stamp):
        """
        Returns (Object, raw_x_m, raw_y_m) where raw_x/y are pre-correction
        metres in robot frame.  Returns (None, 0, 0) if detection is dropped.
        """
        confidence = det['classification']['confidence']
        if confidence < MIN_CONFIDENCE:
            return None, 0.0, 0.0

        ml_class = det['classification']['class']
        pos = det['pose']['position']
        ori = det['pose']['orientation']

        # mm → m, apply TCP offset
        raw_x = mm_to_m(pos['x'])
        raw_y = mm_to_m(pos['y'])

        pose = Pose()
        pose.position.x = raw_x
        pose.position.y = raw_y
        pose.position.z = mm_to_m(pos['z']) + GRIPPER_TCP_OFFSET_M

        pose.orientation.x = -ori['qx']
        pose.orientation.y =  ori['qy']
        pose.orientation.z = -ori['qz']
        pose.orientation.w =  ori['qw']

        # ── Apply affine XY correction (if loaded) ───────────────────────────
        if self._corr_x is not None:
            cx, cy = _apply_affine(self._corr_x, self._corr_y, raw_x, raw_y)
            pose.position.x = cx
            pose.position.y = cy

        # ── Tray boundary clamp ──────────────────────────────────────────────
        clamped = False
        if TRAY_X_MIN is not None and pose.position.x < TRAY_X_MIN:
            pose.position.x = TRAY_X_MIN; clamped = True
        if TRAY_X_MAX is not None and pose.position.x > TRAY_X_MAX:
            pose.position.x = TRAY_X_MAX; clamped = True
        if TRAY_Y_MIN is not None and pose.position.y < TRAY_Y_MIN:
            pose.position.y = TRAY_Y_MIN; clamped = True
        if TRAY_Y_MAX is not None and pose.position.y > TRAY_Y_MAX:
            pose.position.y = TRAY_Y_MAX; clamped = True
        if clamped:
            self.get_logger().warn(
                f'  {ml_class} clamped to tray bounds: '
                f'({pose.position.x:.3f}, {pose.position.y:.3f})')

        dims = det['dimensions']
        dx   = mm_to_m(dims['dx_mm'])
        dy   = mm_to_m(dims['dy_mm'] / 2)
        dz   = mm_to_m(dims['dz_mm'] / 2) if 'dz_mm' in dims else dy

        obj = Object()
        obj.header.frame_id = ROBOT_FRAME
        obj.header.stamp    = stamp
        obj.pose            = pose
        obj.classification  = ml_class
        obj.dimensions      = [dx, dy, dz]

        self.get_logger().debug(
            f'  {ml_class} | raw=({raw_x:.3f},{raw_y:.3f}) '
            f'pub=({pose.position.x:.3f},{pose.position.y:.3f}) m | '
            f'conf={confidence:.2f}')

        return obj, raw_x, raw_y

    # ── EEF position from TF2 ─────────────────────────────────────────────────

    def _get_eef_xy(self):
        """
        Look up current EEF position in robot base frame.
        Returns (x_m, y_m) or raises if TF2 lookup fails.
        """
        try:
            tf = self._tf_buffer.lookup_transform(
                ROBOT_FRAME, EEF_FRAME,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0))
            return (tf.transform.translation.x,
                    tf.transform.translation.y)
        except (LookupException, ConnectivityException,
                ExtrapolationException) as e:
            raise RuntimeError(f'TF2 lookup failed: {e}')

    # ── Services ──────────────────────────────────────────────────────────────

    def _svc_capture(self, _req, resp):
        # Read latest OBB detection
        with self._latest_obb_lock:
            obb = self._latest_obb_xy

        if not obb:
            resp.success = False
            resp.message = 'No OBB detection available — make sure the bottle is visible.'
            self.get_logger().warn(resp.message)
            return resp

        # Use the first (or only) class detected — if multiple, user should
        # ensure only one object is on the tray during calibration
        cls_name, (ox, oy) = next(iter(obb.items()))
        if len(obb) > 1:
            self.get_logger().warn(
                f'Multiple detections ({list(obb.keys())}) — '
                f'using {cls_name}. Remove other objects during calibration.')

        # Read EEF position
        try:
            rx, ry = self._get_eef_xy()
        except RuntimeError as e:
            resp.success = False
            resp.message = str(e)
            self.get_logger().error(resp.message)
            return resp

        with self._cal_lock:
            self._obb_pts.append((ox, oy))
            self._real_pts.append((rx, ry))
            n = len(self._obb_pts)

        msg = (f'Point {n} captured — '
               f'OBB=({ox*1000:.1f},{oy*1000:.1f})mm  '
               f'EEF=({rx*1000:.1f},{ry*1000:.1f})mm  '
               f'raw_error=({(rx-ox)*1000:.1f},{(ry-oy)*1000:.1f})mm')
        self.get_logger().info(msg)
        resp.success = True
        resp.message = msg
        return resp

    def _svc_finish(self, _req, resp):
        with self._cal_lock:
            obb_pts  = list(self._obb_pts)
            real_pts = list(self._real_pts)

        if len(obb_pts) < 3:
            resp.success = False
            resp.message = (f'Need at least 3 points, have {len(obb_pts)}. '
                            f'Capture more points first.')
            self.get_logger().warn(resp.message)
            return resp

        self.get_logger().info(f'Fitting correction from {len(obb_pts)} points...')
        coeffs_x, coeffs_y = _fit_affine(obb_pts, real_pts)

        try:
            data, residuals = self._save_correction(
                coeffs_x, coeffs_y, obb_pts, real_pts)
        except Exception as e:
            resp.success = False
            resp.message = f'Failed to save correction: {e}'
            self.get_logger().error(resp.message)
            return resp

        # Apply immediately
        self._corr_x = coeffs_x
        self._corr_y = coeffs_y

        # Print report
        self.get_logger().info('─── XY Correction fitted ───────────────────────')
        for i, (pt, r) in enumerate(zip(data['calibration_points'], residuals)):
            self.get_logger().info(
                f'  [{i+1}] OBB=({pt["obb_x"]*1000:.1f},{pt["obb_y"]*1000:.1f})mm '
                f'EEF=({pt["real_x"]*1000:.1f},{pt["real_y"]*1000:.1f})mm '
                f'residual={r:.1f}mm')
        self.get_logger().info(
            f'  avg residual: {data["avg_residual_mm"]:.1f}mm  '
            f'max: {data["max_residual_mm"]:.1f}mm')
        self.get_logger().info(
            f'  Correction saved to {os.path.abspath(XY_CORRECTION_FILE)}')
        self.get_logger().info('────────────────────────────────────────────────')

        resp.success = True
        resp.message = (f'Correction fitted from {len(obb_pts)} points. '
                        f'Avg residual: {data["avg_residual_mm"]:.1f}mm. '
                        f'Applied immediately.')
        return resp

    def _svc_reset(self, _req, resp):
        with self._cal_lock:
            n = len(self._obb_pts)
            self._obb_pts.clear()
            self._real_pts.clear()
        msg = f'Cleared {n} calibration point(s).'
        self.get_logger().info(msg)
        resp.success = True
        resp.message = msg
        return resp

    def _svc_status(self, _req, resp):
        with self._cal_lock:
            n = len(self._obb_pts)
        corr = 'loaded' if self._corr_x is not None else 'none'
        msg = f'{n} point(s) collected so far. Correction: {corr}.'
        self.get_logger().info(msg)
        resp.success = True
        resp.message = msg
        return resp


# ──────────────────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = CalibratedTranslator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
