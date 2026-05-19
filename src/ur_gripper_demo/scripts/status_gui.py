#!/usr/bin/env python3

import json
import threading
import time
from datetime import datetime

import tkinter as tk

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# ── Colour palette ────────────────────────────────────────────────────────────
C = {
    'bg':      '#0d1117',
    'surface': '#161b22',
    'card':    '#21262d',
    'border':  '#30363d',
    'text':    '#e6edf3',
    'muted':   '#8b949e',
    'green':   '#3fb950',
    'blue':    '#58a6ff',
    'cyan':    '#79c0ff',
    'red':     '#f85149',
    'yellow':  '#d29922',
    'orange':  '#f0883e',
    'purple':  '#bc8cff',
}


class StatusGui(Node):
    def __init__(self):
        super().__init__("status_gui")

        self.jog_active = False
        self.jog_thread = None

        self._latest_detections = []
        self._latest_grip_poses = []

        self.cmd_pub = self.create_publisher(String, "/motion_system/command", 10)

        # ── existing topics ──────────────────────────────────────────────────
        self.create_subscription(String, "/system_status",
                                 self._cb_sys_status,  10)
        self.create_subscription(String, "/robot_state",
                                 self._cb_robot_state, 10)
        self.create_subscription(String, "/event_log",
                                 self._cb_event_log,   10)
        self.create_subscription(String, "/motion_system/status",
                                 self._cb_mot_status,  10)
        self.create_subscription(String, "/motion_system/current_object",
                                 self._cb_cur_object,  10)

        # ── Rian's perception topics ──────────────────────────────────────────
        self.create_subscription(String, "/plastic_detections",
                                 self._cb_plastic_detections, 10)
        self.create_subscription(String, "/grip_pose",
                                 self._cb_grip_pose,          10)

        self._build()

    # ══════════════════════════════════════════════════════════════════════════
    # GUI Construction
    # ══════════════════════════════════════════════════════════════════════════

    def _build(self):
        self.root = tk.Tk()
        self.root.title("UR3e  ·  Control Panel")
        self.root.geometry("1380x800")
        self.root.resizable(False, False)
        self.root.configure(bg=C['bg'])

        self._build_header()
        self._build_body()
        self._build_estop_bar()

        self.root.after(100, self._spin_ros)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    # ── Header ────────────────────────────────────────────────────────────────

    def _build_header(self):
        hdr = tk.Frame(self.root, bg=C['surface'], height=54)
        hdr.pack(fill=tk.X)
        hdr.pack_propagate(False)

        tk.Label(hdr, text="UR3e  +  RG2   Control Panel",
                 font=("Helvetica", 15, "bold"),
                 bg=C['surface'], fg=C['text']).pack(side=tk.LEFT, padx=22, pady=14)

        self._online_label = tk.Label(hdr, text="●  ONLINE",
                                      font=("Helvetica", 10, "bold"),
                                      bg=C['surface'], fg=C['green'])
        self._online_label.pack(side=tk.RIGHT, padx=22)

    # ── Body ──────────────────────────────────────────────────────────────────

    def _build_body(self):
        body = tk.Frame(self.root, bg=C['bg'])
        body.pack(fill=tk.BOTH, expand=True, padx=16, pady=12)

        left = tk.Frame(body, bg=C['bg'], width=280)
        left.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 10))
        left.pack_propagate(False)

        mid = tk.Frame(body, bg=C['bg'])
        mid.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        right = tk.Frame(body, bg=C['bg'], width=360)
        right.pack(side=tk.LEFT, fill=tk.Y, padx=(10, 0))
        right.pack_propagate(False)

        self._build_left(left)
        self._build_mid(mid)
        self._build_perception_panel(right)

    def _build_left(self, p):
        # ── Sequence ──────────────────────────────────────────────
        self._section_label(p, "SEQUENCE")
        row = tk.Frame(p, bg=C['bg'])
        row.pack(fill=tk.X, pady=(0, 4))
        self._btn(row, "▶  START", "start",
                  bg=C['green'], fg=C['bg'], width=12).pack(side=tk.LEFT, padx=(0, 6))
        self._btn(row, "■  STOP", "stop",
                  bg=C['red'], fg='white', width=12).pack(side=tk.LEFT)

        self._btn(p, "⌂  Home", "home",
                  bg=C['card'], fg=C['text'], width=28).pack(fill=tk.X, pady=(4, 0))

        # ── Gripper ───────────────────────────────────────────────
        self._section_label(p, "GRIPPER")
        row2 = tk.Frame(p, bg=C['bg'])
        row2.pack(fill=tk.X, pady=(0, 4))
        self._btn(row2, "◁▷  Open", "open_gripper",
                  bg=C['blue'], fg=C['bg'], width=12).pack(side=tk.LEFT, padx=(0, 6))
        self._btn(row2, "▷◁  Close", "close_gripper",
                  bg=C['card'], fg=C['text'], width=12).pack(side=tk.LEFT)

        # ── Cartesian Jog ─────────────────────────────────────────
        self._section_label(p, "CARTESIAN JOG")

        jog = tk.Frame(p, bg=C['bg'])
        jog.pack(pady=(0, 6))

        js = dict(font=("Helvetica", 11, "bold"), width=5, height=2,
                  bg=C['card'], fg=C['text'], relief="flat", cursor="hand2",
                  activebackground=C['border'], activeforeground=C['text'],
                  borderwidth=0)

        tk.Button(jog, text="+Y", command=lambda: self._jog("+y"), **js).grid(row=0, column=1, padx=3, pady=3)
        tk.Button(jog, text="-X", command=lambda: self._jog("-x"), **js).grid(row=1, column=0, padx=3, pady=3)
        tk.Label (jog, text="·", bg=C['bg'], fg=C['border'],
                  font=("Helvetica", 18), width=5).grid(row=1, column=1)
        tk.Button(jog, text="+X", command=lambda: self._jog("+x"), **js).grid(row=1, column=2, padx=3, pady=3)
        tk.Button(jog, text="-Y", command=lambda: self._jog("-y"), **js).grid(row=2, column=1, padx=3, pady=3)

        tk.Label(jog, text="Z", bg=C['bg'], fg=C['muted'],
                 font=("Helvetica", 8, "bold")).grid(row=0, column=3, padx=(14, 0))
        tk.Button(jog, text="+Z", command=lambda: self._jog("+z"), **js).grid(row=1, column=3, padx=(14, 3), pady=3)
        tk.Button(jog, text="-Z", command=lambda: self._jog("-z"), **js).grid(row=2, column=3, padx=(14, 3), pady=3)

        sr = tk.Frame(p, bg=C['bg'])
        sr.pack(fill=tk.X, pady=(2, 4))
        tk.Label(sr, text="Step (mm):", bg=C['bg'], fg=C['muted'],
                 font=("Helvetica", 9)).pack(side=tk.LEFT)
        self.step_var = tk.DoubleVar(value=10.0)
        tk.Spinbox(sr, from_=1, to=100, increment=5, textvariable=self.step_var,
                   width=7, font=("Helvetica", 9),
                   bg=C['card'], fg=C['text'], insertbackground=C['text'],
                   buttonbackground=C['border'], relief="flat").pack(side=tk.LEFT, padx=8)

        self._btn(p, "■  Stop Jog", None, bg=C['surface'], fg=C['muted'],
                  width=28, command=self._stop_jog).pack(fill=tk.X, pady=(0, 2))

        # ── Poses ─────────────────────────────────────────────────
        self._section_label(p, "POSES")
        for label, cmd in [
            ("Save Pose",      "save_pose"),
            ("Save to File",   "save_poses_file"),
            ("Load from File", "load_poses"),
            ("Clear All",      "clear_poses"),
        ]:
            self._btn(p, label, cmd, bg=C['card'], fg=C['text'],
                      width=28).pack(fill=tk.X, pady=2)

    def _build_mid(self, p):
        # ── Status cards ──────────────────────────────────────────
        cards = tk.Frame(p, bg=C['bg'])
        cards.pack(fill=tk.X, pady=(0, 12))
        cards.columnconfigure((0, 1, 2, 3), weight=1)

        self._sys_card   = self._status_card(cards, "SYSTEM",   "Stopped", col=0)
        self._state_card = self._status_card(cards, "ROBOT",    "Idle",    col=1)
        self._obj_card   = self._status_card(cards, "OBJECT",   "—",       col=2)
        self._seq_card   = self._status_card(cards, "SEQUENCE", "—",       col=3)

        # ── Log header ────────────────────────────────────────────
        lhdr = tk.Frame(p, bg=C['bg'])
        lhdr.pack(fill=tk.X, pady=(0, 6))
        tk.Label(lhdr, text="EVENT LOG",
                 font=("Helvetica", 9, "bold"),
                 bg=C['bg'], fg=C['muted']).pack(side=tk.LEFT)
        tk.Button(lhdr, text="Clear", command=self._clear_log,
                  bg=C['card'], fg=C['muted'], font=("Helvetica", 8),
                  relief="flat", cursor="hand2", borderwidth=0,
                  activebackground=C['border'], activeforeground=C['text']
                  ).pack(side=tk.RIGHT)

        # ── Log box ───────────────────────────────────────────────
        log_wrap = tk.Frame(p, bg=C['surface'], pady=2, padx=2)
        log_wrap.pack(fill=tk.BOTH, expand=True)

        self.log_box = tk.Text(
            log_wrap,
            font=("Courier", 9),
            bg=C['surface'], fg=C['text'],
            insertbackground=C['text'],
            selectbackground=C['border'],
            relief="flat", borderwidth=0,
            state="disabled",
        )
        self.log_box.pack(fill=tk.BOTH, expand=True, padx=6, pady=4)

        self.log_box.tag_config("ts",     foreground=C['muted'])
        self.log_box.tag_config("ok",     foreground=C['green'])
        self.log_box.tag_config("warn",   foreground=C['yellow'])
        self.log_box.tag_config("err",    foreground=C['red'])
        self.log_box.tag_config("info",   foreground=C['blue'])
        self.log_box.tag_config("normal", foreground=C['text'])

    def _build_perception_panel(self, p):
        """Right panel: live feed from Rian's detection + grip pose nodes."""

        self._section_label(p, "PERCEPTION  (Rian)")

        # ── Detection count card ──────────────────────────────────
        det_frame = tk.Frame(p, bg=C['card'], padx=14, pady=10)
        det_frame.pack(fill=tk.X, pady=(0, 8))

        det_hdr = tk.Frame(det_frame, bg=C['card'])
        det_hdr.pack(fill=tk.X)
        tk.Label(det_hdr, text="DETECTIONS",
                 font=("Helvetica", 7, "bold"),
                 bg=C['card'], fg=C['muted']).pack(side=tk.LEFT)
        self._det_status_dot = tk.Label(det_hdr, text="●",
                                        font=("Helvetica", 9),
                                        bg=C['card'], fg=C['muted'])
        self._det_status_dot.pack(side=tk.RIGHT)

        self._det_count_label = tk.Label(det_frame, text="0 objects",
                                         font=("Helvetica", 12, "bold"),
                                         bg=C['card'], fg=C['muted'])
        self._det_count_label.pack(anchor="w", pady=(3, 0))

        # ── Per-detection list ────────────────────────────────────
        list_wrap = tk.Frame(p, bg=C['surface'], padx=2, pady=2)
        list_wrap.pack(fill=tk.X, pady=(0, 12))

        tk.Label(list_wrap, text="Detected objects  (/plastic_detections)",
                 font=("Helvetica", 8, "bold"),
                 bg=C['surface'], fg=C['muted']).pack(anchor="w", padx=6, pady=(4, 2))

        self._det_list_box = tk.Text(
            list_wrap,
            font=("Courier", 8),
            bg=C['surface'], fg=C['text'],
            height=6,
            relief="flat", borderwidth=0,
            state="disabled",
        )
        self._det_list_box.pack(fill=tk.X, padx=6, pady=(0, 4))
        self._det_list_box.tag_config("cls",   foreground=C['orange'])
        self._det_list_box.tag_config("pos",   foreground=C['cyan'])
        self._det_list_box.tag_config("dim",   foreground=C['purple'])
        self._det_list_box.tag_config("muted", foreground=C['muted'])

        # ── Grip pose section ─────────────────────────────────────
        self._section_label(p, "GRIP POSE  (Rian → /grip_pose)")

        grip_wrap = tk.Frame(p, bg=C['surface'], padx=2, pady=2)
        grip_wrap.pack(fill=tk.X, pady=(0, 12))

        tk.Label(grip_wrap, text="Computed grip poses",
                 font=("Helvetica", 8, "bold"),
                 bg=C['surface'], fg=C['muted']).pack(anchor="w", padx=6, pady=(4, 2))

        self._grip_list_box = tk.Text(
            grip_wrap,
            font=("Courier", 8),
            bg=C['surface'], fg=C['text'],
            height=8,
            relief="flat", borderwidth=0,
            state="disabled",
        )
        self._grip_list_box.pack(fill=tk.X, padx=6, pady=(0, 4))
        self._grip_list_box.tag_config("cls",   foreground=C['orange'])
        self._grip_list_box.tag_config("pos",   foreground=C['cyan'])
        self._grip_list_box.tag_config("grip",  foreground=C['green'])
        self._grip_list_box.tag_config("jaw",   foreground=C['yellow'])
        self._grip_list_box.tag_config("muted", foreground=C['muted'])

        # ── Bilguun's pipeline note ────────────────────────────────
        self._section_label(p, "MOTION SYSTEM  (Bilguun)")

        info = tk.Frame(p, bg=C['card'], padx=14, pady=10)
        info.pack(fill=tk.X)
        tk.Label(info, text="Active pipeline",
                 font=("Helvetica", 7, "bold"),
                 bg=C['card'], fg=C['muted']).pack(anchor="w")
        pipeline_text = (
            "/plastic_detections\n"
            "  → translator_node\n"
            "  → /perception/objects\n"
            "  → pick_n_place_node\n"
            "  ← /motion_system/command"
        )
        tk.Label(info, text=pipeline_text,
                 font=("Courier", 8),
                 bg=C['card'], fg=C['muted'],
                 justify=tk.LEFT).pack(anchor="w", pady=(4, 0))

    # ── E-Stop bar ────────────────────────────────────────────────────────────

    def _build_estop_bar(self):
        bar = tk.Frame(self.root, bg=C['surface'], height=62)
        bar.pack(fill=tk.X, side=tk.BOTTOM)
        bar.pack_propagate(False)

        tk.Button(bar, text="  ⚠  E-STOP  ",
                  command=lambda: self.send_command("estop"),
                  bg=C['red'], fg='white',
                  font=("Helvetica", 13, "bold"),
                  relief="flat", cursor="hand2",
                  activebackground='#c0392b', activeforeground='white',
                  borderwidth=0, height=2
                  ).pack(side=tk.LEFT, padx=16, pady=10)

        tk.Button(bar, text="↺  Reset",
                  command=lambda: self.send_command("reset"),
                  bg=C['yellow'], fg=C['bg'],
                  font=("Helvetica", 11, "bold"),
                  relief="flat", cursor="hand2",
                  activebackground='#b7860b', activeforeground=C['bg'],
                  borderwidth=0, width=10, height=2
                  ).pack(side=tk.LEFT, padx=4, pady=10)

        tk.Label(bar, text="Emergency stop halts all motion immediately.",
                 font=("Helvetica", 9), bg=C['surface'], fg=C['muted']
                 ).pack(side=tk.LEFT, padx=14)

    # ══════════════════════════════════════════════════════════════════════════
    # Widget helpers
    # ══════════════════════════════════════════════════════════════════════════

    def _section_label(self, parent, title):
        tk.Label(parent, text=title,
                 font=("Helvetica", 8, "bold"),
                 bg=C['bg'], fg=C['muted']).pack(anchor="w", pady=(14, 3))
        tk.Frame(parent, bg=C['border'], height=1).pack(fill=tk.X, pady=(0, 6))

    def _btn(self, parent, text, cmd, bg=None, fg=None,
             font=("Helvetica", 10, "bold"), width=None, height=1,
             command=None):
        bg = bg or C['card']
        fg = fg or C['text']
        cb = command if command else (lambda c=cmd: self.send_command(c))
        return tk.Button(parent, text=text, command=cb,
                         bg=bg, fg=fg, font=font, width=width, height=height,
                         relief="flat", cursor="hand2", borderwidth=0,
                         activebackground=C['border'], activeforeground=fg)

    def _status_card(self, parent, label, initial, col):
        frame = tk.Frame(parent, bg=C['card'], padx=14, pady=10)
        frame.grid(row=0, column=col, padx=(0, 8), sticky="ew")
        tk.Label(frame, text=label,
                 font=("Helvetica", 7, "bold"),
                 bg=C['card'], fg=C['muted']).pack(anchor="w")
        pill = tk.Label(frame, text=initial,
                        font=("Helvetica", 12, "bold"),
                        bg=C['card'], fg=C['text'])
        pill.pack(anchor="w", pady=(3, 0))
        return pill

    # ══════════════════════════════════════════════════════════════════════════
    # Commands & jogging
    # ══════════════════════════════════════════════════════════════════════════

    def send_command(self, command):
        msg = String()
        msg.data = command
        self.cmd_pub.publish(msg)
        self._log(f"→ {command}", "info")

    def _jog(self, direction):
        if self.jog_active:
            self._stop_jog()
        step = self.step_var.get() / 1000.0
        cmds = {
            "+x": f"jog_cartesian {step} 0 0 0 0 0",
            "-x": f"jog_cartesian -{step} 0 0 0 0 0",
            "+y": f"jog_cartesian 0 {step} 0 0 0 0",
            "-y": f"jog_cartesian 0 -{step} 0 0 0 0",
            "+z": f"jog_cartesian 0 0 {step} 0 0 0",
            "-z": f"jog_cartesian 0 0 -{step} 0 0 0",
        }
        if direction in cmds:
            self.jog_active = True
            self.jog_thread = threading.Thread(
                target=self._jog_worker, args=(cmds[direction],), daemon=True)
            self.jog_thread.start()

    def _stop_jog(self):
        self.jog_active = False

    def _jog_worker(self, command):
        while self.jog_active:
            msg = String()
            msg.data = command
            self.cmd_pub.publish(msg)
            time.sleep(0.1)

    # ══════════════════════════════════════════════════════════════════════════
    # ROS callbacks — existing topics
    # ══════════════════════════════════════════════════════════════════════════

    def _cb_sys_status(self, msg):
        s = msg.data
        color = C['green']  if s.lower() == "running" else \
                C['red']    if s.lower() == "failed"  else C['muted']
        self._sys_card.config(text=s, fg=color)

    def _cb_robot_state(self, msg):
        s = msg.data
        color = C['yellow'] if "pick" in s.lower() else \
                C['blue']   if "plac" in s.lower() else \
                C['green']  if "grip" in s.lower() else C['text']
        self._state_card.config(text=s, fg=color)

    def _cb_mot_status(self, msg):
        s = msg.data
        color = C['cyan']   if any(w in s.lower() for w in ("processing", "running", "started")) else \
                C['green']  if any(w in s.lower() for w in ("complete", "placed"))               else \
                C['yellow'] if "stop" in s.lower() else C['text']
        self._seq_card.config(text=s, fg=color)

        tag = "ok"   if any(w in s.lower() for w in ("complete", "placed", "home")) else \
              "warn" if "stop" in s.lower() else \
              "err"  if "fail" in s.lower() else "normal"
        self._log(s, tag)

    def _cb_cur_object(self, msg):
        s = msg.data
        color = C['orange'] if s not in ("Idle", "—") else C['muted']
        self._obj_card.config(text=s, fg=color)

    def _cb_event_log(self, msg):
        s = msg.data.lower()
        tag = "ok"   if any(w in s for w in ("picked", "placed", "complete", "saved")) else \
              "err"  if "fail" in s else \
              "warn" if "stop" in s else "normal"
        self._log(msg.data, tag)

    # ══════════════════════════════════════════════════════════════════════════
    # ROS callbacks — Rian's perception topics
    # ══════════════════════════════════════════════════════════════════════════

    def _cb_plastic_detections(self, msg):
        try:
            detections = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        self._latest_detections = detections
        n = len(detections)

        color = C['green'] if n > 0 else C['muted']
        noun = "object" if n == 1 else "objects"
        self._det_count_label.config(text=f"{n} {noun}", fg=color)
        self._det_status_dot.config(fg=C['green'] if n > 0 else C['muted'])

        self._det_list_box.config(state="normal")
        self._det_list_box.delete("1.0", tk.END)
        for det in detections:
            cls  = det.get("classification", {}).get("class", "?")
            conf = det.get("classification", {}).get("confidence", 0.0)
            pos  = det.get("pose", {}).get("position", {})
            dims = det.get("dimensions", {})
            x  = pos.get("x", 0);  y  = pos.get("y", 0);  z  = pos.get("z", 0)
            dx = dims.get("dx_mm", 0); dy = dims.get("dy_mm", 0)

            self._det_list_box.insert(tk.END, f"  {cls}", "cls")
            self._det_list_box.insert(tk.END, f"  ({conf:.2f})\n", "muted")
            self._det_list_box.insert(tk.END,
                f"    pos ({x:.0f}, {y:.0f}, {z:.0f}) mm\n", "pos")
            self._det_list_box.insert(tk.END,
                f"    dims {dx:.0f}×{dy:.0f} mm\n", "dim")
        self._det_list_box.config(state="disabled")

    def _cb_grip_pose(self, msg):
        try:
            grip_poses = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        self._latest_grip_poses = grip_poses

        self._grip_list_box.config(state="normal")
        self._grip_list_box.delete("1.0", tk.END)
        for gp in grip_poses:
            cls  = gp.get("class", "?")
            conf = gp.get("confidence", 0.0)
            gpos = gp.get("grip_position", {})
            jaw  = gp.get("jaw_opening_mm", 0.0)
            appr = gp.get("approach", "?")
            gx = gpos.get("x", 0); gy = gpos.get("y", 0); gz = gpos.get("z", 0)

            self._grip_list_box.insert(tk.END, f"  {cls}", "cls")
            self._grip_list_box.insert(tk.END, f"  ({conf:.2f})\n", "muted")
            self._grip_list_box.insert(tk.END,
                f"    grip ({gx:.0f}, {gy:.0f}, {gz:.0f}) mm\n", "pos")
            self._grip_list_box.insert(tk.END,
                f"    jaw  {jaw:.1f} mm\n", "jaw")
            self._grip_list_box.insert(tk.END,
                f"    approach: {appr}\n", "grip")
        self._grip_list_box.config(state="disabled")

    # ══════════════════════════════════════════════════════════════════════════
    # Log & lifecycle
    # ══════════════════════════════════════════════════════════════════════════

    def _log(self, text, tag="normal"):
        ts = datetime.now().strftime("%H:%M:%S")
        self.log_box.config(state="normal")
        self.log_box.insert(tk.END, f"[{ts}]  ", "ts")
        self.log_box.insert(tk.END, text + "\n", tag)
        self.log_box.see(tk.END)
        self.log_box.config(state="disabled")

    def _clear_log(self):
        self.log_box.config(state="normal")
        self.log_box.delete("1.0", tk.END)
        self.log_box.config(state="disabled")

    def _spin_ros(self):
        rclpy.spin_once(self, timeout_sec=0.01)
        self.root.after(100, self._spin_ros)

    def _on_close(self):
        self._stop_jog()
        self.destroy_node()
        rclpy.shutdown()
        self.root.destroy()


def main():
    rclpy.init()
    gui = StatusGui()
    gui.root.mainloop()


if __name__ == "__main__":
    main()
