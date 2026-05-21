#!/usr/bin/env python3

import json
import threading
import time
from datetime import datetime

import tkinter as tk
import numpy as np
from PIL import Image as PILImage, ImageTk

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image as RosImage

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

# State diagram — ordered pick-place stages
_SD_STATES = [
    ("IDLE",         "IDLE",   C['muted']),
    ("HOMING",       "HOME",   C['blue']),
    ("WAITING",      "WAIT",   C['cyan']),
    ("PREGRASP",     "PICK",   C['yellow']),
    ("GRASPING",     "GRASP",  C['orange']),
    ("RAISING",      "RAISE",  C['cyan']),
    ("TRANSPORTING", "PLACE",  C['cyan']),
    ("DONE",         "DONE",   C['green']),
]
_SD_IDX = {k: i for i, (k, _, _) in enumerate(_SD_STATES)}


class StatusGui(Node):
    def __init__(self):
        super().__init__("status_gui")

        self.jog_active = False
        self.jog_thread = None

        self._latest_detections = []
        self._sd_cur = 0
        self._sd_failed = False

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
        self.create_subscription(RosImage, "/camera/annotated",
                                 self._cb_camera_frame,        1)

        self._build()

    # ══════════════════════════════════════════════════════════════════════════
    # GUI Construction
    # ══════════════════════════════════════════════════════════════════════════

    def _build(self):
        self.root = tk.Tk()
        self.root.title("UR3e  ·  Control Panel")
        self.root.geometry("1380x920")
        self.root.resizable(True, True)
        self.root.configure(bg=C['bg'])

        self._build_header()
        self._build_body()
        self._build_estop_bar()

        self.root.bind("<F11>",  lambda e: self._toggle_fullscreen())
        self.root.bind("<Escape>", lambda e: self._exit_fullscreen())

        self.root.after(100, self._spin_ros)
        self.root.after(300, self._redraw_state_diagram)
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

        self._build_state_diagram(p)

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

        self._section_label(p, "CAMERA FEED  (/camera/annotated)")

        cam_wrap = tk.Frame(p, bg=C['card'], padx=4, pady=4)
        cam_wrap.pack(fill=tk.X, pady=(0, 8))

        self._camera_label = tk.Label(
            cam_wrap,
            text="Waiting for camera...",
            font=("Courier", 9),
            bg=C['card'], fg=C['muted'],
            width=336, height=14,  # placeholder size before first frame
        )
        self._camera_label.pack()

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
            height=4,
            relief="flat", borderwidth=0,
            state="disabled",
        )
        self._det_list_box.pack(fill=tk.X, padx=6, pady=(0, 4))
        self._det_list_box.tag_config("cls",   foreground=C['orange'])
        self._det_list_box.tag_config("pos",   foreground=C['cyan'])
        self._det_list_box.tag_config("dim",   foreground=C['purple'])
        self._det_list_box.tag_config("muted", foreground=C['muted'])

        # ── Pick Priority ─────────────────────────────────────────
        self._section_label(p, "PICK PRIORITY")

        self._target_var = tk.StringVar(value="")

        priority_types = [
            ("",            "Any  (closest first)", C['text']),
            ("pet_bottle",  "pet_bottle",           C['cyan']),
            ("hdpe_bottle", "hdpe_bottle",           C['orange']),
        ]
        self._priority_conf_labels = {}

        for value, label_text, colour in priority_types:
            row = tk.Frame(p, bg=C['bg'])
            row.pack(fill=tk.X, pady=1)

            tk.Radiobutton(
                row,
                text=label_text,
                variable=self._target_var,
                value=value,
                command=self._on_priority_change,
                bg=C['bg'], fg=colour,
                selectcolor=C['card'],
                activebackground=C['bg'], activeforeground=colour,
                font=("Helvetica", 9, "bold"),
                indicatoron=True,
            ).pack(side=tk.LEFT)

            if value:  # no confidence label for "Any"
                conf_lbl = tk.Label(
                    row, text="—",
                    font=("Helvetica", 8),
                    bg=C['bg'], fg=C['muted'],
                )
                conf_lbl.pack(side=tk.RIGHT, padx=6)
                self._priority_conf_labels[value] = conf_lbl

        # ── Bin Poses ─────────────────────────────────────────────
        self._section_label(p, "BIN POSES")

        tk.Label(p, text="Jog arm over drop bin, then click type to save.",
                 font=("Helvetica", 8), bg=C['bg'], fg=C['muted'],
                 wraplength=330, justify=tk.LEFT).pack(anchor="w", pady=(0, 6))

        bin_grid = tk.Frame(p, bg=C['bg'])
        bin_grid.pack(fill=tk.X, pady=(0, 4))
        bin_grid.columnconfigure((0, 1), weight=1)

        known_types = [
            ("pet_bottle",  C['cyan']),
            ("hdpe_bottle", C['orange']),
        ]
        for i, (type_name, colour) in enumerate(known_types):
            row, col = divmod(i, 2)
            tk.Button(
                bin_grid, text=type_name,
                command=lambda t=type_name: self._save_bin(t),
                bg=C['card'], fg=colour,
                font=("Helvetica", 9, "bold"),
                relief="flat", cursor="hand2", borderwidth=0,
                activebackground=C['border'], activeforeground=colour,
                pady=6,
            ).grid(row=row, column=col, padx=3, pady=3, sticky="ew")

        custom_row = tk.Frame(p, bg=C['bg'])
        custom_row.pack(fill=tk.X, pady=(4, 2))
        self._bin_custom_var = tk.StringVar()
        tk.Entry(
            custom_row,
            textvariable=self._bin_custom_var,
            font=("Helvetica", 9),
            bg=C['card'], fg=C['text'],
            insertbackground=C['text'],
            relief="flat",
        ).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 6), ipady=4)
        tk.Button(
            custom_row, text="Save",
            command=self._save_bin_custom,
            bg=C['yellow'], fg=C['bg'],
            font=("Helvetica", 9, "bold"),
            relief="flat", cursor="hand2", borderwidth=0,
            activebackground='#b7860b', activeforeground=C['bg'],
            padx=8, pady=4,
        ).pack(side=tk.LEFT)

        self._btn(p, "↺  Reload Bins", "LOAD_BINS",
                  bg=C['surface'], fg=C['muted'],
                  width=28).pack(fill=tk.X, pady=(6, 0))

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

    # ── State diagram ─────────────────────────────────────────────────────────

    def _build_state_diagram(self, parent):
        frame = tk.Frame(parent, bg=C['card'], padx=6, pady=8)
        frame.pack(fill=tk.X, pady=(0, 10))
        tk.Label(frame, text="PICK-PLACE CYCLE",
                 font=("Helvetica", 7, "bold"),
                 bg=C['card'], fg=C['muted']).pack(anchor='w', pady=(0, 6))
        self._sd_canvas = tk.Canvas(
            frame, bg=C['card'], height=68, highlightthickness=0)
        self._sd_canvas.pack(fill=tk.X)
        self._sd_canvas.bind("<Configure>", lambda *_: self._redraw_state_diagram())

    def _redraw_state_diagram(self):
        c = self._sd_canvas
        W = c.winfo_width()
        if W < 20:
            return
        c.delete("all")
        n = len(_SD_STATES)
        R  = 9
        cy = 26
        ly = cy + R + 5
        xs = [int(W * (i + 0.5) / n) for i in range(n)]

        # Connecting lines
        for i in range(n - 1):
            c.create_line(xs[i] + R, cy, xs[i + 1] - R, cy,
                          fill=C['muted'] if i < self._sd_cur else C['border'],
                          width=2)

        # Nodes
        for i in range(n):
            label, color = _SD_STATES[i][1], _SD_STATES[i][2]
            x = xs[i]
            is_cur  = (i == self._sd_cur)
            is_past = (i < self._sd_cur)

            if is_cur and self._sd_failed:
                fill, out, lc = C['red'],    C['red'],    C['red']
            elif is_cur:
                fill, out, lc = color,       color,       color
            elif is_past:
                fill, out, lc = C['border'], C['border'], C['muted']
            else:
                fill, out, lc = C['card'],   C['border'], C['muted']

            c.create_oval(x - R, cy - R, x + R, cy + R,
                          fill=fill, outline=out, width=2)
            fw = "bold" if is_cur else "normal"
            c.create_text(x, ly, text=label, anchor='n',
                          font=("Helvetica", 7, fw), fill=lc)

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

    def _on_priority_change(self):
        target = self._target_var.get()
        self.send_command(f"TARGET_CLASS:{target}")
        label = target if target else "any"
        self._log(f"Pick priority → {label}", "info")

    def _save_bin(self, type_name: str):
        self.send_command(f"SAVE_BIN:{type_name}")
        self._log(f"Bin saved for '{type_name}'", "ok")

    def _save_bin_custom(self):
        name = self._bin_custom_var.get().strip()
        if not name:
            self._log("Enter a type name before saving", "warn")
            return
        self._save_bin(name)
        self._bin_custom_var.set("")

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
        self._update_state_diagram(s)

    def _update_state_diagram(self, state: str):
        if state == "FAILED":
            self._sd_failed = True
        else:
            self._sd_failed = False
            if state in _SD_IDX:
                self._sd_cur = _SD_IDX[state]
        self._redraw_state_diagram()

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

        # Update confidence labels in PICK PRIORITY section
        best_conf = {}  # cls -> highest confidence seen this message
        for det in detections:
            cls  = det.get("classification", {}).get("class", "?")
            conf = det.get("classification", {}).get("confidence", 0.0)
            if cls not in best_conf or conf > best_conf[cls]:
                best_conf[cls] = conf
        for cls, lbl in self._priority_conf_labels.items():
            if cls in best_conf:
                lbl.config(text=f"{best_conf[cls]:.2f}", fg=C['green'])
            else:
                lbl.config(text="—", fg=C['muted'])

    def _cb_camera_frame(self, msg: RosImage):
        try:
            frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                msg.height, msg.width, 3)
            # BGR → RGB
            frame_rgb = frame[:, :, ::-1]
            pil_img = PILImage.fromarray(frame_rgb)
            pil_img = pil_img.resize((336, 252), PILImage.LANCZOS)
            photo = ImageTk.PhotoImage(pil_img)
            self._camera_label.config(image=photo, text="", width=336, height=252)
            self._camera_label.image = photo  # prevent GC
        except Exception as e:
            self.get_logger().warn(f'Camera frame decode error: {e}')

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

    def _toggle_fullscreen(self):
        is_full = self.root.attributes('-fullscreen')
        self.root.attributes('-fullscreen', not is_full)

    def _exit_fullscreen(self):
        self.root.attributes('-fullscreen', False)

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
