#!/usr/bin/env python3

import threading
import time
import tkinter as tk
from tkinter import messagebox, ttk

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool, Trigger

from system_interfaces.srv import (
    ControlParams,
    SetKp,
    SetOrientation,
    StrategyCommand,
    UpdateObstacle,
)


class StrategyCommandGUI(Node):
    FORMATION_TEMPLATES = {
        "Defensive": [
            (0, 2250.0, 0.0),
            (1, 1600.0, 800.0),
            (2, 1600.0, -800.0),
            (3, 1050.0, 1200.0),
            (4, 1050.0, -1200.0),
            (5, 700.0, 0.0),
        ],
        "Balanced": [
            (0, 2250.0, 0.0),
            (1, 500.0, 0.0),
            (2, 1000.0, 0.0),
            (3, 1500.0, 0.0),
            (4, 500.0, 700.0),
            (5, 500.0, -700.0),
        ],
        "Offensive": [
            (0, 2250.0, 0.0),
            (1, 300.0, 900.0),
            (2, 300.0, -900.0),
            (3, -400.0, 1200.0),
            (4, -400.0, -1200.0),
            (5, -1200.0, 0.0),
        ],
    }

    def __init__(self):
        super().__init__("strategy_command_gui")

        self.strategy_client = self.create_client(StrategyCommand, "strategy_command")
        self.pid_client = self.create_client(ControlParams, "update_pid")
        self.kp_angular_client = self.create_client(SetKp, "update_kp_angular")
        self.set_orientation_client = self.create_client(SetOrientation, "set_orientation")
        self.update_obstacles_client = self.create_client(UpdateObstacle, "update_obstacles")
        self.set_team_color_client = self.create_client(SetBool, "set_team_color")
        self.reset_commands_client = self.create_client(Trigger, "reset_commands")

        self.setup_gui()

        self.ros_thread = threading.Thread(target=self.spin_ros, daemon=True)
        self.ros_thread.start()

        self.check_service_timer = self.create_timer(1.0, self.check_service_availability)

    # ── GUI setup ──────────────────────────────────────────────────────────────

    def setup_gui(self):
        self.root = tk.Tk()
        self.root.title("Strategy Command GUI")
        self.root.geometry("900x760")

        notebook = ttk.Notebook(self.root)
        notebook.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        self._setup_strategy_tab(notebook)
        self._setup_pid_tab(notebook)

    def _setup_strategy_tab(self, notebook):
        frame = ttk.Frame(notebook, padding="10")
        notebook.add(frame, text="Strategy Commands")
        frame.columnconfigure(0, weight=1)
        frame.rowconfigure(3, weight=1)

        self._build_header(frame)
        self._build_middle(frame)
        self._build_obstacle_frame(frame)
        self._build_response_frame(frame)

    def _build_header(self, parent):
        header = ttk.Frame(parent)
        header.grid(row=0, column=0, sticky=(tk.W, tk.E), pady=(0, 8))

        self.status_label = ttk.Label(header, text="Strategy Service: Checking...", foreground="orange")
        self.status_label.grid(row=0, column=0, sticky=tk.W, padx=(0, 16))

        self.obstacles_status_label = ttk.Label(header, text="Obstacles Service: Checking...", foreground="orange")
        self.obstacles_status_label.grid(row=0, column=1, sticky=tk.W, padx=(0, 24))

        color_frame = ttk.LabelFrame(header, text="Team Color", padding="2 2")
        color_frame.grid(row=0, column=2, sticky=tk.W, padx=(0, 12))
        self.team_color_var = tk.StringVar(value="Blue")
        ttk.Radiobutton(
            color_frame, text="Yellow", variable=self.team_color_var, value="Yellow",
            command=self.on_team_color_changed,
        ).grid(row=0, column=0, padx=(2, 6))
        ttk.Radiobutton(
            color_frame, text="Blue", variable=self.team_color_var, value="Blue",
            command=self.on_team_color_changed,
        ).grid(row=0, column=1, padx=(0, 2))

        half_frame = ttk.LabelFrame(header, text="Our Half", padding="2 2")
        half_frame.grid(row=0, column=3, sticky=tk.W)
        self.our_half_var = tk.StringVar(value="Left")
        ttk.Radiobutton(half_frame, text="Left", variable=self.our_half_var, value="Left").grid(row=0, column=0, padx=(2, 6))
        ttk.Radiobutton(half_frame, text="Right", variable=self.our_half_var, value="Right").grid(row=0, column=1, padx=(0, 2))

    def _build_middle(self, parent):
        middle = ttk.Frame(parent)
        middle.grid(row=1, column=0, sticky=(tk.W, tk.E), pady=(0, 8))
        middle.columnconfigure(0, weight=1)
        middle.columnconfigure(1, weight=1)
        self._build_manual_frame(middle)
        self._build_automation_frame(middle)

    def _build_manual_frame(self, parent):
        frame = ttk.LabelFrame(parent, text="Manual Command", padding="8")
        frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S), padx=(0, 6))

        ttk.Label(frame, text="Robot ID:").grid(row=0, column=0, sticky=tk.W, pady=2)
        self.robot_id_var = tk.IntVar(value=0)
        ttk.Spinbox(frame, from_=0, to=15, width=10, textvariable=self.robot_id_var).grid(row=0, column=1, sticky=tk.W, pady=2)

        for row, (label, attr) in enumerate([
            ("Position X (mm):", "position_x_var"),
            ("Position Y (mm):", "position_y_var"),
            ("Velocity X (mm/s):", "velocity_x_var"),
            ("Velocity Y (mm/s):", "velocity_y_var"),
        ], start=1):
            ttk.Label(frame, text=label).grid(row=row, column=0, sticky=tk.W, pady=2)
            var = tk.DoubleVar(value=0.0)
            setattr(self, attr, var)
            ttk.Entry(frame, textvariable=var, width=15).grid(row=row, column=1, sticky=tk.W, pady=2)

        ttk.Separator(frame, orient=tk.HORIZONTAL).grid(row=5, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=6)

        ttk.Label(frame, text="Orientation (rad):").grid(row=6, column=0, sticky=tk.W, pady=2)
        self.orientation_var = tk.DoubleVar(value=0.0)
        ttk.Entry(frame, textvariable=self.orientation_var, width=15).grid(row=6, column=1, sticky=tk.W, pady=2)
        ttk.Scale(
            frame, from_=-3.14159, to=3.14159, orient=tk.HORIZONTAL, variable=self.orientation_var,
        ).grid(row=7, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=(2, 8))

        btn_frame = ttk.Frame(frame)
        btn_frame.grid(row=8, column=0, columnspan=2, pady=4)
        self.send_button = ttk.Button(btn_frame, text="Send Command", command=self.send_command, state="disabled")
        self.send_button.grid(row=0, column=0, padx=4)
        self.set_orientation_button = ttk.Button(btn_frame, text="Set Orientation", command=self.set_orientation, state="disabled")
        self.set_orientation_button.grid(row=0, column=1, padx=4)
        ttk.Button(btn_frame, text="Clear", command=self.clear_fields).grid(row=0, column=2, padx=4)

        preset_frame = ttk.LabelFrame(frame, text="Preset Positions", padding="6")
        preset_frame.grid(row=9, column=0, columnspan=2, pady=(8, 0), sticky=(tk.W, tk.E))
        presets = [
            ("Center", 0, 0, 0), ("Our Goal", -2250, 0, 1), ("Enemy Goal", 2250, 0, 2), ("Our Penalty", -1250, 0, 3),
            ("Top Left", -2250, 1500, 4), ("Top Right", 2250, 1500, 5), ("Bottom Left", -2250, -1500, 6), ("Bottom Right", 2250, -1500, 7),
        ]
        for label, x, y, idx in presets:
            r, c = divmod(idx, 4)
            ttk.Button(
                preset_frame, text=label, command=lambda x=x, y=y: self.set_preset(x, y, 0, 0),
            ).grid(row=r, column=c, padx=2, pady=2, sticky=tk.W + tk.E)
        for i in range(4):
            preset_frame.columnconfigure(i, weight=1)

    def _build_automation_frame(self, parent):
        frame = ttk.LabelFrame(parent, text="Automation", padding="8")
        frame.grid(row=0, column=1, sticky=(tk.W, tk.E, tk.N, tk.S), padx=(6, 0))
        frame.columnconfigure(1, weight=1)

        self.auto_strategy_running = False
        self.auto_strategy_thread = None
        self.auto_strategy_stop_event = threading.Event()

        self.auto_strategy_status_label = ttk.Label(frame, text="Auto Strategy: OFF", foreground="orange")
        self.auto_strategy_status_label.grid(row=0, column=0, sticky=tk.W, pady=2)
        self.auto_strategy_button = ttk.Button(frame, text="Activate Strategy", command=self.toggle_auto_strategy)
        self.auto_strategy_button.grid(row=0, column=1, sticky=tk.E, pady=2, padx=(8, 0))

        ttk.Separator(frame, orient=tk.HORIZONTAL).grid(row=1, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=8)

        self.campaign_running = False
        self.campaign_thread = None
        self.campaign_stop_event = threading.Event()

        ttk.Label(frame, text="Campaign Robot:").grid(row=2, column=0, sticky=tk.W, pady=2)
        self.campaign_robot_var = tk.IntVar(value=0)
        camp_ctrl = ttk.Frame(frame)
        camp_ctrl.grid(row=2, column=1, sticky=(tk.W, tk.E), pady=2)
        ttk.Spinbox(camp_ctrl, from_=0, to=15, width=6, textvariable=self.campaign_robot_var).grid(row=0, column=0, padx=(0, 6))
        self.campaign_button = ttk.Button(camp_ctrl, text="Start Campaign", command=self.toggle_campaign)
        self.campaign_button.grid(row=0, column=1)

        self.vcampaign_running = False
        self.vcampaign_thread = None
        self.vcampaign_stop_event = threading.Event()

        ttk.Label(frame, text="V-Campaign Robot:").grid(row=3, column=0, sticky=tk.W, pady=2)
        self.vcampaign_robot_var = tk.IntVar(value=0)
        vcamp_ctrl = ttk.Frame(frame)
        vcamp_ctrl.grid(row=3, column=1, sticky=(tk.W, tk.E), pady=2)
        ttk.Spinbox(vcamp_ctrl, from_=0, to=15, width=6, textvariable=self.vcampaign_robot_var).grid(row=0, column=0, padx=(0, 6))
        self.vcampaign_button = ttk.Button(vcamp_ctrl, text="Start V-Campaign", command=self.toggle_vcampaign)
        self.vcampaign_button.grid(row=0, column=1)

        ttk.Separator(frame, orient=tk.HORIZONTAL).grid(row=4, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=8)

        formation_frame = ttk.LabelFrame(frame, text="Fixed Formations", padding="6")
        formation_frame.grid(row=5, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=(0, 4))
        for col, name in enumerate(["Defensive", "Balanced", "Offensive"]):
            ttk.Button(
                formation_frame, text=name,
                command=lambda n=name: self._apply_fixed_formation(n, self._formation_for_side(n)),
            ).grid(row=0, column=col, padx=4, pady=2, sticky=tk.W + tk.E)
            formation_frame.columnconfigure(col, weight=1)

    def _build_obstacle_frame(self, parent):
        frame = ttk.LabelFrame(parent, text="Update Obstacles", padding="10")
        frame.grid(row=2, column=0, sticky=(tk.W, tk.E), pady=(0, 8))
        frame.columnconfigure(1, weight=1)

        ttk.Label(frame, text="Robot ID:").grid(row=0, column=0, sticky=tk.W, pady=2)
        self.obstacle_robot_id_var = tk.IntVar(value=0)
        ttk.Spinbox(frame, from_=0, to=15, width=6, textvariable=self.obstacle_robot_id_var).grid(row=0, column=1, sticky=tk.W, pady=2)

        zones_frame = ttk.LabelFrame(frame, text="Avoid Zones", padding="4")
        zones_frame.grid(row=1, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=(8, 4))
        self.field_border_var = tk.BooleanVar()
        self.penalty_area_var = tk.BooleanVar()
        self.center_area_var = tk.BooleanVar()
        self.ball_var = tk.BooleanVar()
        for col, (text, var) in enumerate([
            ("Field Border", self.field_border_var),
            ("Penalty Area", self.penalty_area_var),
            ("Center Area", self.center_area_var),
            ("Ball", self.ball_var),
        ]):
            ttk.Checkbutton(zones_frame, text=text, variable=var).grid(row=0, column=col, padx=8, pady=2, sticky=tk.W)

        ttk.Label(frame, text="Enemy robots:").grid(row=2, column=0, sticky=tk.W, pady=(8, 2))
        enemy_btn_frame = ttk.Frame(frame)
        enemy_btn_frame.grid(row=2, column=1, columnspan=2, sticky=tk.W, pady=(8, 2))
        self.enemy_toggle_vars = []
        for i in range(6):
            var = tk.BooleanVar()
            self.enemy_toggle_vars.append(var)
            ttk.Checkbutton(enemy_btn_frame, text=str(i), variable=var, style="Toolbutton").grid(row=0, column=i, padx=2)

        ttk.Label(frame, text="Ally robots:").grid(row=3, column=0, sticky=tk.W, pady=2)
        ally_btn_frame = ttk.Frame(frame)
        ally_btn_frame.grid(row=3, column=1, columnspan=2, sticky=tk.W, pady=2)
        self.ally_toggle_vars = []
        for i in range(6):
            var = tk.BooleanVar()
            self.ally_toggle_vars.append(var)
            ttk.Checkbutton(ally_btn_frame, text=str(i), variable=var, style="Toolbutton").grid(row=0, column=i, padx=2)

        self.update_obstacles_button = ttk.Button(
            frame, text="Update Obstacles", command=self.update_obstacles, state="disabled"
        )
        self.update_obstacles_button.grid(row=4, column=0, columnspan=3, pady=(10, 4))

    def _build_response_frame(self, parent):
        frame = ttk.LabelFrame(parent, text="Last Response", padding="5")
        frame.grid(row=3, column=0, sticky=(tk.W, tk.E, tk.N, tk.S), pady=(0, 4))
        frame.columnconfigure(0, weight=1)
        frame.rowconfigure(0, weight=1)

        self.response_text = tk.Text(frame, height=6, width=70)
        self.response_text.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        scrollbar = ttk.Scrollbar(frame, orient=tk.VERTICAL, command=self.response_text.yview)
        scrollbar.grid(row=0, column=1, sticky=(tk.N, tk.S))
        self.response_text.configure(yscrollcommand=scrollbar.set)
        ttk.Button(frame, text="Clear", command=lambda: self.response_text.delete("1.0", tk.END)).grid(
            row=1, column=0, columnspan=2, sticky=tk.E, pady=(2, 0)
        )

    def _setup_pid_tab(self, notebook):
        frame = ttk.Frame(notebook, padding="10")
        notebook.add(frame, text="PID Tuning")
        frame.columnconfigure(2, weight=1)
        frame.rowconfigure(10, weight=1)

        self.pid_status_label = ttk.Label(frame, text="PID Service Status: Checking...", foreground="orange")
        self.pid_status_label.grid(row=0, column=0, columnspan=3, pady=(0, 10))

        ttk.Label(frame, text="Robot ID:").grid(row=1, column=0, sticky=tk.W, pady=2)
        self.pid_robot_id_var = tk.IntVar(value=0)
        ttk.Spinbox(frame, from_=0, to=15, width=10, textvariable=self.pid_robot_id_var).grid(row=1, column=1, sticky=tk.W, pady=2)

        for row, (label, attr, default) in enumerate([
            ("Kp (Proportional):", "kp_var", 3.0),
            ("Ki (Integral):", "ki_var", 0.2),
            ("Kd (Derivative):", "kd_var", 1.0),
        ], start=2):
            ttk.Label(frame, text=label).grid(row=row, column=0, sticky=tk.W, pady=2)
            var = tk.DoubleVar(value=default)
            setattr(self, attr, var)
            ttk.Entry(frame, textvariable=var, width=15).grid(row=row, column=1, sticky=tk.W, pady=2)

        ttk.Separator(frame, orient=tk.HORIZONTAL).grid(row=5, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=10)

        ttk.Label(frame, text="Kp Angular (Orientação):").grid(row=6, column=0, sticky=tk.W, pady=2)
        self.kp_angular_var = tk.DoubleVar(value=2.5)
        ttk.Entry(frame, textvariable=self.kp_angular_var, width=15).grid(row=6, column=1, sticky=tk.W, pady=2)
        ttk.Scale(
            frame, from_=0, to=10.0, orient=tk.HORIZONTAL, variable=self.kp_angular_var,
        ).grid(row=7, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=(2, 10))

        pid_btn_frame = ttk.Frame(frame)
        pid_btn_frame.grid(row=8, column=0, columnspan=3, pady=10)
        self.update_pid_button = ttk.Button(pid_btn_frame, text="Update PID", command=self.update_pid, state="disabled")
        self.update_pid_button.grid(row=0, column=0, padx=5)
        ttk.Button(pid_btn_frame, text="Reset to Defaults", command=self.reset_pid_defaults).grid(row=0, column=1, padx=5)
        self.update_kp_angular_button = ttk.Button(pid_btn_frame, text="Update Kp Angular", command=self.update_kp_angular, state="disabled")
        self.update_kp_angular_button.grid(row=0, column=2, padx=5)

        pid_preset_frame = ttk.LabelFrame(frame, text="PID Presets", padding="10")
        pid_preset_frame.grid(row=9, column=0, columnspan=3, pady=10, sticky=(tk.W, tk.E))
        presets = [
            ("Conservative", 1.5, 0.1, 0.5), ("Default", 3.0, 0.2, 1.0), ("Aggressive", 5.0, 0.5, 1.5),
            ("High Precision", 4.0, 0.3, 2.0), ("Fast Response", 6.0, 0.1, 0.8), ("Smooth", 2.0, 0.4, 1.2),
        ]
        for idx, (label, kp, ki, kd) in enumerate(presets):
            r, c = divmod(idx, 3)
            ttk.Button(
                pid_preset_frame, text=label, command=lambda kp=kp, ki=ki, kd=kd: self.set_pid_preset(kp, ki, kd),
            ).grid(row=r, column=c, padx=2, pady=2, sticky=tk.W + tk.E)
        for i in range(3):
            pid_preset_frame.columnconfigure(i, weight=1)

        pid_response_frame = ttk.LabelFrame(frame, text="PID Update Response", padding="5")
        pid_response_frame.grid(row=10, column=0, columnspan=3, pady=10, sticky=(tk.W, tk.E, tk.N, tk.S))
        pid_response_frame.columnconfigure(0, weight=1)
        pid_response_frame.rowconfigure(0, weight=1)

        self.pid_response_text = tk.Text(pid_response_frame, height=8, width=70)
        self.pid_response_text.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        pid_scrollbar = ttk.Scrollbar(pid_response_frame, orient=tk.VERTICAL, command=self.pid_response_text.yview)
        pid_scrollbar.grid(row=0, column=1, sticky=(tk.N, tk.S))
        self.pid_response_text.configure(yscrollcommand=pid_scrollbar.set)
        ttk.Button(pid_response_frame, text="Clear", command=lambda: self.pid_response_text.delete("1.0", tk.END)).grid(
            row=1, column=0, columnspan=2, sticky=tk.E, pady=(2, 0)
        )

    # ── Service availability ───────────────────────────────────────────────────

    def check_service_availability(self):
        checks = [
            (self.strategy_client, self.status_label, "Strategy Service", self.send_button),
            (self.pid_client, self.pid_status_label, "PID Service", self.update_pid_button),
            (self.update_obstacles_client, self.obstacles_status_label, "Obstacles Service", self.update_obstacles_button),
        ]
        results = [(label, text, client.wait_for_service(timeout_sec=0.1), btn) for client, label, text, btn in checks]
        kp_ok = self.kp_angular_client.wait_for_service(timeout_sec=0.1)
        ori_ok = self.set_orientation_client.wait_for_service(timeout_sec=0.1)

        def _apply():
            for label, text, available, btn in results:
                label.configure(
                    text=f"{text}: {'Available' if available else 'Unavailable'}",
                    foreground="green" if available else "red",
                )
                if btn is not None:
                    btn.configure(state="normal" if available else "disabled")
            if hasattr(self, "update_kp_angular_button"):
                self.update_kp_angular_button.configure(state="normal" if kp_ok else "disabled")
            if hasattr(self, "set_orientation_button"):
                self.set_orientation_button.configure(state="normal" if ori_ok else "disabled")

        self.root.after(0, _apply)

    # ── Field helpers ──────────────────────────────────────────────────────────

    def set_preset(self, x, y, vx, vy):
        self.position_x_var.set(x)
        self.position_y_var.set(y)
        self.velocity_x_var.set(vx)
        self.velocity_y_var.set(vy)

    def set_pid_preset(self, kp, ki, kd):
        self.kp_var.set(kp)
        self.ki_var.set(ki)
        self.kd_var.set(kd)

    def reset_pid_defaults(self):
        self.pid_robot_id_var.set(0)
        self.kp_var.set(3.0)
        self.ki_var.set(0.2)
        self.kd_var.set(1.0)

    def clear_fields(self):
        self.robot_id_var.set(0)
        self.position_x_var.set(0.0)
        self.position_y_var.set(0.0)
        self.velocity_x_var.set(0.0)
        self.velocity_y_var.set(0.0)
        self.orientation_var.set(0.0)

    # ── Service calls ──────────────────────────────────────────────────────────

    def send_command(self):
        if not self.strategy_client.wait_for_service(timeout_sec=1.0):
            messagebox.showerror("Error", "Strategy command service is not available")
            return
        if self.auto_strategy_running or self.campaign_running or self.vcampaign_running:
            active = [
                n for n, flag in [
                    ("Auto Strategy", self.auto_strategy_running),
                    ("Campaign", self.campaign_running),
                    ("V-Campaign", self.vcampaign_running),
                ] if flag
            ]
            if not messagebox.askyesno(
                "Automation active",
                f"{', '.join(active)} is running and will override this command.\n\nStop automation and send?",
            ):
                return
            self.auto_strategy_stop_event.set()
            self.auto_strategy_running = False
            self.auto_strategy_button.configure(text="Activate Strategy")
            self.auto_strategy_status_label.configure(text="Auto Strategy: OFF", foreground="orange")
            self.campaign_stop_event.set()
            self.campaign_running = False
            self.campaign_button.configure(text="Start Campaign")
            self.vcampaign_stop_event.set()
            self.vcampaign_running = False
            self.vcampaign_button.configure(text="Start V-Campaign")
        try:
            request = StrategyCommand.Request()
            request.id = self.robot_id_var.get()
            request.position_x = float(self.position_x_var.get())
            request.position_y = float(self.position_y_var.get())
            request.velocity_x = float(self.velocity_x_var.get())
            request.velocity_y = float(self.velocity_y_var.get())
            self.get_logger().info(
                f"Sending strategy command: ID={request.id}, "
                f"Pos=({request.position_x}, {request.position_y}), "
                f"Vel=({request.velocity_x}, {request.velocity_y})"
            )
            future = self.strategy_client.call_async(request)
            future.add_done_callback(self.handle_response)
            self._append_response(
                f"Request sent at {time.strftime('%H:%M:%S')}:\n"
                f"Robot ID: {request.id}\n"
                f"Position: ({request.position_x}, {request.position_y})\n"
                f"Velocity: ({request.velocity_x}, {request.velocity_y})\n"
                f"Waiting for response...\n"
            )
        except ValueError as e:
            messagebox.showerror("Input Error", f"Invalid input values: {e}")
        except Exception as e:
            messagebox.showerror("Error", f"Failed to send command: {e}")
            self.get_logger().error(f"Failed to send strategy command: {e}")

    def update_obstacles(self):
        if not self.update_obstacles_client.wait_for_service(timeout_sec=1.0):
            messagebox.showerror("Error", "Update obstacles service is not available")
            return
        try:
            request = UpdateObstacle.Request()
            request.id = self.obstacle_robot_id_var.get()
            request.field_border = self.field_border_var.get()
            request.penalty_area = self.penalty_area_var.get()
            request.center_area = self.center_area_var.get()
            request.ball = self.ball_var.get()
            request.enemy_ids = [i for i, v in enumerate(self.enemy_toggle_vars) if v.get()]
            request.ally_ids = [i for i, v in enumerate(self.ally_toggle_vars) if v.get()]
            self.get_logger().info(
                f"Sending update obstacles: ID={request.id}, "
                f"Field Border={request.field_border}, Penalty={request.penalty_area}, "
                f"Center={request.center_area}, Ball={request.ball}, "
                f"Enemy={request.enemy_ids}, Ally={request.ally_ids}"
            )
            future = self.update_obstacles_client.call_async(request)
            future.add_done_callback(self.handle_obstacles_response)
            self._append_response(
                f"Obstacles Update sent at {time.strftime('%H:%M:%S')}:\n"
                f"Robot ID: {request.id} | "
                f"Border={request.field_border} Penalty={request.penalty_area} "
                f"Center={request.center_area} Ball={request.ball}\n"
                f"Enemy: {request.enemy_ids} | Ally: {request.ally_ids}\n"
                f"Waiting for response...\n"
            )
        except ValueError as e:
            messagebox.showerror("Input Error", f"Invalid input values: {e}")
        except Exception as e:
            messagebox.showerror("Error", f"Failed to send obstacles update: {e}")
            self.get_logger().error(f"Failed to send obstacles update: {e}")

    def update_pid(self):
        if not self.pid_client.wait_for_service(timeout_sec=1.0):
            messagebox.showerror("Error", "PID service is not available")
            return
        try:
            request = ControlParams.Request()
            request.id = self.pid_robot_id_var.get()
            request.kp = float(self.kp_var.get())
            request.ki = float(self.ki_var.get())
            request.kd = float(self.kd_var.get())
            self.get_logger().info(f"Sending PID update: ID={request.id}, Kp={request.kp}, Ki={request.ki}, Kd={request.kd}")
            future = self.pid_client.call_async(request)
            future.add_done_callback(self.handle_pid_response)
            self._append_pid_response(
                f"PID Update sent at {time.strftime('%H:%M:%S')}:\n"
                f"Robot ID: {request.id}\n"
                f"Kp: {request.kp}, Ki: {request.ki}, Kd: {request.kd}\n"
                f"Waiting for response...\n"
            )
        except ValueError as e:
            messagebox.showerror("Input Error", f"Invalid PID values: {e}")
        except Exception as e:
            messagebox.showerror("Error", f"Failed to update PID: {e}")
            self.get_logger().error(f"Failed to update PID: {e}")

    def update_kp_angular(self):
        if not self.kp_angular_client.wait_for_service(timeout_sec=1.0):
            messagebox.showerror("Error", "Angular Kp service 'update_kp_angular' is not available")
            return
        try:
            request = SetKp.Request()
            request.kp = float(self.kp_angular_var.get())
            self.get_logger().info(f"Sending Angular Kp update: Kp={request.kp}")
            future = self.kp_angular_client.call_async(request)
            future.add_done_callback(self.handle_pid_response)
            self._append_pid_response(
                f"Angular Kp Update sent at {time.strftime('%H:%M:%S')}:\n"
                f"Kp: {request.kp}\n"
                f"Waiting for response...\n"
            )
        except ValueError as e:
            messagebox.showerror("Input Error", f"Invalid Angular Kp value: {e}")
        except Exception as e:
            messagebox.showerror("Error", f"Failed to update Angular Kp: {e}")
            self.get_logger().error(f"Failed to update Angular Kp: {e}")

    def set_orientation(self):
        if not self.set_orientation_client.wait_for_service(timeout_sec=1.0):
            messagebox.showerror("Error", "Set orientation service is not available")
            return
        try:
            request = SetOrientation.Request()
            request.robot_id = self.robot_id_var.get()
            request.orientation = float(self.orientation_var.get())
            self.get_logger().info(f"Sending set orientation: ID={request.robot_id}, Orientation={request.orientation}")
            future = self.set_orientation_client.call_async(request)
            future.add_done_callback(self.handle_response)
            self._append_response(
                f"Set Orientation sent at {time.strftime('%H:%M:%S')}:\n"
                f"Robot ID: {request.robot_id}\n"
                f"Orientation: {request.orientation}\n"
                f"Waiting for response...\n"
            )
        except ValueError as e:
            messagebox.showerror("Input Error", f"Invalid orientation value: {e}")
        except Exception as e:
            messagebox.showerror("Error", f"Failed to set orientation: {e}")
            self.get_logger().error(f"Failed to set orientation: {e}")

    # ── Response handlers ──────────────────────────────────────────────────────

    def handle_response(self, future):
        try:
            response = future.result()
            success = "SUCCESS" if response.success else "FAILED"
            self.get_logger().info(f"Strategy command response: {success}")
            self._append_response(f"Response at {time.strftime('%H:%M:%S')}: {success}\n{'=' * 40}\n")
            if not response.success:
                self.root.after(0, lambda: messagebox.showwarning("Command Failed", "The strategy command was not successful"))
        except Exception as e:
            self.get_logger().error(str(e))
            self._append_response(f"ERROR at {time.strftime('%H:%M:%S')}: {e}\n")
            self.root.after(0, lambda e=e: messagebox.showerror("Service Error", str(e)))

    def handle_obstacles_response(self, future):
        try:
            response = future.result()
            success = "SUCCESS" if response.success else "FAILED"
            self._append_response(f"Obstacles Response at {time.strftime('%H:%M:%S')}: {success}\n{'=' * 40}\n")
            if not response.success:
                self.root.after(0, lambda: messagebox.showwarning("Update Failed", "The obstacles update was not successful"))
        except Exception as e:
            self.get_logger().error(str(e))
            self._append_response(f"ERROR at {time.strftime('%H:%M:%S')}: {e}\n")
            self.root.after(0, lambda e=e: messagebox.showerror("Service Error", str(e)))

    def handle_pid_response(self, future):
        try:
            response = future.result()
            success = "SUCCESS" if response.success else "FAILED"
            self.get_logger().info(f"PID update response: {success}")
            self._append_pid_response(f"PID Response at {time.strftime('%H:%M:%S')}: {success}\n{'=' * 40}\n")
            if not response.success:
                self.root.after(0, lambda: messagebox.showwarning("PID Update Failed", "The PID update was not successful"))
        except Exception as e:
            self.get_logger().error(str(e))
            self._append_pid_response(f"ERROR at {time.strftime('%H:%M:%S')}: {e}\n")
            self.root.after(0, lambda e=e: messagebox.showerror("PID Service Error", str(e)))

    # ── Display helpers ────────────────────────────────────────────────────────

    def _append_response(self, text):
        self.root.after(0, lambda: (self.response_text.insert(tk.END, text), self.response_text.see(tk.END)))

    def _append_pid_response(self, text):
        self.root.after(0, lambda: (self.pid_response_text.insert(tk.END, text), self.pid_response_text.see(tk.END)))

    # ── ROS spin / lifecycle ───────────────────────────────────────────────────

    def spin_ros(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

    def run(self):
        try:
            self.root.mainloop()
        finally:
            self.auto_strategy_stop_event.set()
            self.campaign_stop_event.set()
            self.vcampaign_stop_event.set()
            self.get_logger().info("Shutting down Strategy Command GUI")

    # ── Automation ─────────────────────────────────────────────────────────────

    def toggle_campaign(self):
        if not self.campaign_running:
            self.campaign_stop_event.clear()
            self.campaign_thread = threading.Thread(target=self._campaign_runner, daemon=True)
            self.campaign_thread.start()
            self.campaign_running = True
            self.campaign_button.configure(text="Stop Campaign")
            self.get_logger().info("Campaign started")
        else:
            self.campaign_stop_event.set()
            self.campaign_running = False
            self.campaign_button.configure(text="Start Campaign")
            self.get_logger().info("Campaign stopped")

    def toggle_vcampaign(self):
        if not self.vcampaign_running:
            self.vcampaign_stop_event.clear()
            self.vcampaign_thread = threading.Thread(target=self._vcampaign_runner, daemon=True)
            self.vcampaign_thread.start()
            self.vcampaign_running = True
            self.vcampaign_button.configure(text="Stop Vertical Campaign")
            self.get_logger().info("Vertical campaign started")
        else:
            self.vcampaign_stop_event.set()
            self.vcampaign_running = False
            self.vcampaign_button.configure(text="Start Vertical Campaign")
            self.get_logger().info("Vertical campaign stopped")

    def toggle_auto_strategy(self):
        if not self.auto_strategy_running:
            if not self.strategy_client.wait_for_service(timeout_sec=1.0):
                messagebox.showerror("Error", "Strategy service is unavailable. Cannot activate strategy.")
                return
            self.auto_strategy_stop_event.clear()
            self.auto_strategy_thread = threading.Thread(target=self._auto_strategy_runner, daemon=True)
            self.auto_strategy_thread.start()
            self.auto_strategy_running = True
            self.auto_strategy_button.configure(text="Deactivate Strategy")
            self.auto_strategy_status_label.configure(text="Auto Strategy: ON", foreground="green")
            self.get_logger().info("Auto strategy activated")
        else:
            self.auto_strategy_stop_event.set()
            self.auto_strategy_running = False
            self.auto_strategy_button.configure(text="Activate Strategy")
            self.auto_strategy_status_label.configure(text="Auto Strategy: OFF", foreground="orange")
            self.get_logger().info("Auto strategy deactivated")

    def _auto_strategy_runner(self):
        while not self.auto_strategy_stop_event.is_set():
            for robot_id, x, y in self._formation_for_side("Balanced"):
                self._send_strategy(robot_id, x, y)
            self.auto_strategy_stop_event.wait(2.0)

    def _campaign_runner(self):
        robot_id = int(self.campaign_robot_var.get())
        targets = [(-2250, 0), (2250, 0)]
        toggle = False
        while not self.campaign_stop_event.is_set():
            x, y = targets[toggle]
            self._send_strategy(robot_id, x, y)
            toggle = not toggle
            self.campaign_stop_event.wait(5.0)

    def _vcampaign_runner(self):
        robot_id = int(self.vcampaign_robot_var.get())
        toggle = False
        while not self.vcampaign_stop_event.is_set():
            y = -1500 if toggle else 1500
            self._send_strategy(robot_id, 0, y)
            toggle = not toggle
            self.vcampaign_stop_event.wait(5.0)

    def _field_side_sign(self) -> float:
        return -1.0 if self.our_half_var.get() == "Left" else 1.0

    def _formation_for_side(self, formation_name: str):
        side = self._field_side_sign()
        return [(robot_id, x * side, y) for robot_id, x, y in self.FORMATION_TEMPLATES[formation_name]]

    def _apply_fixed_formation(self, formation_name: str, formation):
        if not self.strategy_client.wait_for_service(timeout_sec=1.0):
            messagebox.showerror("Error", "Strategy service is unavailable")
            return
        success_count = sum(1 for robot_id, x, y in formation if self._send_strategy(robot_id, x, y))
        self._append_response(
            f"{formation_name} formation sent at {time.strftime('%H:%M:%S')}: "
            f"{success_count}/{len(formation)} robots updated\n"
        )

    def _send_strategy(self, robot_id: int, x: float, y: float, vx: float = 0.0, vy: float = 0.0) -> bool:
        if not self.strategy_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning("Strategy service unavailable when sending campaign command")
            return False
        try:
            request = StrategyCommand.Request()
            request.id = int(robot_id)
            request.position_x = float(x)
            request.position_y = float(y)
            request.velocity_x = float(vx)
            request.velocity_y = float(vy)
            self.strategy_client.call_async(request)
            self.get_logger().info(f"Campaign sent to robot {robot_id}: ({x},{y})")
            return True
        except Exception as e:
            self.get_logger().error(f"Failed to send campaign command: {e}")
            return False

    # ── Team color ─────────────────────────────────────────────────────────────

    def on_team_color_changed(self):
        selection = self.team_color_var.get()
        if not self.set_team_color_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning("set_team_color service unavailable")
            return
        try:
            req = SetBool.Request()
            req.data = selection == "Yellow"
            future = self.set_team_color_client.call_async(req)

            def _cb(fut):
                try:
                    resp = fut.result()
                    if resp.success:
                        self.get_logger().info(f"Team color set to {selection}")
                        self._reset_path_driver_commands()
                    else:
                        self.get_logger().warning(f"Failed to set team color: {resp.message}")
                except Exception as e:
                    self.get_logger().error(f"Error calling set_team_color: {e}")

            future.add_done_callback(_cb)
        except Exception as e:
            self.get_logger().error(f"Exception while calling set_team_color: {e}")

    def _reset_path_driver_commands(self):
        if not self.reset_commands_client.service_is_ready():
            return
        future = self.reset_commands_client.call_async(Trigger.Request())

        def _cb(fut):
            try:
                fut.result()
            except Exception as e:
                self.get_logger().error(f"Error resetting commands: {e}")

        future.add_done_callback(_cb)


def main(args=None):
    rclpy.init(args=args)
    try:
        gui_node = StrategyCommandGUI()
        gui_node.run()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
