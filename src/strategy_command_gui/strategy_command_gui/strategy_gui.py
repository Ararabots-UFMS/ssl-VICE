#!/usr/bin/env python3

import threading
import time
import tkinter as tk
from tkinter import messagebox, ttk

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool

from system_interfaces.srv import ControlParams, SetKp, SetOrientation, StrategyCommand, UpdateObstacle


class StrategyCommandGUI(Node):
    def __init__(self):
        super().__init__("strategy_command_gui")

        # Create service clients
        self.strategy_client = self.create_client(StrategyCommand, "strategy_command")
        self.pid_client = self.create_client(ControlParams, "update_pid")
        self.kp_angular_client = self.create_client(SetKp, "update_kp_angular")
        self.set_orientation_client = self.create_client(
            SetOrientation, "set_orientation"
        )
        self.update_obstacles_client = self.create_client(
            UpdateObstacle, "update_obstacles"
        )
        # Client to request team color change on the game watcher
        self.set_team_color_client = self.create_client(SetBool, "set_team_color")

        # Wait for services to become available
        self.get_logger().info("Waiting for services...")

        # Initialize GUI
        self.setup_gui()

        # Start ROS2 spinning in a separate thread
        self.ros_thread = threading.Thread(target=self.spin_ros, daemon=True)
        self.ros_thread.start()

        # Check service availability periodically
        self.check_service_timer = self.create_timer(
            1.0, self.check_service_availability
        )

    def setup_gui(self):
        """Setup the tkinter GUI"""
        self.root = tk.Tk()
        self.root.title("Strategy Command GUI")
        self.root.geometry("800x700")

        # Create notebook for tabs
        notebook = ttk.Notebook(self.root)
        notebook.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        # Strategy Commands Tab
        self.setup_strategy_tab(notebook)

        # PID Tuning Tab
        self.setup_pid_tab(notebook)

    def setup_strategy_tab(self, notebook):
        """Setup the strategy commands tab"""
        # Strategy tab frame
        strategy_frame = ttk.Frame(notebook, padding="10")
        notebook.add(strategy_frame, text="Strategy Commands")

        # Service status
        self.status_label = ttk.Label(
            strategy_frame, text="Service Status: Checking...", foreground="orange"
        )
        self.status_label.grid(row=0, column=0, columnspan=3, pady=(0, 10))

        # Update Obstacles Service status
        self.obstacles_status_label = ttk.Label(
            strategy_frame,
            text="Obstacles Service Status: Checking...",
            foreground="orange",
        )
        self.obstacles_status_label.grid(row=1, column=0, columnspan=3, pady=(0, 10))

        # Team color selector (Yellow / Blue)
        ttk.Label(strategy_frame, text="Team Color:").grid(
            row=0, column=3, sticky=tk.W, pady=2
        )
        self.team_color_var = tk.StringVar(value="Blue")
        team_color_combo = ttk.Combobox(
            strategy_frame,
            values=["Yellow", "Blue"],
            width=8,
            textvariable=self.team_color_var,
        )
        team_color_combo.current(0)
        team_color_combo.grid(row=0, column=4, sticky=tk.W, pady=2)
        team_color_combo.bind("<<ComboboxSelected>>", self.on_team_color_changed)

        # Robot ID
        ttk.Label(strategy_frame, text="Robot ID:").grid(
            row=2, column=0, sticky=tk.W, pady=2
        )
        self.robot_id_var = tk.IntVar(value=0)
        robot_id_spinbox = ttk.Spinbox(
            strategy_frame, from_=0, to=15, width=10, textvariable=self.robot_id_var
        )
        robot_id_spinbox.grid(row=2, column=1, sticky=tk.W, pady=2)

        # Campaign controls: select robot and start/stop
        ttk.Label(strategy_frame, text="Campaign Robot ID:").grid(
            row=2, column=2, sticky=tk.W, pady=2
        )
        self.campaign_robot_var = tk.IntVar(value=0)
        campaign_robot_spinbox = ttk.Spinbox(
            strategy_frame,
            from_=0,
            to=15,
            width=8,
            textvariable=self.campaign_robot_var,
        )
        campaign_robot_spinbox.grid(row=2, column=3, sticky=tk.W, pady=2)

        self.campaign_running = False
        self.campaign_thread = None
        self.campaign_stop_event = threading.Event()

        self.campaign_button = ttk.Button(
            strategy_frame, text="Start Campaign", command=self.toggle_campaign
        )
        self.campaign_button.grid(row=2, column=4, padx=8)

        # Position X
        ttk.Label(strategy_frame, text="Position X (mm):").grid(
            row=3, column=0, sticky=tk.W, pady=2
        )
        self.position_x_var = tk.DoubleVar(value=0.0)
        position_x_entry = ttk.Entry(
            strategy_frame, textvariable=self.position_x_var, width=15
        )
        position_x_entry.grid(row=3, column=1, sticky=tk.W, pady=2)

        # Position Y
        ttk.Label(strategy_frame, text="Position Y (mm):").grid(
            row=4, column=0, sticky=tk.W, pady=2
        )
        self.position_y_var = tk.DoubleVar(value=0.0)
        position_y_entry = ttk.Entry(
            strategy_frame, textvariable=self.position_y_var, width=15
        )
        position_y_entry.grid(row=4, column=1, sticky=tk.W, pady=2)

        # Velocity X
        ttk.Label(strategy_frame, text="Velocity X (mm/s):").grid(
            row=5, column=0, sticky=tk.W, pady=2
        )
        self.velocity_x_var = tk.DoubleVar(value=0.0)
        velocity_x_entry = ttk.Entry(
            strategy_frame, textvariable=self.velocity_x_var, width=15
        )
        velocity_x_entry.grid(row=5, column=1, sticky=tk.W, pady=2)

        # Velocity Y
        ttk.Label(strategy_frame, text="Velocity Y (mm/s):").grid(
            row=6, column=0, sticky=tk.W, pady=2
        )
        self.velocity_y_var = tk.DoubleVar(value=0.0)
        velocity_y_entry = ttk.Entry(
            strategy_frame, textvariable=self.velocity_y_var, width=15
        )
        velocity_y_entry.grid(row=6, column=1, sticky=tk.W, pady=2)

        # Separador visual
        ttk.Label(strategy_frame, text="-" * 50).grid(
            row=7, column=0, columnspan=3, pady=10
        )

        # Orientação
        ttk.Label(strategy_frame, text="Orientação (radianos):").grid(
            row=8, column=0, sticky=tk.W, pady=2
        )
        self.orientation_var = tk.DoubleVar(value=0.0)
        orientation_entry = ttk.Entry(
            strategy_frame, textvariable=self.orientation_var, width=15
        )
        orientation_entry.grid(row=8, column=1, sticky=tk.W, pady=2)

        # Slider para orientação (-π a π)
        orientation_slider = ttk.Scale(
            strategy_frame,
            from_=-3.14159,
            to=3.14159,
            orient=tk.HORIZONTAL,
            variable=self.orientation_var,
        )
        orientation_slider.grid(
            row=9, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=(2, 10)
        )

        # Buttons frame
        button_frame = ttk.Frame(strategy_frame)
        button_frame.grid(row=10, column=0, columnspan=3, pady=20)

        # Send Command Button
        self.send_button = ttk.Button(
            button_frame, text="Send Command", command=self.send_command
        )
        self.send_button.grid(row=0, column=0, padx=5)

        # Set Orientation Button
        self.set_orientation_button = ttk.Button(
            button_frame, text="Set Orientation", command=self.set_orientation
        )
        self.set_orientation_button.grid(row=0, column=1, padx=5)
        self.set_orientation_button.configure(state="disabled")

        # Clear Button
        clear_button = ttk.Button(button_frame, text="Clear", command=self.clear_fields)
        clear_button.grid(row=0, column=2, padx=5)

        # Preset Commands Frame
        preset_frame = ttk.LabelFrame(
            strategy_frame, text="Preset Positions", padding="10"
        )
        preset_frame.grid(row=11, column=0, columnspan=3, pady=10, sticky=(tk.W, tk.E))

        ttk.Button(
            preset_frame, text="Center", command=lambda: self.set_preset(0, 0, 0, 0)
        ).grid(row=0, column=0, padx=2, pady=2, sticky=tk.W + tk.E)
        ttk.Button(
            preset_frame,
            text="Our Goal",
            command=lambda: self.set_preset(-2250, 0, 0, 0),
        ).grid(row=0, column=1, padx=2, pady=2, sticky=tk.W + tk.E)
        ttk.Button(
            preset_frame,
            text="Enemy Goal",
            command=lambda: self.set_preset(2250, 0, 0, 0),
        ).grid(row=0, column=2, padx=2, pady=2, sticky=tk.W + tk.E)
        ttk.Button(
            preset_frame,
            text="Our Penalty",
            command=lambda: self.set_preset(-1250, 0, 0, 0),
        ).grid(row=0, column=3, padx=2, pady=2, sticky=tk.W + tk.E)

        ttk.Button(
            preset_frame,
            text="Top Left",
            command=lambda: self.set_preset(-2250, 1500, 0, 0),
        ).grid(row=1, column=0, padx=2, pady=2, sticky=tk.W + tk.E)
        ttk.Button(
            preset_frame,
            text="Top Right",
            command=lambda: self.set_preset(2250, 1500, 0, 0),
        ).grid(row=1, column=1, padx=2, pady=2, sticky=tk.W + tk.E)
        ttk.Button(
            preset_frame,
            text="Bottom Left",
            command=lambda: self.set_preset(-2250, -1500, 0, 0),
        ).grid(row=1, column=2, padx=2, pady=2, sticky=tk.W + tk.E)
        ttk.Button(
            preset_frame,
            text="Bottom Right",
            command=lambda: self.set_preset(2250, -1500, 0, 0),
        ).grid(row=1, column=3, padx=2, pady=2, sticky=tk.W + tk.E)

        # Configure grid weights for preset buttons
        for i in range(4):
            preset_frame.columnconfigure(i, weight=1)

        # Obstacle Update Frame
        obstacle_frame = ttk.LabelFrame(
            strategy_frame, text="Update Obstacles", padding="10"
        )
        obstacle_frame.grid(row=12, column=0, columnspan=3, pady=10, sticky=(tk.W, tk.E))

        # Obstacle checkboxes
        self.field_border_var = tk.BooleanVar()
        ttk.Checkbutton(
            obstacle_frame, text="Field Border", variable=self.field_border_var
        ).grid(row=0, column=0, sticky=tk.W, pady=2)

        self.penalty_area_var = tk.BooleanVar()
        ttk.Checkbutton(
            obstacle_frame, text="Penalty Area", variable=self.penalty_area_var
        ).grid(row=0, column=1, sticky=tk.W, pady=2)

        self.center_area_var = tk.BooleanVar()
        ttk.Checkbutton(
            obstacle_frame, text="Center Area", variable=self.center_area_var
        ).grid(row=0, column=2, sticky=tk.W, pady=2)

        self.ball_var = tk.BooleanVar()
        ttk.Checkbutton(obstacle_frame, text="Ball", variable=self.ball_var).grid(
            row=0, column=3, sticky=tk.W, pady=2
        )

        # Enemy IDs
        ttk.Label(obstacle_frame, text="Enemy IDs (comma-separated):").grid(
            row=1, column=0, sticky=tk.W, pady=2
        )
        self.enemy_ids_var = tk.StringVar()
        enemy_ids_entry = ttk.Entry(
            obstacle_frame, textvariable=self.enemy_ids_var, width=20
        )
        enemy_ids_entry.grid(row=1, column=1, columnspan=2, sticky=tk.W, pady=2)

        # Ally IDs
        ttk.Label(obstacle_frame, text="Ally IDs (comma-separated):").grid(
            row=2, column=0, sticky=tk.W, pady=2
        )
        self.ally_ids_var = tk.StringVar()
        ally_ids_entry = ttk.Entry(
            obstacle_frame, textvariable=self.ally_ids_var, width=20
        )
        ally_ids_entry.grid(row=2, column=1, columnspan=2, sticky=tk.W, pady=2)

        # Update Obstacles Button
        self.update_obstacles_button = ttk.Button(
            obstacle_frame, text="Update Obstacles", command=self.update_obstacles
        )
        self.update_obstacles_button.grid(row=3, column=0, columnspan=4, pady=10)

        # Response display
        response_frame = ttk.LabelFrame(
            strategy_frame, text="Last Response", padding="5"
        )
        response_frame.grid(
            row=13, column=0, columnspan=3, pady=10, sticky=(tk.W, tk.E, tk.N, tk.S)
        )

        self.response_text = tk.Text(response_frame, height=8, width=70)
        self.response_text.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))

        scrollbar = ttk.Scrollbar(
            response_frame, orient=tk.VERTICAL, command=self.response_text.yview
        )
        scrollbar.grid(row=0, column=1, sticky=(tk.N, tk.S))
        self.response_text.configure(yscrollcommand=scrollbar.set)

        # Configure grid weights for resizing
        strategy_frame.columnconfigure(2, weight=1)
        strategy_frame.rowconfigure(13, weight=1)
        response_frame.columnconfigure(0, weight=1)
        response_frame.rowconfigure(0, weight=1)

        # Disable send button initially
        self.send_button.configure(state="disabled")

        # Disable update obstacles button initially
        self.update_obstacles_button.configure(state="disabled")

        # Vertical Campaign controls
        ttk.Label(strategy_frame, text="Vertical Campaign Robot ID:").grid(
            row=3, column=2, sticky=tk.W, pady=2
        )
        self.vcampaign_robot_var = tk.IntVar(value=0)
        vcampaign_spinbox = ttk.Spinbox(
            strategy_frame,
            from_=0,
            to=15,
            width=8,
            textvariable=self.vcampaign_robot_var,
        )
        vcampaign_spinbox.grid(row=3, column=3, sticky=tk.W, pady=2)

        self.vcampaign_running = False
        self.vcampaign_thread = None
        self.vcampaign_stop_event = threading.Event()

        self.vcampaign_button = ttk.Button(
            strategy_frame,
            text="Start Vertical Campaign",
            command=self.toggle_vcampaign,
        )
        self.vcampaign_button.grid(row=3, column=4, padx=8)

    def setup_pid_tab(self, notebook):
        """Setup the PID tuning tab"""
        # PID tab frame
        pid_frame = ttk.Frame(notebook, padding="10")
        notebook.add(pid_frame, text="PID Tuning")

        # PID Service status
        self.pid_status_label = ttk.Label(
            pid_frame, text="PID Service Status: Checking...", foreground="orange"
        )
        self.pid_status_label.grid(row=0, column=0, columnspan=3, pady=(0, 10))

        # Robot ID for PID
        ttk.Label(pid_frame, text="Robot ID:").grid(
            row=1, column=0, sticky=tk.W, pady=2
        )
        self.pid_robot_id_var = tk.IntVar(value=0)
        pid_robot_id_spinbox = ttk.Spinbox(
            pid_frame, from_=0, to=15, width=10, textvariable=self.pid_robot_id_var
        )
        pid_robot_id_spinbox.grid(row=1, column=1, sticky=tk.W, pady=2)

        # PID Parameters
        ttk.Label(pid_frame, text="Kp (Proportional):").grid(
            row=2, column=0, sticky=tk.W, pady=2
        )
        self.kp_var = tk.DoubleVar(value=3.0)
        kp_entry = ttk.Entry(pid_frame, textvariable=self.kp_var, width=15)
        kp_entry.grid(row=2, column=1, sticky=tk.W, pady=2)

        ttk.Label(pid_frame, text="Ki (Integral):").grid(
            row=3, column=0, sticky=tk.W, pady=2
        )
        self.ki_var = tk.DoubleVar(value=0.2)
        ki_entry = ttk.Entry(pid_frame, textvariable=self.ki_var, width=15)
        ki_entry.grid(row=3, column=1, sticky=tk.W, pady=2)

        ttk.Label(pid_frame, text="Kd (Derivative):").grid(
            row=4, column=0, sticky=tk.W, pady=2
        )
        self.kd_var = tk.DoubleVar(value=1.0)
        kd_entry = ttk.Entry(pid_frame, textvariable=self.kd_var, width=15)
        kd_entry.grid(row=4, column=1, sticky=tk.W, pady=2)

        # Separador visual para clareza
        ttk.Label(pid_frame, text="-" * 50).grid(row=5, column=0, columnspan=3, pady=10)

        # Rótulo
        ttk.Label(pid_frame, text="Kp Angular (Orientação):").grid(
            row=6, column=0, sticky=tk.W, pady=2
        )

        # Variável Tkinter para sincronizar o slider e a caixa de texto
        self.kp_angular_var = tk.DoubleVar(value=2.5)

        # Caixa de Texto (Entry)
        kp_angular_entry = ttk.Entry(
            pid_frame, textvariable=self.kp_angular_var, width=15
        )
        kp_angular_entry.grid(row=6, column=1, sticky=tk.W, pady=2)

        # Slider (Scale)
        kp_angular_slider = ttk.Scale(
            pid_frame,
            from_=0,
            to=10.0,
            orient=tk.HORIZONTAL,
            variable=self.kp_angular_var,
        )
        kp_angular_slider.grid(
            row=7, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=(2, 10)
        )

        # PID Buttons frame
        pid_button_frame = ttk.Frame(pid_frame)
        pid_button_frame.grid(row=8, column=0, columnspan=3, pady=10)

        # Update PID Button
        self.update_pid_button = ttk.Button(
            pid_button_frame, text="Update PID", command=self.update_pid
        )
        self.update_pid_button.grid(row=0, column=0, padx=5)

        # Reset PID Button
        reset_pid_button = ttk.Button(
            pid_button_frame, text="Reset to Defaults", command=self.reset_pid_defaults
        )
        reset_pid_button.grid(row=0, column=1, padx=5)

        # Update Kp Angular Button
        self.update_kp_angular_button = ttk.Button(
            pid_button_frame, text="Update Kp Angular", command=self.update_kp_angular
        )
        self.update_kp_angular_button.grid(row=0, column=2, padx=5)
        self.update_kp_angular_button.configure(state="disabled")

        # PID Presets Frame
        pid_preset_frame = ttk.LabelFrame(pid_frame, text="PID Presets", padding="10")
        pid_preset_frame.grid(
            row=9, column=0, columnspan=3, pady=10, sticky=(tk.W, tk.E)
        )

        # PID preset buttons
        ttk.Button(
            pid_preset_frame,
            text="Conservative",
            command=lambda: self.set_pid_preset(1.5, 0.1, 0.5),
        ).grid(row=0, column=0, padx=2, pady=2, sticky=tk.W + tk.E)
        ttk.Button(
            pid_preset_frame,
            text="Default",
            command=lambda: self.set_pid_preset(3.0, 0.2, 1.0),
        ).grid(row=0, column=1, padx=2, pady=2, sticky=tk.W + tk.E)
        ttk.Button(
            pid_preset_frame,
            text="Aggressive",
            command=lambda: self.set_pid_preset(5.0, 0.5, 1.5),
        ).grid(row=0, column=2, padx=2, pady=2, sticky=tk.W + tk.E)
        ttk.Button(
            pid_preset_frame,
            text="High Precision",
            command=lambda: self.set_pid_preset(4.0, 0.3, 2.0),
        ).grid(row=1, column=0, padx=2, pady=2, sticky=tk.W + tk.E)
        ttk.Button(
            pid_preset_frame,
            text="Fast Response",
            command=lambda: self.set_pid_preset(6.0, 0.1, 0.8),
        ).grid(row=1, column=1, padx=2, pady=2, sticky=tk.W + tk.E)
        ttk.Button(
            pid_preset_frame,
            text="Smooth",
            command=lambda: self.set_pid_preset(2.0, 0.4, 1.2),
        ).grid(row=1, column=2, padx=2, pady=2, sticky=tk.W + tk.E)

        # Configure grid weights for PID preset buttons
        for i in range(3):
            pid_preset_frame.columnconfigure(i, weight=1)

        # PID Response display
        pid_response_frame = ttk.LabelFrame(
            pid_frame, text="PID Update Response", padding="5"
        )
        pid_response_frame.grid(
            row=10, column=0, columnspan=3, pady=10, sticky=(tk.W, tk.E, tk.N, tk.S)
        )

        self.pid_response_text = tk.Text(pid_response_frame, height=8, width=70)
        self.pid_response_text.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))

        pid_scrollbar = ttk.Scrollbar(
            pid_response_frame, orient=tk.VERTICAL, command=self.pid_response_text.yview
        )
        pid_scrollbar.grid(row=0, column=1, sticky=(tk.N, tk.S))
        self.pid_response_text.configure(yscrollcommand=pid_scrollbar.set)

        # Configure grid weights for resizing
        pid_frame.columnconfigure(2, weight=1)
        pid_frame.rowconfigure(10, weight=1)
        pid_response_frame.columnconfigure(0, weight=1)
        pid_response_frame.rowconfigure(0, weight=1)

        # Disable update PID button initially
        self.update_pid_button.configure(state="disabled")

    def check_service_availability(self):
        """Check if the services are available"""
        # Check strategy service
        if self.strategy_client.wait_for_service(timeout_sec=0.1):
            self.status_label.configure(
                text="Strategy Service: Available", foreground="green"
            )
            self.send_button.configure(state="normal")
        else:
            self.status_label.configure(
                text="Strategy Service: Unavailable", foreground="red"
            )
            self.send_button.configure(state="disabled")

        # Check PID service
        if self.pid_client.wait_for_service(timeout_sec=0.1):
            self.pid_status_label.configure(
                text="PID Service: Available", foreground="green"
            )
            self.update_pid_button.configure(state="normal")
        else:
            self.pid_status_label.configure(
                text="PID Service: Unavailable", foreground="red"
            )
            self.update_pid_button.configure(state="disabled")

        # Check Update Obstacles service
        if self.update_obstacles_client.wait_for_service(timeout_sec=0.1):
            self.obstacles_status_label.configure(
                text="Obstacles Service: Available", foreground="green"
            )
            self.update_obstacles_button.configure(state="normal")
        else:
            self.obstacles_status_label.configure(
                text="Obstacles Service: Unavailable", foreground="red"
            )
            self.update_obstacles_button.configure(state="disabled")

        # Check Kp Angular service
        if self.kp_angular_client.wait_for_service(timeout_sec=0.1):
            if hasattr(self, "update_kp_angular_button"):
                self.update_kp_angular_button.configure(state="normal")
        else:
            if hasattr(self, "update_kp_angular_button"):
                self.update_kp_angular_button.configure(state="disabled")

        # Check set orientation service
        if self.set_orientation_client.wait_for_service(timeout_sec=0.1):
            if hasattr(self, "set_orientation_button"):
                self.set_orientation_button.configure(state="normal")
        else:
            if hasattr(self, "set_orientation_button"):
                self.set_orientation_button.configure(state="disabled")

    def set_preset(self, x, y, vx, vy):
        """Set preset values for quick testing"""
        self.position_x_var.set(x)
        self.position_y_var.set(y)
        self.velocity_x_var.set(vx)
        self.velocity_y_var.set(vy)

    def set_pid_preset(self, kp, ki, kd):
        """Set PID preset values"""
        self.kp_var.set(kp)
        self.ki_var.set(ki)
        self.kd_var.set(kd)

    def reset_pid_defaults(self):
        """Reset PID values to defaults"""
        self.pid_robot_id_var.set(0)
        self.kp_var.set(3.0)
        self.ki_var.set(0.2)
        self.kd_var.set(1.0)

    def clear_fields(self):
        """Clear all input fields"""
        self.robot_id_var.set(0)
        self.position_x_var.set(0.0)
        self.position_y_var.set(0.0)
        self.velocity_x_var.set(0.0)
        self.velocity_y_var.set(0.0)
        self.orientation_var.set(0.0)

    def send_command(self):
        """Send strategy command service request"""
        if not self.strategy_client.wait_for_service(timeout_sec=1.0):
            messagebox.showerror("Error", "Strategy command service is not available")
            return

        try:
            # Create service request
            request = StrategyCommand.Request()
            request.id = self.robot_id_var.get()
            request.position_x = float(self.position_x_var.get())
            request.position_y = float(self.position_y_var.get())
            request.velocity_x = float(self.velocity_x_var.get())
            request.velocity_y = float(self.velocity_y_var.get())

            # Log the request
            self.get_logger().info(
                f"Sending strategy command: ID={request.id}, "
                f"Pos=({request.position_x}, {request.position_y}), "
                f"Vel=({request.velocity_x}, {request.velocity_y})"
            )

            # Send async request
            future = self.strategy_client.call_async(request)

            # Add callback to handle response
            future.add_done_callback(self.handle_response)

            # Update response display with request info
            self.update_response_display(
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
        """Send update obstacles service request"""
        if not self.update_obstacles_client.wait_for_service(timeout_sec=1.0):
            messagebox.showerror("Error", "Update obstacles service is not available")
            return

        try:
            # Create service request
            request = UpdateObstacle.Request()
            request.id = self.robot_id_var.get()
            request.field_border = self.field_border_var.get()
            request.penalty_area = self.penalty_area_var.get()
            request.center_area = self.center_area_var.get()
            request.ball = self.ball_var.get()

            # Parse enemy IDs
            enemy_ids_str = self.enemy_ids_var.get().strip()
            if enemy_ids_str:
                request.enemy_ids = [
                    int(id.strip()) for id in enemy_ids_str.split(",") if id.strip()
                ]
            else:
                request.enemy_ids = []

            # Parse ally IDs
            ally_ids_str = self.ally_ids_var.get().strip()
            if ally_ids_str:
                request.ally_ids = [
                    int(id.strip()) for id in ally_ids_str.split(",") if id.strip()
                ]
            else:
                request.ally_ids = []

            # Log the request
            self.get_logger().info(
                f"Sending update obstacles: ID={request.id}, "
                f"Field Border={request.field_border}, Penalty={request.penalty_area}, "
                f"Center={request.center_area}, Ball={request.ball}, "
                f"Enemy IDs={request.enemy_ids}, Ally IDs={request.ally_ids}"
            )

            # Send async request
            future = self.update_obstacles_client.call_async(request)

            # Add callback to handle response
            future.add_done_callback(self.handle_obstacles_response)

            # Update response display with request info
            self.update_response_display(
                f"Obstacles Update sent at {time.strftime('%H:%M:%S')}:\n"
                f"Robot ID: {request.id}\n"
                f"Field Border: {request.field_border}\n"
                f"Penalty Area: {request.penalty_area}\n"
                f"Center Area: {request.center_area}\n"
                f"Ball: {request.ball}\n"
                f"Enemy IDs: {request.enemy_ids}\n"
                f"Ally IDs: {request.ally_ids}\n"
                f"Waiting for response...\n"
            )

        except ValueError as e:
            messagebox.showerror("Input Error", f"Invalid input values: {e}")
        except Exception as e:
            messagebox.showerror("Error", f"Failed to send obstacles update: {e}")
            self.get_logger().error(f"Failed to send obstacles update: {e}")

    def update_pid(self):
        """Send PID update service request"""
        if not self.pid_client.wait_for_service(timeout_sec=1.0):
            messagebox.showerror("Error", "PID service is not available")
            return

        try:
            # Create PID service request
            request = ControlParams.Request()
            request.id = self.pid_robot_id_var.get()
            request.kp = float(self.kp_var.get())
            request.ki = float(self.ki_var.get())
            request.kd = float(self.kd_var.get())

            # Log the request
            self.get_logger().info(
                f"Sending PID update: ID={request.id}, "
                f"Kp={request.kp}, Ki={request.ki}, Kd={request.kd}"
            )

            # Send async request
            future = self.pid_client.call_async(request)

            # Add callback to handle response
            future.add_done_callback(self.handle_pid_response)

            # Update response display with request info
            self.update_pid_response_display(
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

    def handle_response(self, future):
        """Handle service response"""
        try:
            response = future.result()
            success_text = "SUCCESS" if response.success else "FAILED"

            self.get_logger().info(f"Strategy command response: {success_text}")

            # Update response display
            response_info = f"Response received at {time.strftime('%H:%M:%S')}:\n"
            response_info += f"Success: {success_text}\n"
            response_info += "=" * 40 + "\n"

            self.update_response_display(response_info)

            # Show popup for failed commands
            if not response.success:
                messagebox.showwarning(
                    "Command Failed", "The strategy command was not successful"
                )

        except Exception as e:
            error_msg = f"Service call failed: {e}"
            self.get_logger().error(error_msg)
            self.update_response_display(
                f"ERROR at {time.strftime('%H:%M:%S')}: {error_msg}\n"
            )
            messagebox.showerror("Service Error", error_msg)

    def handle_obstacles_response(self, future):
        """Handle obstacles service response"""
        try:
            response = future.result()
            success_text = "SUCCESS" if response.success else "FAILED"

            self.get_logger().info(f"Update obstacles response: {success_text}")

            # Update response display
            response_info = (
                f"Obstacles Response received at {time.strftime('%H:%M:%S')}:\n"
            )
            response_info += f"Success: {success_text}\n"
            response_info += "=" * 40 + "\n"

            self.update_response_display(response_info)

            # Show popup for failed commands
            if not response.success:
                messagebox.showwarning(
                    "Update Failed", "The obstacles update was not successful"
                )

        except Exception as e:
            error_msg = f"Obstacles service call failed: {e}"
            self.get_logger().error(error_msg)
            self.update_response_display(
                f"ERROR at {time.strftime('%H:%M:%S')}: {error_msg}\n"
            )
            messagebox.showerror("Service Error", error_msg)

    def handle_pid_response(self, future):
        """Handle PID service response"""
        try:
            response = future.result()
            success_text = "SUCCESS" if response.success else "FAILED"

            self.get_logger().info(f"PID update response: {success_text}")

            # Update response display
            response_info = f"PID Response received at {time.strftime('%H:%M:%S')}:\n"
            response_info += f"Success: {success_text}\n"
            response_info += "=" * 40 + "\n"

            self.update_pid_response_display(response_info)

            # Show popup for failed commands
            if not response.success:
                messagebox.showwarning(
                    "PID Update Failed", "The PID update was not successful"
                )

        except Exception as e:
            error_msg = f"PID service call failed: {e}"
            self.get_logger().error(error_msg)
            self.update_pid_response_display(
                f"ERROR at {time.strftime('%H:%M:%S')}: {error_msg}\n"
            )
            messagebox.showerror("PID Service Error", error_msg)

    def update_response_display(self, text):
        """Update the response text widget"""

        def update():
            self.response_text.insert(tk.END, text)
            self.response_text.see(tk.END)

        # Schedule GUI update on main thread
        self.root.after(0, update)

    def update_pid_response_display(self, text):
        """Update the PID response text widget"""

        def update():
            self.pid_response_text.insert(tk.END, text)
            self.pid_response_text.see(tk.END)

        # Schedule GUI update on main thread
        self.root.after(0, update)

    def spin_ros(self):
        """Spin ROS2 in a separate thread"""
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

    def run(self):
        """Run the GUI application"""
        try:
            self.root.mainloop()
        finally:
            self.get_logger().info("Shutting down Strategy Command GUI")

    def toggle_campaign(self):
        """Toggle the alternate-goal campaign on or off"""
        if not self.campaign_running:
            # start campaign
            self.campaign_stop_event.clear()
            self.campaign_thread = threading.Thread(
                target=self._campaign_runner, daemon=True
            )
            self.campaign_thread.start()
            self.campaign_running = True
            self.campaign_button.configure(text="Stop Campaign")
            self.get_logger().info("Campaign started")
        else:
            # stop campaign
            self.campaign_stop_event.set()
            self.campaign_running = False
            self.campaign_button.configure(text="Start Campaign")
            self.get_logger().info("Campaign stopped")

    def toggle_vcampaign(self):
        """Toggle the vertical campaign on or off"""
        if not self.vcampaign_running:
            # start vertical campaign
            self.vcampaign_stop_event.clear()
            self.vcampaign_thread = threading.Thread(
                target=self._vcampaign_runner, daemon=True
            )
            self.vcampaign_thread.start()
            self.vcampaign_running = True
            self.vcampaign_button.configure(text="Stop Vertical Campaign")
            self.get_logger().info("Vertical campaign started")
        else:
            # stop vertical campaign
            self.vcampaign_stop_event.set()
            self.vcampaign_running = False
            self.vcampaign_button.configure(text="Start Vertical Campaign")
            self.get_logger().info("Vertical campaign stopped")

    def _send_strategy(
        self, robot_id: int, x: float, y: float, vx: float = 0.0, vy: float = 0.0
    ):
        if not self.strategy_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning(
                "Strategy service unavailable when sending campaign command"
            )
            return False
        try:
            request = StrategyCommand.Request()
            request.id = int(robot_id)
            request.position_x = float(x)
            request.position_y = float(y)
            request.velocity_x = float(vx)
            request.velocity_y = float(vy)
            # optional team color handling
            team_color = getattr(self, "team_color_var", None)
            if team_color and hasattr(request, "team_color"):
                # map string to boolean expected elsewhere (True == Yellow)
                request.team_color = self.team_color_var.get() == "Yellow"
            future = self.strategy_client.call_async(request)
            # no blocking; caller may rely on visual feedback
            self.get_logger().info(f"Campaign sent to robot {robot_id}: ({x},{y})")
            return True
        except Exception as e:
            self.get_logger().error(f"Failed to send campaign command: {e}")
            return False

    def _campaign_runner(self):
        robot_id = int(self.campaign_robot_var.get())
        our_goal = (-2250, 0)
        enemy_goal = (2250, 0)
        target_toggle = False
        while not self.campaign_stop_event.is_set():
            if target_toggle:
                self._send_strategy(robot_id, enemy_goal[0], enemy_goal[1])
            else:
                self._send_strategy(robot_id, our_goal[0], our_goal[1])
            target_toggle = not target_toggle
            # wait 5 seconds or until stopped
            self.campaign_stop_event.wait(5.0)

    def _vcampaign_runner(self):
        robot_id = int(self.vcampaign_robot_var.get())
        top = (0, 1500)
        bottom = (0, -1500)
        target_toggle = False
        while not self.vcampaign_stop_event.is_set():
            if target_toggle:
                self._send_strategy(robot_id, bottom[0], bottom[1])
            else:
                self._send_strategy(robot_id, top[0], top[1])
            target_toggle = not target_toggle
            # wait 5 seconds or until stopped
            self.vcampaign_stop_event.wait(5.0)

    def on_team_color_changed(self, event=None):
        """Called when the user changes the team color combobox. Calls the set_team_color service."""
        # Map selection to bool: Yellow -> True, Blue -> False
        selection = self.team_color_var.get()
        is_yellow = selection == "Yellow"

        # Call service
        if not self.set_team_color_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning("set_team_color service unavailable")
            return

        try:
            req = SetBool.Request()
            req.data = bool(is_yellow)
            future = self.set_team_color_client.call_async(req)

            def _cb(fut):
                try:
                    resp = fut.result()
                    if resp.success:
                        self.get_logger().info(f"Team color set to {selection}")
                    else:
                        self.get_logger().warning(
                            f"Failed to set team color: {resp.message}"
                        )
                except Exception as e:
                    self.get_logger().error(f"Error calling set_team_color: {e}")

            future.add_done_callback(_cb)
        except Exception as e:
            self.get_logger().error(f"Exception while calling set_team_color: {e}")

    def update_kp_angular(self):
        """Send Kp angular update service request"""
        if not self.kp_angular_client.wait_for_service(timeout_sec=1.0):
            messagebox.showerror(
                "Error", "Angular Kp service 'update_kp_angular' is not available"
            )
            return

        try:
            request = SetKp.Request()
            request.kp = float(self.kp_angular_var.get())

            self.get_logger().info(f"Sending Angular Kp update: Kp={request.kp}")

            future = self.kp_angular_client.call_async(request)
            future.add_done_callback(self.handle_pid_response)

            self.update_pid_response_display(
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
        """Send set orientation service request"""
        if not self.set_orientation_client.wait_for_service(timeout_sec=1.0):
            messagebox.showerror("Error", "Set orientation service is not available")
            return

        try:
            # Create service request
            request = SetOrientation.Request()
            request.robot_id = (
                self.robot_id_var.get()
            )  # Usa o mesmo robot_id da aba Strategy
            request.orientation = float(self.orientation_var.get())

            # Log the request
            self.get_logger().info(
                f"Sending set orientation: ID={request.robot_id}, Orientation={request.orientation}"
            )

            # Send async request
            future = self.set_orientation_client.call_async(request)

            # Add callback to handle response
            future.add_done_callback(
                self.handle_response
            )  # Usa o mesmo handler da strategy

            # Update response display with request info
            self.update_response_display(
                f"Set Orientation request sent at {time.strftime('%H:%M:%S')}:\n"
                f"Robot ID: {request.robot_id}\n"
                f"Orientation: {request.orientation}\n"
                f"Waiting for response...\n"
            )

        except ValueError as e:
            messagebox.showerror("Input Error", f"Invalid orientation value: {e}")
        except Exception as e:
            messagebox.showerror("Error", f"Failed to set orientation: {e}")
            self.get_logger().error(f"Failed to set orientation: {e}")


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
