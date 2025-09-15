#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.client import Client
import tkinter as tk
from tkinter import ttk, messagebox
import threading
from system_interfaces.srv import StrategyCommand, ControlParams,SetKp
# No topo do arquivo
import time


class StrategyCommandGUI(Node):
    def __init__(self):
        super().__init__('strategy_command_gui')
        
        # Create service clients
        self.strategy_client = self.create_client(StrategyCommand, 'strategy_command')
        self.pid_client = self.create_client(ControlParams, 'update_pid')
        self.kp_angular_client = self.create_client(SetKp, 'update_kp_angular')
        
        # Wait for services to become available
        self.get_logger().info('Waiting for services...')
                
        # Initialize GUI
        self.setup_gui()
        
        # Start ROS2 spinning in a separate thread
        self.ros_thread = threading.Thread(target=self.spin_ros, daemon=True)
        self.ros_thread.start()
        
        # Check service availability periodically
        self.check_service_timer = self.create_timer(1.0, self.check_service_availability)
        
    def check_service_availability(self):
        """Check if the services are available"""
            # ... seu código para strategy_client e pid_client ...

            # --- ADIÇÃO ---
            # Check Kp Angular service
        if self.kp_angular_client.wait_for_service(timeout_sec=0.1):
        # Habilita o botão se o serviço estiver online
        # (Vamos criar o botão no próximo passo)
            if hasattr(self, 'update_kp_angular_button'):
                self.update_kp_angular_button.configure(state='normal')
        else:
            if hasattr(self, 'update_kp_angular_button'):
                self.update_kp_angular_button.configure(state='disabled')

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
        self.status_label = ttk.Label(strategy_frame, text="Service Status: Checking...", foreground="orange")
        self.status_label.grid(row=0, column=0, columnspan=3, pady=(0, 10))
        
        # Robot ID
        ttk.Label(strategy_frame, text="Robot ID:").grid(row=1, column=0, sticky=tk.W, pady=2)
        self.robot_id_var = tk.IntVar(value=0)
        robot_id_spinbox = ttk.Spinbox(strategy_frame, from_=0, to=15, width=10, textvariable=self.robot_id_var)
        robot_id_spinbox.grid(row=1, column=1, sticky=tk.W, pady=2)
        
        # Position X
        ttk.Label(strategy_frame, text="Position X (mm):").grid(row=2, column=0, sticky=tk.W, pady=2)
        self.position_x_var = tk.DoubleVar(value=0.0)
        position_x_entry = ttk.Entry(strategy_frame, textvariable=self.position_x_var, width=15)
        position_x_entry.grid(row=2, column=1, sticky=tk.W, pady=2)
        
        # Position Y
        ttk.Label(strategy_frame, text="Position Y (mm):").grid(row=3, column=0, sticky=tk.W, pady=2)
        self.position_y_var = tk.DoubleVar(value=0.0)
        position_y_entry = ttk.Entry(strategy_frame, textvariable=self.position_y_var, width=15)
        position_y_entry.grid(row=3, column=1, sticky=tk.W, pady=2)
        
        # Velocity X
        ttk.Label(strategy_frame, text="Velocity X (mm/s):").grid(row=4, column=0, sticky=tk.W, pady=2)
        self.velocity_x_var = tk.DoubleVar(value=0.0)
        velocity_x_entry = ttk.Entry(strategy_frame, textvariable=self.velocity_x_var, width=15)
        velocity_x_entry.grid(row=4, column=1, sticky=tk.W, pady=2)
        
        # Velocity Y
        ttk.Label(strategy_frame, text="Velocity Y (mm/s):").grid(row=5, column=0, sticky=tk.W, pady=2)
        self.velocity_y_var = tk.DoubleVar(value=0.0)
        velocity_y_entry = ttk.Entry(strategy_frame, textvariable=self.velocity_y_var, width=15)
        velocity_y_entry.grid(row=5, column=1, sticky=tk.W, pady=2)
        
        # Buttons frame
        button_frame = ttk.Frame(strategy_frame)
        button_frame.grid(row=6, column=0, columnspan=3, pady=20)
        
        # Send Command Button
        self.send_button = ttk.Button(button_frame, text="Send Command", command=self.send_command)
        self.send_button.grid(row=0, column=0, padx=5)
        
        # Clear Button
        clear_button = ttk.Button(button_frame, text="Clear", command=self.clear_fields)
        clear_button.grid(row=0, column=1, padx=5)
        
        # Preset Commands Frame
        preset_frame = ttk.LabelFrame(strategy_frame, text="Preset Positions", padding="10")
        preset_frame.grid(row=7, column=0, columnspan=3, pady=10, sticky=(tk.W, tk.E))
        
        ttk.Button(preset_frame, text="Center", command=lambda: self.set_preset(0, 0, 0, 0)).grid(row=0, column=0, padx=2, pady=2, sticky=tk.W+tk.E)
        ttk.Button(preset_frame, text="Our Goal", command=lambda: self.set_preset(-2250, 0, 0, 0)).grid(row=0, column=1, padx=2, pady=2, sticky=tk.W+tk.E)
        ttk.Button(preset_frame, text="Enemy Goal", command=lambda: self.set_preset(2250, 0, 0, 0)).grid(row=0, column=2, padx=2, pady=2, sticky=tk.W+tk.E)
        ttk.Button(preset_frame, text="Our Penalty", command=lambda: self.set_preset(-1250, 0, 0, 0)).grid(row=0, column=3, padx=2, pady=2, sticky=tk.W+tk.E)
        
        ttk.Button(preset_frame, text="Top Left", command=lambda: self.set_preset(-2250, 1500, 0, 0)).grid(row=1, column=0, padx=2, pady=2, sticky=tk.W+tk.E)
        ttk.Button(preset_frame, text="Top Right", command=lambda: self.set_preset(2250, 1500, 0, 0)).grid(row=1, column=1, padx=2, pady=2, sticky=tk.W+tk.E)
        ttk.Button(preset_frame, text="Bottom Left", command=lambda: self.set_preset(-2250, -1500, 0, 0)).grid(row=1, column=2, padx=2, pady=2, sticky=tk.W+tk.E)
        ttk.Button(preset_frame, text="Bottom Right", command=lambda: self.set_preset(2250, -1500, 0, 0)).grid(row=1, column=3, padx=2, pady=2, sticky=tk.W+tk.E)
        
        # Configure grid weights for preset buttons
        for i in range(4):
            preset_frame.columnconfigure(i, weight=1)
        
        # Response display
        response_frame = ttk.LabelFrame(strategy_frame, text="Last Response", padding="5")
        response_frame.grid(row=8, column=0, columnspan=3, pady=10, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        self.response_text = tk.Text(response_frame, height=8, width=70)
        self.response_text.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        scrollbar = ttk.Scrollbar(response_frame, orient=tk.VERTICAL, command=self.response_text.yview)
        scrollbar.grid(row=0, column=1, sticky=(tk.N, tk.S))
        self.response_text.configure(yscrollcommand=scrollbar.set)
        
        # Configure grid weights for resizing
        strategy_frame.columnconfigure(2, weight=1)
        strategy_frame.rowconfigure(8, weight=1)
        response_frame.columnconfigure(0, weight=1)
        response_frame.rowconfigure(0, weight=1)
        
        # Disable send button initially
        self.send_button.configure(state='disabled')
        
    def setup_pid_tab(self, notebook):
        """Setup the PID tuning tab"""
        # PID tab frame
        pid_frame = ttk.Frame(notebook, padding="10")
        notebook.add(pid_frame, text="PID Tuning")
               
        # PID Service status
        self.pid_status_label = ttk.Label(pid_frame, text="PID Service Status: Checking...", foreground="orange")
        self.pid_status_label.grid(row=0, column=0, columnspan=3, pady=(0, 10))
        
        # Robot ID for PID
        ttk.Label(pid_frame, text="Robot ID:").grid(row=1, column=0, sticky=tk.W, pady=2)
        self.pid_robot_id_var = tk.IntVar(value=0)
        pid_robot_id_spinbox = ttk.Spinbox(pid_frame, from_=0, to=15, width=10, textvariable=self.pid_robot_id_var)
        pid_robot_id_spinbox.grid(row=1, column=1, sticky=tk.W, pady=2)
        
        # PID Parameters
        ttk.Label(pid_frame, text="Kp (Proportional):").grid(row=2, column=0, sticky=tk.W, pady=2)
        self.kp_var = tk.DoubleVar(value=3.0)
        kp_entry = ttk.Entry(pid_frame, textvariable=self.kp_var, width=15)
        kp_entry.grid(row=2, column=1, sticky=tk.W, pady=2)
        
        ttk.Label(pid_frame, text="Ki (Integral):").grid(row=3, column=0, sticky=tk.W, pady=2)
        self.ki_var = tk.DoubleVar(value=0.2)
        ki_entry = ttk.Entry(pid_frame, textvariable=self.ki_var, width=15)
        ki_entry.grid(row=3, column=1, sticky=tk.W, pady=2)
        
        ttk.Label(pid_frame, text="Kd (Derivative):").grid(row=4, column=0, sticky=tk.W, pady=2)
        self.kd_var = tk.DoubleVar(value=1.0)
        kd_entry = ttk.Entry(pid_frame, textvariable=self.kd_var, width=15)
        kd_entry.grid(row=4, column=1, sticky=tk.W, pady=2)
          # --- ADICIONADO: Widgets para o Kp Angular ---
        # Separador visual para clareza
        ttk.Label(pid_frame, text="-"*50).grid(row=5, column=0, columnspan=3, pady=10)

        # Rótulo
        ttk.Label(pid_frame, text="Kp Angular (Orientação):").grid(row=6, column=0, sticky=tk.W, pady=2)
        
        # Variável Tkinter para sincronizar o slider e a caixa de texto
        self.kp_angular_var = tk.DoubleVar(value=2.5)
        
        # Caixa de Texto (Entry)
        kp_angular_entry = ttk.Entry(pid_frame, textvariable=self.kp_angular_var, width=15)
        kp_angular_entry.grid(row=6, column=1, sticky=tk.W, pady=2)
        
        # Slider (Scale)
        kp_angular_slider = ttk.Scale(pid_frame, from_=0, to=10.0, orient=tk.HORIZONTAL,
                                      variable=self.kp_angular_var)
        kp_angular_slider.grid(row=7, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=(2, 10))

        # --- FIM DA ADIÇÃO ---

        # # PID Buttons frame
        # pid_button_frame = ttk.Frame(pid_frame)
        # pid_button_frame.grid(row=5, column=0, columnspan=3, pady=20)
        
        # # Update PID Button
        # self.update_pid_button = ttk.Button(pid_button_frame, text="Update PID", command=self.update_pid)
        # self.update_pid_button.grid(row=0, column=0, padx=5)
        
        # # Reset PID Button
        # reset_pid_button = ttk.Button(pid_button_frame, text="Reset to Defaults", command=self.reset_pid_defaults)
        # reset_pid_button.grid(row=0, column=1, padx=5)

         # PID Buttons frame
        pid_button_frame = ttk.Frame(pid_frame)
        pid_button_frame.grid(row=8, column=0, columnspan=3, pady=10) # Era row=5
        
        # Update PID Button
        self.update_pid_button = ttk.Button(pid_button_frame, text="Update PID", command=self.update_pid)
        self.update_pid_button.grid(row=0, column=0, padx=5)
        
        # Reset PID Button
        reset_pid_button = ttk.Button(pid_button_frame, text="Reset to Defaults", command=self.reset_pid_defaults)
        reset_pid_button.grid(row=0, column=1, padx=5)

        
        # # PID Presets Frame
        # pid_preset_frame = ttk.LabelFrame(pid_frame, text="PID Presets", padding="10")
        # pid_preset_frame.grid(row=6, column=0, columnspan=3, pady=10, sticky=(tk.W, tk.E))

         # --- ADICIONADO: Botão para o Kp Angular ---
        self.update_kp_angular_button = ttk.Button(pid_button_frame, text="Update Kp Angular", command=self.update_kp_angular)
        self.update_kp_angular_button.grid(row=0, column=2, padx=5)
        self.update_kp_angular_button.configure(state='disabled')
        
        # --- MODIFICADO: Ajuste do 'row' para os widgets abaixo ---
        # PID Presets Frame
        pid_preset_frame = ttk.LabelFrame(pid_frame, text="PID Presets", padding="10")
        pid_preset_frame.grid(row=9, column=0, columnspan=3, pady=10, sticky=(tk.W, tk.E)) # Era row=6

        
        # PID preset buttons
        ttk.Button(pid_preset_frame, text="Conservative", command=lambda: self.set_pid_preset(1.5, 0.1, 0.5)).grid(row=0, column=0, padx=2, pady=2, sticky=tk.W+tk.E)
        ttk.Button(pid_preset_frame, text="Default", command=lambda: self.set_pid_preset(3.0, 0.2, 1.0)).grid(row=0, column=1, padx=2, pady=2, sticky=tk.W+tk.E)
        ttk.Button(pid_preset_frame, text="Aggressive", command=lambda: self.set_pid_preset(5.0, 0.5, 1.5)).grid(row=0, column=2, padx=2, pady=2, sticky=tk.W+tk.E)
        ttk.Button(pid_preset_frame, text="High Precision", command=lambda: self.set_pid_preset(4.0, 0.3, 2.0)).grid(row=1, column=0, padx=2, pady=2, sticky=tk.W+tk.E)
        ttk.Button(pid_preset_frame, text="Fast Response", command=lambda: self.set_pid_preset(6.0, 0.1, 0.8)).grid(row=1, column=1, padx=2, pady=2, sticky=tk.W+tk.E)
        ttk.Button(pid_preset_frame, text="Smooth", command=lambda: self.set_pid_preset(2.0, 0.4, 1.2)).grid(row=1, column=2, padx=2, pady=2, sticky=tk.W+tk.E)
        
        # Configure grid weights for PID preset buttons
        for i in range(3):
            pid_preset_frame.columnconfigure(i, weight=1)
        
        # # PID Response display
        # pid_response_frame = ttk.LabelFrame(pid_frame, text="PID Update Response", padding="5")
        # pid_response_frame.grid(row=7, column=0, columnspan=3, pady=10, sticky=(tk.W, tk.E, tk.N, tk.S))
        # --- MODIFICADO: Ajuste do 'row' para os widgets abaixo ---
        # PID Response display
        pid_response_frame = ttk.LabelFrame(pid_frame, text="PID Update Response", padding="5")
        pid_response_frame.grid(row=10, column=0, columnspan=3, pady=10, sticky=(tk.W, tk.E, tk.N, tk.S)) # Era row=7
        
        self.pid_response_text = tk.Text(pid_response_frame, height=8, width=70)
        self.pid_response_text.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        pid_scrollbar = ttk.Scrollbar(pid_response_frame, orient=tk.VERTICAL, command=self.pid_response_text.yview)
        pid_scrollbar.grid(row=0, column=1, sticky=(tk.N, tk.S))
        self.pid_response_text.configure(yscrollcommand=pid_scrollbar.set)
        
        # Configure grid weights for resizing
        pid_frame.columnconfigure(2, weight=1)
        pid_frame.rowconfigure(7, weight=1)
        pid_response_frame.columnconfigure(0, weight=1)
        pid_response_frame.rowconfigure(0, weight=1)
        
        # Disable update PID button initially
        self.update_pid_button.configure(state='disabled')
        
    def check_service_availability(self):
        """Check if the services are available"""
        # Check strategy service
        if self.strategy_client.wait_for_service(timeout_sec=0.1):
            self.status_label.configure(text="Strategy Service: Available", foreground="green")
            self.send_button.configure(state='normal')
        else:
            self.status_label.configure(text="Strategy Service: Unavailable", foreground="red")
            self.send_button.configure(state='disabled')
            
        # Check PID service
        if self.pid_client.wait_for_service(timeout_sec=0.1):
            self.pid_status_label.configure(text="PID Service: Available", foreground="green")
            self.update_pid_button.configure(state='normal')
        else:
            self.pid_status_label.configure(text="PID Service: Unavailable", foreground="red")
            self.update_pid_button.configure(state='disabled')
          
          # --- ADICIONADO: Checagem do serviço do Kp Angular ---
        if self.kp_angular_client.wait_for_service(timeout_sec=0.1):
            if hasattr(self, 'update_kp_angular_button'): # Garante que o botão já foi criado
                self.update_kp_angular_button.configure(state='normal')
        else:
            if hasattr(self, 'update_kp_angular_button'):
                self.update_kp_angular_button.configure(state='disabled')
    
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
            self.get_logger().info(f'Sending strategy command: ID={request.id}, '
                                 f'Pos=({request.position_x}, {request.position_y}), '
                                 f'Vel=({request.velocity_x}, {request.velocity_y})')
            
            # Send async request
            future = self.strategy_client.call_async(request)
            
            # Add callback to handle response
            future.add_done_callback(self.handle_response)
            
            # Update response display with request info
            self.update_response_display(f"Request sent at {time.strftime('%H:%M:%S')}:\n"
                                       f"Robot ID: {request.id}\n"
                                       f"Position: ({request.position_x}, {request.position_y})\n"
                                       f"Velocity: ({request.velocity_x}, {request.velocity_y})\n"
                                       f"Waiting for response...\n")
            
        except ValueError as e:
            messagebox.showerror("Input Error", f"Invalid input values: {e}")
        except Exception as e:
            messagebox.showerror("Error", f"Failed to send command: {e}")
            self.get_logger().error(f'Failed to send strategy command: {e}')
    
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
            self.get_logger().info(f'Sending PID update: ID={request.id}, '
                                 f'Kp={request.kp}, Ki={request.ki}, Kd={request.kd}')
            
            # Send async request
            future = self.pid_client.call_async(request)
            
            # Add callback to handle response
            future.add_done_callback(self.handle_pid_response)
            
            # Update response display with request info
            self.update_pid_response_display(f"PID Update sent at {time.strftime('%H:%M:%S')}:\n"
                                           f"Robot ID: {request.id}\n"
                                           f"Kp: {request.kp}, Ki: {request.ki}, Kd: {request.kd}\n"
                                           f"Waiting for response...\n")
            
        except ValueError as e:
            messagebox.showerror("Input Error", f"Invalid PID values: {e}")
        except Exception as e:
            messagebox.showerror("Error", f"Failed to update PID: {e}")
            self.get_logger().error(f'Failed to update PID: {e}')
    
    def handle_response(self, future):
        """Handle service response"""
        try:
            response = future.result()
            success_text = "SUCCESS" if response.success else "FAILED"
            
            self.get_logger().info(f'Strategy command response: {success_text}')
            
            # Update response display
            response_info = f"Response received at {time.strftime('%H:%M:%S')}:\n"
            response_info += f"Success: {success_text}\n"
            response_info += "="*40 + "\n"
            
            self.update_response_display(response_info)
            
            # Show popup for failed commands
            if not response.success:
                messagebox.showwarning("Command Failed", "The strategy command was not successful")
                
        except Exception as e:
            error_msg = f"Service call failed: {e}"
            self.get_logger().error(error_msg)
            self.update_response_display(f"ERROR at {time.strftime('%H:%M:%S')}: {error_msg}\n")
            messagebox.showerror("Service Error", error_msg)
    
    def handle_pid_response(self, future):
        """Handle PID service response"""
        try:
            response = future.result()
            success_text = "SUCCESS" if response.success else "FAILED"
            
            self.get_logger().info(f'PID update response: {success_text}')
            
            # Update response display
            response_info = f"PID Response received at {time.strftime('%H:%M:%S')}:\n"
            response_info += f"Success: {success_text}\n"
            response_info += "="*40 + "\n"
            
            self.update_pid_response_display(response_info)
            
            # Show popup for failed commands
            if not response.success:
                messagebox.showwarning("PID Update Failed", "The PID update was not successful")
                
        except Exception as e:
            error_msg = f"PID service call failed: {e}"
            self.get_logger().error(error_msg)
            self.update_pid_response_display(f"ERROR at {time.strftime('%H:%M:%S')}: {error_msg}\n")
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
            self.get_logger().info('Shutting down Strategy Command GUI')
            
    # --- ADICIONADO: Nova função para atualizar o Kp Angular ---
    def update_kp_angular(self):
        """Send Kp angular update service request"""
        if not self.kp_angular_client.wait_for_service(timeout_sec=1.0):
            messagebox.showerror("Error", "Angular Kp service 'update_kp_angular' is not available")
            return
        
        try:
            request = SetKp.Request()
            request.kp = float(self.kp_angular_var.get())
            
            self.get_logger().info(f'Sending Angular Kp update: Kp={request.kp}')
            
            future = self.kp_angular_client.call_async(request)
            future.add_done_callback(self.handle_pid_response) # Podemos reusar o mesmo handler
            
            self.update_pid_response_display(f"Angular Kp Update sent at {time.strftime('%H:%M:%S')}:\n"
                                           f"Kp: {request.kp}\n"
                                           f"Waiting for response...\n")
            
        except ValueError as e:
            messagebox.showerror("Input Error", f"Invalid Angular Kp value: {e}")
        except Exception as e:
            messagebox.showerror("Error", f"Failed to update Angular Kp: {e}")
            self.get_logger().error(f'Failed to update Angular Kp: {e}')


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


if __name__ == '__main__':
    main()





#!/usr/bin/env python3
# """
# GUI para enviar comandos de estratégia e sintonizar parâmetros de controle em tempo real.
# Utiliza Tkinter para a interface gráfica e rclpy para a comunicação com o ROS 2.
# """
# import rclpy
# from rclpy.node import Node
# import tkinter as tk
# from tkinter import ttk, messagebox
# import threading
# from system_interfaces.srv import StrategyCommand, ControlParams, SetKp
# import time


# class StrategyCommandGUI(Node):
#     """
#     Nó ROS 2 que gerencia uma GUI em Tkinter para interagir com o sistema de robôs.
#     """
#     def __init__(self):
#         super().__init__('strategy_command_gui')
        
#         # --- Nomes dos Serviços (Constantes) ---
#         STRATEGY_SERVICE = 'strategy_command'
#         PID_SERVICE = 'update_pid'
#         KP_ANGULAR_SERVICE = 'update_kp_angular'

#         # --- Clientes dos Serviços ROS 2 ---
#         self.strategy_client = self.create_client(StrategyCommand, STRATEGY_SERVICE)
#         self.pid_client = self.create_client(ControlParams, PID_SERVICE)
#         self.kp_angular_client = self.create_client(SetKp, KP_ANGULAR_SERVICE)
        
#         self.get_logger().info('Aguardando os serviços ficarem online...')
                
#         # --- Inicialização da GUI ---
#         self.root = tk.Tk()
#         self.setup_gui()
        
#         # --- Threads e Timers ---
#         # ROS 2 roda em uma thread separada para não travar a GUI
#         self.ros_thread = threading.Thread(target=self.spin_ros, daemon=True)
#         self.ros_thread.start()
        
#         # Timer para verificar a disponibilidade dos serviços periodicamente
#         self.check_service_timer = self.create_timer(1.0, self.check_service_availability)
        
#     def setup_gui(self):
#         """Configura a janela principal e as abas da GUI."""
#         self.root.title("Strategy Command GUI")
#         self.root.geometry("800x700")
        
#         notebook = ttk.Notebook(self.root)
#         notebook.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
#         self.setup_strategy_tab(notebook)
#         self.setup_pid_tab(notebook)
        
#     def setup_strategy_tab(self, notebook):
#         """Configura a aba de comandos de estratégia."""
#         strategy_frame = ttk.Frame(notebook, padding="10")
#         notebook.add(strategy_frame, text="Strategy Commands")
        
#         # --- Widgets da Aba de Estratégia ---
#         self.status_label = ttk.Label(strategy_frame, text="Service Status: Checking...", foreground="orange")
#         self.status_label.grid(row=0, column=0, columnspan=3, pady=(0, 10))
        
#         ttk.Label(strategy_frame, text="Robot ID:").grid(row=1, column=0, sticky=tk.W, pady=2)
#         self.robot_id_var = tk.IntVar(value=0)
#         ttk.Spinbox(strategy_frame, from_=0, to=15, width=10, textvariable=self.robot_id_var).grid(row=1, column=1, sticky=tk.W, pady=2)
        
#         # Entradas de Posição e Velocidade
#         self.position_x_var = tk.DoubleVar(value=0.0)
#         ttk.Label(strategy_frame, text="Position X (mm):").grid(row=2, column=0, sticky=tk.W, pady=2)
#         ttk.Entry(strategy_frame, textvariable=self.position_x_var, width=15).grid(row=2, column=1, sticky=tk.W, pady=2)
        
#         self.position_y_var = tk.DoubleVar(value=0.0)
#         ttk.Label(strategy_frame, text="Position Y (mm):").grid(row=3, column=0, sticky=tk.W, pady=2)
#         ttk.Entry(strategy_frame, textvariable=self.position_y_var, width=15).grid(row=3, column=1, sticky=tk.W, pady=2)
        
#         self.velocity_x_var = tk.DoubleVar(value=0.0)
#         ttk.Label(strategy_frame, text="Velocity X (mm/s):").grid(row=4, column=0, sticky=tk.W, pady=2)
#         ttk.Entry(strategy_frame, textvariable=self.velocity_x_var, width=15).grid(row=4, column=1, sticky=tk.W, pady=2)
        
#         self.velocity_y_var = tk.DoubleVar(value=0.0)
#         ttk.Label(strategy_frame, text="Velocity Y (mm/s):").grid(row=5, column=0, sticky=tk.W, pady=2)
#         ttk.Entry(strategy_frame, textvariable=self.velocity_y_var, width=15).grid(row=5, column=1, sticky=tk.W, pady=2)
        
#         # Botões de Ação
#         button_frame = ttk.Frame(strategy_frame)
#         button_frame.grid(row=6, column=0, columnspan=3, pady=20)
#         self.send_button = ttk.Button(button_frame, text="Send Command", command=self.send_command, state='disabled')
#         self.send_button.grid(row=0, column=0, padx=5)
#         ttk.Button(button_frame, text="Clear", command=self.clear_fields).grid(row=0, column=1, padx=5)
        
#         # Posições Predefinidas
#         preset_frame = ttk.LabelFrame(strategy_frame, text="Preset Positions", padding="10")
#         preset_frame.grid(row=7, column=0, columnspan=3, pady=10, sticky="ew")
#         ttk.Button(preset_frame, text="Center", command=lambda: self.set_preset(0, 0, 0, 0)).grid(row=0, column=0, padx=2, pady=2, sticky="ew")
#         ttk.Button(preset_frame, text="Our Goal", command=lambda: self.set_preset(-4500, 0, 0, 0)).grid(row=0, column=1, padx=2, pady=2, sticky="ew")
#         ttk.Button(preset_frame, text="Enemy Goal", command=lambda: self.set_preset(4500, 0, 0, 0)).grid(row=0, column=2, padx=2, pady=2, sticky="ew")
#         for i in range(3): preset_frame.columnconfigure(i, weight=1)
        
#         # Caixa de Texto para Respostas
#         response_frame = ttk.LabelFrame(strategy_frame, text="Last Response", padding="5")
#         response_frame.grid(row=8, column=0, columnspan=3, pady=10, sticky="nsew")
#         self.response_text = tk.Text(response_frame, height=8, width=70)
#         self.response_text.grid(row=0, column=0, sticky="nsew")
#         scrollbar = ttk.Scrollbar(response_frame, orient=tk.VERTICAL, command=self.response_text.yview)
#         scrollbar.grid(row=0, column=1, sticky="ns")
#         self.response_text.configure(yscrollcommand=scrollbar.set)
        
#         # Configuração de Redimensionamento da Janela
#         strategy_frame.rowconfigure(8, weight=1)
#         response_frame.columnconfigure(0, weight=1)
#         response_frame.rowconfigure(0, weight=1)

#     def setup_pid_tab(self, notebook):
#         """Configura a aba de sintonia de PID de forma mais flexível."""
#         pid_frame = ttk.Frame(notebook, padding="10")
#         notebook.add(pid_frame, text="PID Tuning")
        
#         row_counter = 0

#         # --- REFACTOR: Layout flexível usando um contador de linha ---
#         self.pid_status_label = ttk.Label(pid_frame, text="PID Service Status: Checking...", foreground="orange")
#         self.pid_status_label.grid(row=row_counter, column=0, columnspan=3, pady=(0, 10)); row_counter += 1
        
#         ttk.Label(pid_frame, text="Robot ID:").grid(row=row_counter, column=0, sticky=tk.W, pady=2)
#         self.pid_robot_id_var = tk.IntVar(value=0)
#         ttk.Spinbox(pid_frame, from_=0, to=15, width=10, textvariable=self.pid_robot_id_var).grid(row=row_counter, column=1, sticky=tk.W, pady=2); row_counter += 1
        
#         self.kp_var = tk.DoubleVar(value=3.0)
#         ttk.Label(pid_frame, text="Kp (Proportional):").grid(row=row_counter, column=0, sticky=tk.W, pady=2)
#         ttk.Entry(pid_frame, textvariable=self.kp_var, width=15).grid(row=row_counter, column=1, sticky=tk.W, pady=2); row_counter += 1
        
#         self.ki_var = tk.DoubleVar(value=0.2)
#         ttk.Label(pid_frame, text="Ki (Integral):").grid(row=row_counter, column=0, sticky=tk.W, pady=2)
#         ttk.Entry(pid_frame, textvariable=self.ki_var, width=15).grid(row=row_counter, column=1, sticky=tk.W, pady=2); row_counter += 1
        
#         self.kd_var = tk.DoubleVar(value=1.0)
#         ttk.Label(pid_frame, text="Kd (Derivative):").grid(row=row_counter, column=0, sticky=tk.W, pady=2)
#         ttk.Entry(pid_frame, textvariable=self.kd_var, width=15).grid(row=row_counter, column=1, sticky=tk.W, pady=2); row_counter += 1
        
#         ttk.Label(pid_frame, text="-"*50).grid(row=row_counter, column=0, columnspan=3, pady=10); row_counter += 1
        
#         self.kp_angular_var = tk.DoubleVar(value=2.5)
#         ttk.Label(pid_frame, text="Kp Angular (Orientação):").grid(row=row_counter, column=0, sticky=tk.W, pady=2)
#         ttk.Entry(pid_frame, textvariable=self.kp_angular_var, width=15).grid(row=row_counter, column=1, sticky=tk.W, pady=2); row_counter += 1
#         ttk.Scale(pid_frame, from_=0, to=10.0, orient=tk.HORIZONTAL, variable=self.kp_angular_var).grid(row=row_counter, column=0, columnspan=2, sticky="ew", pady=(2, 10)); row_counter += 1

#         pid_button_frame = ttk.Frame(pid_frame)
#         pid_button_frame.grid(row=row_counter, column=0, columnspan=3, pady=20); row_counter += 1
#         self.update_pid_button = ttk.Button(pid_button_frame, text="Update PID", command=self.update_pid, state='disabled')
#         self.update_pid_button.grid(row=0, column=0, padx=5)
#         ttk.Button(pid_button_frame, text="Reset to Defaults", command=self.reset_pid_defaults).grid(row=0, column=1, padx=5)
#         self.update_kp_angular_button = ttk.Button(pid_button_frame, text="Update Kp Angular", command=self.update_kp_angular, state='disabled')
#         self.update_kp_angular_button.grid(row=0, column=2, padx=5)

#         pid_response_frame = ttk.LabelFrame(pid_frame, text="PID Update Response", padding="5")
#         pid_response_frame.grid(row=row_counter, column=0, columnspan=3, pady=10, sticky="nsew"); row_counter += 1
#         self.pid_response_text = tk.Text(pid_response_frame, height=8, width=70)
#         self.pid_response_text.grid(row=0, column=0, sticky="nsew")
#         pid_scrollbar = ttk.Scrollbar(pid_response_frame, orient=tk.VERTICAL, command=self.pid_response_text.yview)
#         pid_scrollbar.grid(row=0, column=1, sticky="ns")
#         self.pid_response_text.configure(yscrollcommand=pid_scrollbar.set)
        
#         pid_frame.rowconfigure(row_counter - 1, weight=1)

#     # --- REFACTOR: Função auxiliar para checar serviços e atualizar widgets ---
#     def _update_widget_status(self, client, widget, service_name, status_label=None):
#         """Verifica um serviço e atualiza o estado de um botão e um rótulo de status."""
#         if client.wait_for_service(timeout_sec=0.1):
#             widget.configure(state='normal')
#             if status_label:
#                 status_label.configure(text=f"{service_name}: Available", foreground="green")
#         else:
#             widget.configure(state='disabled')
#             if status_label:
#                 status_label.configure(text=f"{service_name}: Unavailable", foreground="red")

#     def check_service_availability(self):
#         """Verifica todos os serviços usando a função auxiliar."""
#         self._update_widget_status(self.strategy_client, self.send_button, "Strategy Service", self.status_label)
#         self._update_widget_status(self.pid_client, self.update_pid_button, "PID Service", self.pid_status_label)
#         self._update_widget_status(self.kp_angular_client, self.update_kp_angular_button, "Angular Kp Service")

#     # --- Funções de Ação da GUI ---
#     def set_preset(self, x, y, vx, vy):
#         self.position_x_var.set(x)
#         self.position_y_var.set(y)
#         self.velocity_x_var.set(vx)
#         self.velocity_y_var.set(vy)
    
#     def set_pid_preset(self, kp, ki, kd):
#         self.kp_var.set(kp)
#         self.ki_var.set(ki)
#         self.kd_var.set(kd)
    
#     def reset_pid_defaults(self):
#         self.pid_robot_id_var.set(0)
#         self.kp_var.set(3.0)
#         self.ki_var.set(0.2)
#         self.kd_var.set(1.0)
    
#     def clear_fields(self):
#         self.robot_id_var.set(0)
#         self.position_x_var.set(0.0)
#         self.position_y_var.set(0.0)
#         self.velocity_x_var.set(0.0)
#         self.velocity_y_var.set(0.0)
    
#     def send_command(self):
#         """Chama o serviço de comando de estratégia."""
#         try:
#             request = StrategyCommand.Request()
#             request.id = self.robot_id_var.get()
#             request.position_x = float(self.position_x_var.get())
#             request.position_y = float(self.position_y_var.get())
#             request.velocity_x = float(self.velocity_x_var.get())
#             request.velocity_y = float(self.velocity_y_var.get())
#             self._call_service(self.strategy_client, request, self.response_text, "Strategy")
#         except ValueError as e:
#             messagebox.showerror("Input Error", f"Invalid input values: {e}")
    
#     def update_pid(self):
#         """Chama o serviço de atualização de PID."""
#         try:
#             request = ControlParams.Request()
#             request.kp = float(self.kp_var.get())
#             request.ki = float(self.ki_var.get())
#             request.kd = float(self.kd_var.get())
#             self._call_service(self.pid_client, request, self.pid_response_text, "PID")
#         except ValueError as e:
#             messagebox.showerror("Input Error", f"Invalid PID values: {e}")
    
#     def update_kp_angular(self):
#         """Chama o serviço de atualização do Kp angular."""
#         try:
#             request = SetKp.Request()
#             request.kp = float(self.kp_angular_var.get())
#             self._call_service(self.kp_angular_client, request, self.pid_response_text, "Angular Kp")
#         except ValueError as e:
#             messagebox.showerror("Input Error", f"Invalid Angular Kp value: {e}")

#     # --- REFACTOR: Função auxiliar genérica para chamar serviços ---
#     def _call_service(self, client, request, text_widget, service_name):
#         """Chama um serviço de forma assíncrona e lida com a resposta."""
#         if not client.wait_for_service(timeout_sec=1.0):
#             messagebox.showerror("Error", f"{service_name} service is not available")
#             return
        
#         self.get_logger().info(f'Sending {service_name} request...')
#         future = client.call_async(request)
        
#         # Usa uma função anônima (lambda) para passar argumentos extras ao callback
#         future.add_done_callback(
#             lambda future: self._handle_service_response(future, text_widget, service_name)
#         )
#         self._update_text_display(text_widget, f"{service_name} request sent at {time.strftime('%H:%M:%S')}\nWaiting...\n")

#     # --- REFACTOR: Função auxiliar genérica para lidar com respostas ---
#     def _handle_service_response(self, future, text_widget, service_name):
#         """Lida com a resposta de qualquer chamada de serviço."""
#         try:
#             response = future.result()
#             success_text = "SUCCESS" if response.success else "FAILED"
#             self.get_logger().info(f'{service_name} response: {success_text}')
#             response_info = f"{service_name} response at {time.strftime('%H:%M:%S')}: {success_text}\n" + "="*40 + "\n"
#             self._update_text_display(text_widget, response_info)
#         except Exception as e:
#             error_msg = f"{service_name} service call failed: {e}"
#             self.get_logger().error(error_msg)
#             self._update_text_display(text_widget, f"ERROR: {error_msg}\n")
    
#     # --- REFACTOR: Função auxiliar genérica para atualizar caixas de texto ---
#     def _update_text_display(self, text_widget, text):
#         """Atualiza uma caixa de texto da GUI de forma segura a partir de threads."""
#         def update():
#             text_widget.insert(tk.END, text)
#             text_widget.see(tk.END)
#         self.root.after(0, update)
    
#     def spin_ros(self):
#         """Executa o spin do rclpy em uma thread separada."""
#         while rclpy.ok():
#             rclpy.spin_once(self, timeout_sec=0.1)
    
#     def run(self):
#         """Inicia o loop principal da GUI."""
#         try:
#             self.root.mainloop()
#         finally:
#             self.get_logger().info('Shutting down Strategy Command GUI')

# def main(args=None):
#     rclpy.init(args=args)
#     gui_node = None
#     try:
#         gui_node = StrategyCommandGUI()
#         gui_node.run()
#     except KeyboardInterrupt:
#         pass
#     finally:
#         if gui_node:
#             gui_node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()