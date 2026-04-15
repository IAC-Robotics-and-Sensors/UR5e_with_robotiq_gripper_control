import tkinter as tk
from tkinter import ttk, messagebox
from ur_control import URControl, GripperNotActivatedError
import logging

class URControlGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("UR Robot Control")
        self.root.geometry("1200x800")
        self.root.configure(bg="#2b2b2b")

        style = ttk.Style()
        style.theme_use("clam")

        style.configure("TButton",
                        font=("Segoe UI", 12),
                        padding=8,
                        relief="flat",
                        background="#444",
                        foreground="white")
        style.map("TButton",
                  background=[("active", "#E32222"), ("disabled", "#333333")],
                  foreground=[("active", "white"), ("disabled", "#666666")])

        style.configure("TRadiobutton",
                        background="#2b2b2b",
                        foreground="white",
                        font=("Segoe UI", 11))
        
        style.map("TRadiobutton",
                    background=[("active", "#E32222")],
                    foreground=[("active", "white")])

        style.configure("TEntry",
                        fieldbackground="#333",
                        foreground="white")

        # Connect to robot
        self.robot = URControl("192.168.1.3")  # <-- change IP
        self.robot.connect()
        self.robot.connect_gripper()

        self.step_size = 0.01  # default linear step [m]
        self.rot_step = 0.1    # default rotation step [rad]

        # Frame selection variable
        self.frame_var = tk.StringVar(value="world")

        # Press-and-hold tracking
        self.move_timer = None
        self.move_params = None

        # --- Top Control Bar (Step Size + Reconnect) ---
        top_bar = tk.Frame(root, bg="#2b2b2b")
        top_bar.pack(pady=5, fill="x", padx=10)

        # Left side: Step Size Controls
        step_frame = tk.Frame(top_bar, bg="#2b2b2b")
        step_frame.pack(side=tk.LEFT, fill="x", expand=True)

        tk.Label(step_frame, text="Linear step (m):",
                 fg="light blue", bg="#2b2b2b", font=("Segoe UI", 11)).pack(side=tk.LEFT, padx=5)
        self.linear_entry = ttk.Entry(step_frame, width=8)
        self.linear_entry.insert(0, str(self.step_size))
        self.linear_entry.pack(side=tk.LEFT, padx=5)

        tk.Label(step_frame, text="Rotation step (rad):",
                 fg="light blue", bg="#2b2b2b", font=("Segoe UI", 11)).pack(side=tk.LEFT, padx=5)
        self.rot_entry = ttk.Entry(step_frame, width=8)
        self.rot_entry.insert(0, str(self.rot_step))
        self.rot_entry.pack(side=tk.LEFT, padx=5)

        ttk.Button(step_frame, text="Apply",
                   command=self.update_steps).pack(side=tk.LEFT, padx=10)

        # Right side: Reconnect Button
        settings_frame = tk.Frame(top_bar, bg="#2b2b2b")
        settings_frame.pack(side=tk.RIGHT, padx=10)

        ttk.Button(settings_frame, text="Reconnect",
                   command=self.reconnect_robot).pack(side=tk.RIGHT, padx=5)

        # Frame toggle
        toggle_frame = tk.Frame(root, bg="#2b2b2b")
        toggle_frame.pack(pady=5)
        tk.Label(toggle_frame, text="Control Frame:",
                 fg="light blue", bg="#2b2b2b", font=("Segoe UI", 12)).pack(side=tk.LEFT, padx=5)
        ttk.Radiobutton(toggle_frame, text="Base", variable=self.frame_var, value="world").pack(side=tk.LEFT, padx=5)
        ttk.Radiobutton(toggle_frame, text="Tool", variable=self.frame_var, value="tool").pack(side=tk.LEFT, padx=5)

        # --- Container for Translation + Rotation side by side ---
        controls_frame = tk.Frame(root, bg="#2b2b2b")
        controls_frame.pack(pady=10)

        # Translation Controls
        move_frame = tk.LabelFrame(controls_frame, text="Translation",
                                   fg="light blue", bg="#2b2b2b", font=("Segoe UI", 12, "bold"))
        move_frame.pack(side=tk.LEFT, padx=15, pady=5)

        # Y+ button
        btn_yp = ttk.Button(move_frame, text="Y+", width=6)
        btn_yp.grid(row=0, column=1, pady=5)
        btn_yp.bind('<ButtonPress-1>', lambda e: self.start_continuous_move(0, self.step_size, 0, 0, 0, 0))
        btn_yp.bind('<ButtonRelease-1>', lambda e: self.stop_continuous_move())

        # Y- button
        btn_yn = ttk.Button(move_frame, text="Y-", width=6)
        btn_yn.grid(row=2, column=1, pady=5)
        btn_yn.bind('<ButtonPress-1>', lambda e: self.start_continuous_move(0, -self.step_size, 0, 0, 0, 0))
        btn_yn.bind('<ButtonRelease-1>', lambda e: self.stop_continuous_move())

        # X- button
        btn_xn = ttk.Button(move_frame, text="X-", width=6)
        btn_xn.grid(row=1, column=0, padx=5)
        btn_xn.bind('<ButtonPress-1>', lambda e: self.start_continuous_move(-self.step_size, 0, 0, 0, 0, 0))
        btn_xn.bind('<ButtonRelease-1>', lambda e: self.stop_continuous_move())

        # X+ button
        btn_xp = ttk.Button(move_frame, text="X+", width=6)
        btn_xp.grid(row=1, column=2, padx=5)
        btn_xp.bind('<ButtonPress-1>', lambda e: self.start_continuous_move(self.step_size, 0, 0, 0, 0, 0))
        btn_xp.bind('<ButtonRelease-1>', lambda e: self.stop_continuous_move())

        # Z+ button
        btn_zp = ttk.Button(move_frame, text="Z+", width=6)
        btn_zp.grid(row=0, column=3, padx=15)
        btn_zp.bind('<ButtonPress-1>', lambda e: self.start_continuous_move(0, 0, self.step_size, 0, 0, 0))
        btn_zp.bind('<ButtonRelease-1>', lambda e: self.stop_continuous_move())

        # Z- button
        btn_zn = ttk.Button(move_frame, text="Z-", width=6)
        btn_zn.grid(row=2, column=3, padx=15)
        btn_zn.bind('<ButtonPress-1>', lambda e: self.start_continuous_move(0, 0, -self.step_size, 0, 0, 0))
        btn_zn.bind('<ButtonRelease-1>', lambda e: self.stop_continuous_move())

        # Rotation Controls
        rot_frame = tk.LabelFrame(controls_frame, text="Rotation",
                                  fg="light blue", bg="#2b2b2b", font=("Segoe UI", 12, "bold"))
        rot_frame.pack(side=tk.LEFT, padx=15, pady=5)

        # Rx+ button
        btn_rxp = ttk.Button(rot_frame, text="Rx+", width=6)
        btn_rxp.grid(row=0, column=1, pady=5)
        btn_rxp.bind('<ButtonPress-1>', lambda e: self.start_continuous_move(0, 0, 0, self.rot_step, 0, 0))
        btn_rxp.bind('<ButtonRelease-1>', lambda e: self.stop_continuous_move())

        # Rx- button
        btn_rxn = ttk.Button(rot_frame, text="Rx-", width=6)
        btn_rxn.grid(row=2, column=1, pady=5)
        btn_rxn.bind('<ButtonPress-1>', lambda e: self.start_continuous_move(0, 0, 0, -self.rot_step, 0, 0))
        btn_rxn.bind('<ButtonRelease-1>', lambda e: self.stop_continuous_move())

        # Ry- button
        btn_ryn = ttk.Button(rot_frame, text="Ry-", width=6)
        btn_ryn.grid(row=1, column=0, padx=5)
        btn_ryn.bind('<ButtonPress-1>', lambda e: self.start_continuous_move(0, 0, 0, 0, -self.rot_step, 0))
        btn_ryn.bind('<ButtonRelease-1>', lambda e: self.stop_continuous_move())

        # Ry+ button
        btn_ryp = ttk.Button(rot_frame, text="Ry+", width=6)
        btn_ryp.grid(row=1, column=2, padx=5)
        btn_ryp.bind('<ButtonPress-1>', lambda e: self.start_continuous_move(0, 0, 0, 0, self.rot_step, 0))
        btn_ryp.bind('<ButtonRelease-1>', lambda e: self.stop_continuous_move())

        # Rz+ button
        btn_rzp = ttk.Button(rot_frame, text="Rz+", width=6)
        btn_rzp.grid(row=0, column=3, padx=15)
        btn_rzp.bind('<ButtonPress-1>', lambda e: self.start_continuous_move(0, 0, 0, 0, 0, self.rot_step))
        btn_rzp.bind('<ButtonRelease-1>', lambda e: self.stop_continuous_move())

        # Rz- button
        btn_rzn = ttk.Button(rot_frame, text="Rz-", width=6)
        btn_rzn.grid(row=2, column=3, padx=15)
        btn_rzn.bind('<ButtonPress-1>', lambda e: self.start_continuous_move(0, 0, 0, 0, 0, -self.rot_step))
        btn_rzn.bind('<ButtonRelease-1>', lambda e: self.stop_continuous_move())

        # Joint Controls
        joint_frame = tk.LabelFrame(controls_frame, text="Joint Control",
                                    fg="light blue", bg="#2b2b2b", font=("Segoe UI", 12, "bold"))
        joint_frame.pack(side=tk.LEFT, padx=15, pady=5)

        # Store joint position labels for updating
        self.joint_labels = []

        # Names of joints
        self.joint_names = ["Base", "Shoulder", "Elbow", "Wrist 1", "Wrist 2", "Wrist 3"]

        # Create a row for each joint with Left button, position display, Right button
        for i in range(6):
            # Left button (-)
            btn_left = ttk.Button(joint_frame, text="◄", width=3)
            btn_left.grid(row=i, column=0, padx=2, pady=2)
            btn_left.bind('<ButtonPress-1>', lambda e, idx=i: self.start_continuous_joint_move(idx, -self.rot_step))
            btn_left.bind('<ButtonRelease-1>', lambda e: self.stop_continuous_move())
            
            # Joint position label
            joint_label = tk.Label(joint_frame, text=f"{self.joint_names[i]}: 0.000", 
                                   fg="white", bg="#2b2b2b", 
                                   font=("Segoe UI", 10), width=15)
            joint_label.grid(row=i, column=1, padx=5, pady=2)
            self.joint_labels.append(joint_label)
            
            # Right button (+)
            btn_right = ttk.Button(joint_frame, text="►", width=3)
            btn_right.grid(row=i, column=2, padx=2, pady=2)
            btn_right.bind('<ButtonPress-1>', lambda e, idx=i: self.start_continuous_joint_move(idx, self.rot_step))
            btn_right.bind('<ButtonRelease-1>', lambda e: self.stop_continuous_move())

        # Start updating joint positions
        self.update_joint_positions()

        # --- Control Panel ---
        control_panel = tk.Frame(root, bg="#2b2b2b")
        control_panel.pack(pady=20, fill="both", expand=True)

        # --- ROBOT STATUS & BASIC MOVES ---
        home_frame = tk.LabelFrame(control_panel, text="Robot Status & Basic Moves",
                                fg="light blue", bg="#2b2b2b", font=("Segoe UI", 12, "bold"),
                                padx=10, pady=10, borderwidth=2, relief="groove")
        home_frame.grid(row=0, column=0, padx=15, pady=15, sticky="nsew")

        ttk.Button(home_frame, text="Go Home", width=15,
                command=self.robot.move_home).grid(row=0, column=0, padx=10, pady=8)

        ttk.Button(home_frame, text="Get Joint Pos", width=15,
                command=self.show_joint_pos_popup).grid(row=0, column=1, padx=10, pady=8)

        ttk.Button(home_frame, text="Get TCP Pos", width=15,
                command=self.show_tcp_pos_popup).grid(row=1, column=0, padx=10, pady=8)

        # --- WAYPOINTS ---
        waypoint_frame = tk.LabelFrame(control_panel, text="Waypoints",
                                    fg="light blue", bg="#2b2b2b", font=("Segoe UI", 12, "bold"),
                                    padx=10, pady=10, borderwidth=2, relief="groove")
        waypoint_frame.grid(row=0, column=1, padx=15, pady=15, sticky="nsew")

        ttk.Button(waypoint_frame, text="Waypoint 1", width=15,
                command=lambda: self.robot.move_to_waypoint("waypoint_1")).grid(row=0, column=0, padx=8, pady=8)

        ttk.Button(waypoint_frame, text="Waypoint 2", width=15,
                command=lambda: self.robot.move_to_waypoint("waypoint_2")).grid(row=0, column=1, padx=8, pady=8)

        ttk.Button(waypoint_frame, text="Waypoint 3", width=15,
                command=lambda: self.robot.move_to_waypoint("waypoint_3")).grid(row=1, column=0, padx=8, pady=8)

        # --- GRIPPER CONTROL ---
        gripper_frame = tk.LabelFrame(control_panel, text="Gripper Control",
                                    fg="light blue", bg="#2b2b2b", font=("Segoe UI", 12, "bold"),
                                    padx=10, pady=10, borderwidth=2, relief="groove")
        gripper_frame.grid(row=0, column=2, padx=15, pady=15, sticky="nsew")

        ttk.Button(gripper_frame, text="Activate Gripper", width=15,
                command=self.robot.gripper_activate).grid(row=0, column=0, padx=8, pady=8)

        ttk.Button(gripper_frame, text="Open Gripper", width=15,
                command=self.gripper_open_safe).grid(row=0, column=1, padx=8, pady=8)

        ttk.Button(gripper_frame, text="Close Gripper", width=15,
                command=self.gripper_close_safe).grid(row=1, column=0, padx=8, pady=8)

        # Ensure even stretch
        control_panel.grid_rowconfigure(0, weight=1)
        control_panel.grid_columnconfigure(0, weight=1)
        control_panel.grid_columnconfigure(1, weight=1)
        control_panel.grid_columnconfigure(2, weight=1)

    def update_steps(self):
        """Update linear and rotation step sizes from user input."""
        try:
            self.step_size = float(self.linear_entry.get())
            self.rot_step = float(self.rot_entry.get())
            logging.info(f"Updated steps: linear={self.step_size}, rotation={self.rot_step}")
        except ValueError:
            logging.error("Invalid input for step sizes")

    def move(self, x, y, z, rx, ry, rz):
        frame = self.frame_var.get()
        if frame == "world":
            self.robot.relative_world_move([x, y, z, rx, ry, rz])
        else:
            self.robot.relative_tool_move([x, y, z, rx, ry, rz])

    def start_continuous_move(self, x, y, z, rx, ry, rz):
        """Start continuous movement on button press"""
        self.move_params = (x, y, z, rx, ry, rz)
        self._execute_continuous_move()

    def _execute_continuous_move(self):
        """Execute movement and schedule next movement"""
        if self.move_params is not None:
            if isinstance(self.move_params, tuple) and self.move_params[0] == 'joint':
                # This is handled by _execute_continuous_joint_move
                return
            else:
                x, y, z, rx, ry, rz = self.move_params
                self.move(x, y, z, rx, ry, rz)
                # Schedule next movement (50ms interval for smooth continuous motion)
                self.move_timer = self.root.after(50, self._execute_continuous_move)

    def stop_continuous_move(self):
        """Stop continuous movement on button release"""
        if self.move_timer is not None:
            self.root.after_cancel(self.move_timer)
            self.move_timer = None
        self.move_params = None

    def start_continuous_joint_move(self, joint_index, step):
        """Start continuous joint movement on button press"""
        # Create a move array with zeros except for the specified joint
        joint_move = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        joint_move[joint_index] = step
        self.move_params = ('joint', joint_move)
        self._execute_continuous_joint_move()

    def _execute_continuous_joint_move(self):
        """Execute joint movement and schedule next movement"""
        if self.move_params is not None and self.move_params[0] == 'joint':
            joint_move = self.move_params[1]
            self.robot.move_add_j(joint_move)
            # Schedule next movement (50ms interval for smooth continuous motion)
            self.move_timer = self.root.after(50, self._execute_continuous_joint_move)

    def update_joint_positions(self):
        """Update joint position labels periodically"""
        try:
            joint_pos = self.robot.get_joint_pos()
            if joint_pos is not None:
                for i, label in enumerate(self.joint_labels):
                    # Convert radians to degrees for easier reading
                    degrees = joint_pos[i] * 180.0 / 3.14159
                    label.config(text=f"{self.joint_names[i]}: {degrees:6.1f}°")
        except Exception as e:
            # If robot not connected, just skip update
            pass
        
        # Schedule next update (every 100ms)
        self.root.after(100, self.update_joint_positions)

    def gripper_open_safe(self):
        """Safe wrapper for gripper open with error handling."""
        try:
            self.robot.gripper_open()
        except GripperNotActivatedError as e:
            messagebox.showwarning(
                "Gripper Not Activated",
                str(e),
                parent=self.root
            )
        except Exception as e:
            messagebox.showerror(
                "Gripper Error",
                f"Failed to open gripper: {str(e)}",
                parent=self.root
            )

    def gripper_close_safe(self):
        """Safe wrapper for gripper close with error handling."""
        try:
            self.robot.gripper_close()
        except GripperNotActivatedError as e:
            messagebox.showwarning(
                "Gripper Not Activated",
                str(e),
                parent=self.root
            )
        except Exception as e:
            messagebox.showerror(
                "Gripper Error",
                f"Failed to close gripper: {str(e)}",
                parent=self.root
            )

    def show_tcp_pos_popup(self):
        """Display TCP position in a popup window as copyable text."""
        try:
            tcp_pos = self.robot.get_tcp_pos()
            if tcp_pos is None:
                messagebox.showerror(
                    "Error",
                    "Could not retrieve TCP position",
                    parent=self.root
                )
                return
            
            # Also log to terminal
            logging.info(f"TCP position: {tcp_pos}")
            
            # Create popup window
            popup = tk.Toplevel(self.root)
            popup.title("TCP Position")
            popup.geometry("450x300")
            popup.configure(bg="#2b2b2b")
            
            # Title label
            title_label = tk.Label(
                popup,
                text="Current TCP Position",
                fg="light blue",
                bg="#2b2b2b",
                font=("Segoe UI", 12, "bold")
            )
            title_label.pack(pady=10)
            
            # Text widget with TCP position
            text_frame = tk.Frame(popup, bg="#2b2b2b")
            text_frame.pack(pady=5, padx=10, fill="both", expand=True)
            
            text_widget = tk.Text(
                text_frame,
                height=10,
                width=50,
                bg="#333",
                fg="white",
                font=("Courier", 10),
                relief="solid",
                borderwidth=1
            )
            text_widget.pack(fill="both", expand=True)
            
            # Format and insert TCP position
            axis_names = ["X (Position)", "Y (Position)", "Z (Position)",
                         "Rx (Rotation)", "Ry (Rotation)", "Rz (Rotation)"]
            
            text_content = "TCP Position and Orientation:\n\n"
            for i, (name, value) in enumerate(zip(axis_names, tcp_pos)):
                if i < 3:  # Position values in meters
                    text_content += f"{name:20s}: {value:10.6f} m\n"
                else:  # Rotation values in radians
                    degrees = value * 180.0 / 3.14159
                    text_content += f"{name:20s}: {value:10.6f} rad ({degrees:7.2f}°)\n"
            
            text_widget.insert("1.0", text_content)
            text_widget.config(state="disabled")  # Make read-only
            
            # Button frame
            button_frame = tk.Frame(popup, bg="#2b2b2b")
            button_frame.pack(pady=10)
            
            # Copy button
            def copy_to_clipboard():
                popup.clipboard_clear()
                popup.clipboard_append(text_widget.get("1.0", tk.END))
                messagebox.showinfo(
                    "Copied",
                    "TCP position copied to clipboard!",
                    parent=popup
                )
            
            ttk.Button(
                button_frame,
                text="Copy to Clipboard",
                command=copy_to_clipboard
            ).pack(side=tk.LEFT, padx=5)
            
            ttk.Button(
                button_frame,
                text="Close",
                command=popup.destroy
            ).pack(side=tk.LEFT, padx=5)
            
        except Exception as e:
            logging.error(f"Error displaying TCP position: {e}")
            messagebox.showerror(
                "Error",
                f"Failed to retrieve TCP position: {str(e)}",
                parent=self.root
            )

    def show_joint_pos_popup(self):
        """Display joint positions in a popup window as copyable text."""
        try:
            joint_pos = self.robot.get_joint_pos()
            if joint_pos is None:
                messagebox.showerror(
                    "Error",
                    "Could not retrieve joint positions",
                    parent=self.root
                )
                return
            
            # Also log to terminal
            logging.info(f"Joint positions: {joint_pos}")
            
            # Create popup window
            popup = tk.Toplevel(self.root)
            popup.title("Joint Positions")
            popup.geometry("400x350")
            popup.configure(bg="#2b2b2b")
            
            # Title label
            title_label = tk.Label(
                popup,
                text="Current Joint Positions (radians)",
                fg="light blue",
                bg="#2b2b2b",
                font=("Segoe UI", 12, "bold")
            )
            title_label.pack(pady=10)
            
            # Text widget with joint positions
            text_frame = tk.Frame(popup, bg="#2b2b2b")
            text_frame.pack(pady=5, padx=10, fill="both", expand=True)
            
            text_widget = tk.Text(
                text_frame,
                height=12,
                width=45,
                bg="#333",
                fg="white",
                font=("Courier", 10),
                relief="solid",
                borderwidth=1
            )
            text_widget.pack(fill="both", expand=True)
            
            # Format and insert joint positions
            joint_names = ["Base (J1)", "Shoulder (J2)", "Elbow (J3)", 
                          "Wrist 1 (J4)", "Wrist 2 (J5)", "Wrist 3 (J6)"]
            
            text_content = "Joint Positions:\n\n"
            for i, (name, value) in enumerate(zip(joint_names, joint_pos)):
                degrees = value * 180.0 / 3.14159
                text_content += f"{name:20s}: {value:10.6f} rad ({degrees:7.2f}°)\n"
            
            text_widget.insert("1.0", text_content)
            text_widget.config(state="disabled")  # Make read-only
            
            # Button frame
            button_frame = tk.Frame(popup, bg="#2b2b2b")
            button_frame.pack(pady=10)
            
            # Copy button
            def copy_to_clipboard():
                popup.clipboard_clear()
                popup.clipboard_append(text_widget.get("1.0", tk.END))
                messagebox.showinfo(
                    "Copied",
                    "Joint positions copied to clipboard!",
                    parent=popup
                )
            
            ttk.Button(
                button_frame,
                text="Copy to Clipboard",
                command=copy_to_clipboard
            ).pack(side=tk.LEFT, padx=5)
            
            ttk.Button(
                button_frame,
                text="Close",
                command=popup.destroy
            ).pack(side=tk.LEFT, padx=5)
            
        except Exception as e:
            logging.error(f"Error displaying joint positions: {e}")
            messagebox.showerror(
                "Error",
                f"Failed to retrieve joint positions: {str(e)}",
                parent=self.root
            )

    def reconnect_robot(self):
        """Manually reconnect to the robot."""
        try:
            logging.info("Manual reconnection requested...")
            
            # Reconnect RTDE interfaces
            try:
                self.robot.connect()
                logging.info("RTDE reconnected successfully")
            except Exception as e:
                logging.error(f"RTDE reconnection failed: {e}")
                messagebox.showerror(
                    "Reconnection Failed",
                    f"Failed to reconnect RTDE interfaces:\n{str(e)}",
                    parent=self.root
                )
                return
            
            # Reconnect gripper
            try:
                self.robot.connect_gripper()
                logging.info("Gripper reconnected successfully")
                messagebox.showinfo(
                    "Reconnection",
                    "Robot RTDE and gripper reconnected successfully!",
                    parent=self.root
                )
            except Exception as e:
                logging.error(f"Gripper reconnection failed: {e}")
                messagebox.showwarning(
                    "Partial Reconnection",
                    f"RTDE reconnected but gripper failed:\n{str(e)}",
                    parent=self.root
                )
        except Exception as e:
            logging.error(f"Reconnection error: {e}")
            messagebox.showerror(
                "Reconnection Error",
                f"An unexpected error occurred:\n{str(e)}",
                parent=self.root
            )


def main():
    logging.basicConfig(level=logging.INFO)
    root = tk.Tk()
    app = URControlGUI(root)
    root.mainloop()

if __name__ == "__main__":
    main()
