import tkinter as tk
from tkinter import messagebox, filedialog
import serial
import threading
import time
import numpy as np
import math 
from queue import Queue, Empty
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import csv
import os
import matplotlib.pyplot as plt
from threading import Lock

# Joint limits (in degrees)
v1_min, v1_max = 0, 180
v2_min, v2_max = -360, 360

# Set up the serial port (adjust 'COM3' and baud rate to your settings)
ser = serial.Serial('COM3', 115200, timeout=1)
time.sleep(2)  # Wait for Arduino to initialize

def minimal_angle_difference(a, b):
    """
    Computes the minimal difference between two angles in degrees
    """
    diff = a - b
    return (diff + 180) % 360 - 180

def forward_kinematics(theta1, theta2, r1, r2):
    """
    Used to calculate the position (x,y) of the end-effector

    Parameters:
    - theta1: Angle of the first joint in radians
    - theta2: Angle of the second joint in radians
    - r1: Length of the first arm segment
    - r2: Length of the second arm segment

    Returns:
    - A array containing the x and y coordinates
    """
    x = r1 * np.cos(theta1) + r2 * np.cos(theta1 + theta2)
    y = r1 * np.sin(theta1) + r2 * np.sin(theta1 + theta2)
    return np.array([x, y])

def jacobian(theta1, theta2, r1, r2):
    """
    Computes the Jacobian matrix 

    Parameters:
    - theta1: Angle of the first joint in radians
    - theta2: Angle of the second joint in radians
    - r1: Length of the first arm segment
    - r2: Length of the second arm segment

    Returns:
    - Jacobian matrix (2x2 array) 
    """
    J = np.array([
        [-r1 * np.sin(theta1) - r2 * np.sin(theta1 + theta2), -r2 * np.sin(theta1 + theta2)],
        [ r1 * np.cos(theta1) + r2 * np.cos(theta1 + theta2),  r2 * np.cos(theta1 + theta2)]
    ])
    return J

def ik_differential(x_target, y_target, r1, r2, theta1_init_deg, theta2_init_deg, max_iterations=100, tolerance=1e-3):
    """
    Performing Differential Inverse Kinematics

    Parameters:
    - x_target, y_target: Desired end-effector position
    - r1, r2: Lengths of the arm segments
    - theta1_init_deg, theta2_init_deg: Initial guesses for joint angles in degrees
    - max_iterations: Maximum number of iterations
    - tolerance: Convergence criterion

    Returns:
    - Joint angles (theta1_deg, theta2_deg) in degrees, or None if not converged
    """
    theta1 = np.radians(theta1_init_deg)
    theta2 = np.radians(theta2_init_deg)

    for i in range(max_iterations):
        # Compute current end-effector position
        position_current = forward_kinematics(theta1, theta2, r1, r2)
        position_target = np.array([x_target, y_target])

        # Compute position error with desired position
        error = position_target - position_current
        error_norm = np.linalg.norm(error)

        if error_norm < tolerance:
            # Converged, return angles in degrees
            return np.degrees(theta1), np.degrees(theta2)
        
        # Compute Jacobian
        J = jacobian(theta1, theta2, r1, r2)

        # Compute delta_theta using pseudoinverse of Jacobian
        delta_theta = np.linalg.pinv(J).dot(error)

        # Update theta values
        theta1 += delta_theta[0]
        theta2 += delta_theta[1]

        # Normalize angles to range [-pi, pi]
        theta1 = np.arctan2(np.sin(theta1), np.cos(theta1))
        theta2 = np.arctan2(np.sin(theta2), np.cos(theta2))

    return None  # Return None if convergence could not be achieved within max_iterations

def ik_differential_ANA(x_target, y_target, r1, r2, theta1_init_deg, theta2_init_deg, max_iterations=2000, tolerance=1e-3):

    """
    Performing analytical Differential Inverse Kinematics

    Parameters:
    - x_target, y_target: Desired end-effector position
    - r1, r2: Lengths of the arm segments

    Returns:
    - Joint angles (theta1_deg, theta2_deg) in degrees

    """
    import math 
    P0x = x_target
    P0y = y_target 
    
    t2 = math.degrees(math.acos((P0x**2 + P0y**2 - r1**2 - r2**2) / (2 * r1 * r2)))
    t1 = math.degrees(math.atan2(P0y, P0x)) - math.degrees(math.atan2((r2 * math.sin(math.radians(t2))), (r1 + r2 * math.cos(math.radians(t2)))))
                                                           

    return t1, t2  # Return None if convergence could not be acheievd within max_iterations

def send_to_arduino(v1, v2, ser):
    """
    Sends joint angles to the Arduino via serial communication.

    Parameters:
    - v1, v2: Joint angles in degrees
    """
    message = f"{v1:.2f},{v2:.2f}\n"
    ser.write(message.encode('utf-8'))
    print(f"Sending to Arduino: {message.strip()}")

class MotorControlGUI:
    """
    GUI class used to control the robot
    """

    def __init__(self, root):
        ####################################################################

        # Set initial angles for the robotic arm
        self.v1_current = 0.0  # Initial angle for the first arm segment
        self.v2_current = 0.0   # Initial angle for the second arm segment

        # Arm segment lengths
        self.r1 = 120.25  # Length of the first arm segment
        self.r2 = 93.0   # Length of the second arm (87.1 mm) segment plus radius of the pen we are using (4.9 mm)

        # Initialize target angles
        self.v1_target = self.v1_current
        self.v2_target = self.v2_current

        ####################################################################

        self.root = root
        self.root.title("Motor Control GUI")

        # Set the minimum size for the GUI window
        self.root.minsize(800, 800)  # Increased size for better plot and new controls
        self.root.geometry("1000x1200")

        # Main container frames
        self.main_frame = tk.Frame(root, padx=10, pady=10)
        self.main_frame.pack(fill=tk.BOTH, expand=True)

        # Frame for Input Controls
        self.input_frame = tk.LabelFrame(self.main_frame, text="Position Control", padx=10, pady=10, font=("Arial", 14))
        self.input_frame.pack(fill=tk.X, pady=5)

        # Input fields for desired position
        tk.Label(self.input_frame, text="X Position:", font=("Arial", 12)).grid(row=0, column=0, padx=5, pady=5, sticky='e')
        self.position_input_x = tk.Entry(self.input_frame, font=("Arial", 12), width=15)
        self.position_input_x.grid(row=0, column=1, padx=5, pady=5)

        tk.Label(self.input_frame, text="Y Position:", font=("Arial", 12)).grid(row=1, column=0, padx=5, pady=5, sticky='e')
        self.position_input_y = tk.Entry(self.input_frame, font=("Arial", 12), width=15)
        self.position_input_y.grid(row=1, column=1, padx=5, pady=5)

        # Checkbox to enable/disable saving and plotting
        self.save_var = tk.BooleanVar(value=True)  # Default to saving enabled
        self.save_checkbox = tk.Checkbutton(
            self.input_frame,
            text="Save Data and Plots",
            variable=self.save_var,
            font=("Arial", 12)
        )
        self.save_checkbox.grid(row=2, column=0, columnspan=2, pady=10)

        # Buttons
        self.button_frame = tk.Frame(self.input_frame)
        self.button_frame.grid(row=3, column=0, columnspan=2, pady=5)

        self.send_button = tk.Button(self.button_frame, text="Set Position", command=self.send_position, font=("Arial", 12), width=15)
        self.send_button.pack(side=tk.LEFT, padx=5)

        self.home_button = tk.Button(self.button_frame, text="Home", command=self.send_home_position, font=("Arial", 12), bg="lightgreen", width=15)
        self.home_button.pack(side=tk.LEFT, padx=5)

        # Label for current position
        self.current_position_label = tk.Label(self.input_frame, text="Current Position: v1=0.00°, v2=0.00°", font=("Arial", 12))
        self.current_position_label.grid(row=4, column=0, columnspan=2, pady=5)

        # Save Status Label
        self.save_status_label = tk.Label(self.input_frame, text="", font=("Arial", 12), fg="green")
        self.save_status_label.grid(row=5, column=0, columnspan=2, pady=5)

        # Frame for PID Controls
        self.pid_frame = tk.LabelFrame(self.main_frame, text="PID Controller Settings", padx=10, pady=10, font=("Arial", 14))
        self.pid_frame.pack(fill=tk.X, pady=5)

        # PID Inputs for Motor 1
        tk.Label(self.pid_frame, text="Motor 1 Kp:", font=("Arial", 12)).grid(row=0, column=0, padx=5, pady=5, sticky='e')
        self.Kp_m1_entry = tk.Entry(self.pid_frame, font=("Arial", 12), width=10)
        self.Kp_m1_entry.grid(row=0, column=1, padx=5, pady=5)
        self.Kp_m1_entry.insert(0, "4.0")  # Default value

        tk.Label(self.pid_frame, text="Motor 1 Ki:", font=("Arial", 12)).grid(row=1, column=0, padx=5, pady=5, sticky='e')
        self.Ki_m1_entry = tk.Entry(self.pid_frame, font=("Arial", 12), width=10)
        self.Ki_m1_entry.grid(row=1, column=1, padx=5, pady=5)
        self.Ki_m1_entry.insert(0, "0.1")  # Default value

        tk.Label(self.pid_frame, text="Motor 1 Kd:", font=("Arial", 12)).grid(row=2, column=0, padx=5, pady=5, sticky='e')
        self.Kd_m1_entry = tk.Entry(self.pid_frame, font=("Arial", 12), width=10)
        self.Kd_m1_entry.grid(row=2, column=1, padx=5, pady=5)
        self.Kd_m1_entry.insert(0, "0.3")  # Default value

        # PID Inputs for Motor 2
        tk.Label(self.pid_frame, text="Motor 2 Kp:", font=("Arial", 12)).grid(row=0, column=2, padx=5, pady=5, sticky='e')
        self.Kp_m2_entry = tk.Entry(self.pid_frame, font=("Arial", 12), width=10)
        self.Kp_m2_entry.grid(row=0, column=3, padx=5, pady=5)
        self.Kp_m2_entry.insert(0, "7.0")  # Default value

        tk.Label(self.pid_frame, text="Motor 2 Ki:", font=("Arial", 12)).grid(row=1, column=2, padx=5, pady=5, sticky='e')
        self.Ki_m2_entry = tk.Entry(self.pid_frame, font=("Arial", 12), width=10)
        self.Ki_m2_entry.grid(row=1, column=3, padx=5, pady=5)
        self.Ki_m2_entry.insert(0, "0.2")  # Default value

        tk.Label(self.pid_frame, text="Motor 2 Kd:", font=("Arial", 12)).grid(row=2, column=2, padx=5, pady=5, sticky='e')
        self.Kd_m2_entry = tk.Entry(self.pid_frame, font=("Arial", 12), width=10)
        self.Kd_m2_entry.grid(row=2, column=3, padx=5, pady=5)
        self.Kd_m2_entry.insert(0, "0.2")  # Default value

        # Button to send PID gains
        self.send_pid_button = tk.Button(self.pid_frame, text="Update PID Gains", command=self.send_pid, font=("Arial", 12), bg="lightblue", width=20)
        self.send_pid_button.grid(row=3, column=0, columnspan=4, pady=10)

        # Auotmatic PID button
        self.auto_tune_both_button = tk.Button(self.pid_frame,text="Auto Tune Both Motors PID",command=self.automatic_pid_tuning_both,font=("Arial", 12),bg="orange",width=20)
        self.auto_tune_both_button.grid(row=4, column=0, columnspan=4, pady=10)

        # Frame for Path Following Controls
        self.path_frame = tk.LabelFrame(self.main_frame, text="Path Following", padx=10, pady=10, font=("Arial", 14))
        self.path_frame.pack(fill=tk.X, pady=5)

        # Checkbox to enable/disable path following from CSV
        self.path_var = tk.BooleanVar(value=False)
        self.path_checkbox = tk.Checkbutton(
            self.path_frame,
            text="Follow Path from CSV",
            variable=self.path_var,
            font=("Arial", 12),
            command=self.toggle_path_following
        )
        self.path_checkbox.grid(row=0, column=0, columnspan=2, pady=5, sticky='w')

        # Button to load CSV file
        self.load_csv_button = tk.Button(self.path_frame, text="Load Path CSV", command=self.load_csv, font=("Arial", 12), width=15, state='disabled')
        self.load_csv_button.grid(row=1, column=0, padx=5, pady=5)

        # Buttons for Manual Navigation
        self.navigation_frame = tk.Frame(self.path_frame)
        self.navigation_frame.grid(row=1, column=1, padx=5, pady=5)

        self.prev_button = tk.Button(self.navigation_frame, text="Previous Point", command=self.previous_point, font=("Arial", 12), width=15, state='disabled')
        self.prev_button.pack(side=tk.LEFT, padx=2)

        self.next_button = tk.Button(self.navigation_frame, text="Next Point", command=self.next_point, font=("Arial", 12), width=15, state='disabled')
        self.next_button.pack(side=tk.LEFT, padx=2)

        # Label to show current point
        self.current_point_label = tk.Label(self.path_frame, text="Current Point: N/A", font=("Arial", 12))
        self.current_point_label.grid(row=2, column=0, columnspan=2, pady=5)

        # Canvas for matplotlib plot
        self.fig = Figure(figsize=(8, 8))  # Increased figure size
        self.ax = self.fig.add_subplot(111)
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.main_frame)  # A tk.DrawingArea.
        self.canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True, pady=10)

        # Initialize plot lines
        self.current_arm_line, = self.ax.plot([], [], 'ro-', linewidth=4, label='Current Position')
        self.target_arm_line, = self.ax.plot([], [], 'bo--', linewidth=2, label='Target Position')
        self.path_points_plot, = self.ax.plot([], [], 'gx', markersize=8, label='Path Points')  # Plot for path points

        # Initialize angle annotations
        self.angle1_text = self.ax.text(0, 0, "", fontsize=12, color='black', ha='right', va='bottom')
        self.angle2_text = self.ax.text(0, 0, "", fontsize=12, color='black', ha='right', va='bottom')

        # Set plot limits
        total_length = self.r1 + self.r2 + 50  # Added margin
        self.ax.set_xlim(-total_length, total_length)
        self.ax.set_ylim(-total_length, total_length)
        self.ax.set_aspect('equal')

        # Add legend
        self.ax.legend()

        # Draw the canvas initially
        self.canvas.draw()

        # Queue for thread communication
        self.position_queue = Queue()

        # Start a thread to read current position from serial port
        self.running = True
        self.update_thread = threading.Thread(target=self.update_current_position, daemon=True)
        self.update_thread.start()

        # Data recording variables
        self.time_data = []
        self.v1_data = []
        self.v2_data = []
        self.error_v1_data = []  # Error data for V1
        self.error_v2_data = []  # Error data for V2
        self.recording = False
        self.start_time = None
        self.condition_met_time = None
        self.condition_duration = 1  # seconds
        self.degrees_threshold = 1  # degrees
        self.lock = threading.Lock()
        self.current_combination_count = None
        


        # Variables to store the last input X and Y for filename encoding
        self.last_input_x = None
        self.last_input_y = None

        # Directory to save plots and data
        self.save_dir = "motor_data"
        os.makedirs(self.save_dir, exist_ok=True)

        # Initialize sequence number
        self.sequence_number = self.get_initial_sequence_number()

        # Variables to track best settling times
        self.best_settling_time_v1 = None
        self.best_settling_time_v2 = None
        self.best_settling_seq_v1 = None
        self.best_settling_seq_v2 = None
        self.best_settling_pid_v1 = None
        self.best_settling_pid_v2 = None

        # Path Following Variables
        self.path_points = []          # List of (x, y) tuples
        self.current_path_index = 0    # Index of the current target point
        self.following_path = False    # Flag to indicate if path following is active

        # Update GUI as frequently as possible
        self.update_gui()

    def get_initial_sequence_number(self):
        """
        Reads the last sequence number from counter.txt.
        If counter.txt doesn't exist, starts from 1.
        """
        counter_file = os.path.join(self.save_dir, 'counter.txt')
        if os.path.exists(counter_file):
            try:
                with open(counter_file, 'r') as f:
                    num = int(f.read().strip())
                    return num + 1
            except (ValueError, IOError) as e:
                print(f"Error reading counter.txt: {e}")
                return 1
        else:
            return 1

    def update_sequence_number(self):
        """
        Updates the counter.txt with the current sequence number.
        """
        counter_file = os.path.join(self.save_dir, 'counter.txt')
        try:
            with open(counter_file, 'w') as f:
                f.write(str(self.sequence_number))
        except IOError as e:
            print(f"Error writing to counter.txt: {e}")

    def plot_arm(self, theta1_deg, theta2_deg, target_theta1_deg=None, target_theta2_deg=None):
        """
        Updates the arm's current and target positions on the plot and annotates the angles.

        Parameters:
        - theta1_deg: Current angle of the first joint in degrees
        - theta2_deg: Current angle of the second joint in degrees
        - target_theta1_deg: Target angle of the first joint in degrees
        - target_theta2_deg: Target angle of the second joint in degrees
        """
        # Convert angles to radians
        theta1 = np.radians(theta1_deg)
        theta2 = np.radians(theta2_deg)

        # Compute joint positions for current angles
        x0, y0 = 0, 0  # Base position
        x1 = self.r1 * np.cos(theta1)
        y1 = self.r1 * np.sin(theta1)
        x2 = x1 + self.r2 * np.cos(theta1 + theta2)
        y2 = y1 + self.r2 * np.sin(theta1 + theta2)

        # Update current arm line data
        self.current_arm_line.set_data([x0, x1, x2], [y0, y1, y2])

        # Update target arm line data if target angles are provided
        if target_theta1_deg is not None and target_theta2_deg is not None:
            target_theta1 = np.radians(target_theta1_deg)
            target_theta2 = np.radians(target_theta2_deg)

            x1_t = self.r1 * np.cos(target_theta1)
            y1_t = self.r1 * np.sin(target_theta1)
            x2_t = x1_t + self.r2 * np.cos(target_theta1 + target_theta2)
            y2_t = y1_t + self.r2 * np.sin(target_theta1 + target_theta2)

            self.target_arm_line.set_data([x0, x1_t, x2_t], [y0, y1_t, y2_t])
        else:
            self.target_arm_line.set_data([], [])

        # Calculate midpoints for angle annotations
        # θ1 is between base (x0, y0) and joint1 (x1, y1)
        mid_x1 = (x0 + x1) / 2
        mid_y1 = (y0 + y1) / 2

        # θ2 is between joint1 (x1, y1) and end-effector (x2, y2)
        mid_x2 = (x1 + x2) / 2
        mid_y2 = (y1 + y2) / 2

        # Update angle annotations
        self.angle1_text.set_position((mid_x1, mid_y1))
        self.angle1_text.set_text(f"θ1={theta1_deg:.2f}°")

        self.angle2_text.set_position((mid_x2, mid_y2))
        self.angle2_text.set_text(f"θ2={theta2_deg:.2f}°")

        # Redraw the canvas
        self.canvas.draw()

    def send_position(self):
        """
        1) Get the desired position from the input fields
        2) Calculate inverse kinematics
        3) Send the joint angles to the Arduino
        4) Update the target position on the plot
        5) Initialize data recording for plotting performance
        """
        # Get the desired position from the input fields
        x_position = self.position_input_x.get()
        y_position = self.position_input_y.get()

        if x_position and y_position:
            try:
                x_value = float(x_position)
                y_value = float(y_position)

                if y_value < 0:
                    messagebox.showerror("Invalid Input", "Y position must be at least 40.")
                    return  # Prevent further processing

                # Store the last input X and Y for filename encoding
                self.last_input_x = x_value
                self.last_input_y = y_value

                # Define multiple initial guesses
                initial_guesses = [
                    (self.v1_current, self.v2_current),
                    (self.v1_current + 180, self.v2_current),
                    (self.v1_current, self.v2_current + 180),
                    (self.v1_current + 180, self.v2_current + 180)
                ]

                # Initialising variable that will hold all possible angles
                valid_solutions = []

                for theta1_init_deg, theta2_init_deg in initial_guesses:
                    # Inverse kinematics calculation using differential method
                    result = ik_differential(x_value, y_value, self.r1, self.r2, theta1_init_deg, theta2_init_deg)

                    # Inverse kinematics calculation using analytical method
                    # result = ik_differential_ana(x_value, y_value, self.r1, self.r2, theta1_init_deg, theta2_init_deg)

                    if result is not None:
                        v1_new, v2_new = result

                        # Check joint limits
                        if v1_min <= v1_new <= v1_max and v2_min <= v2_new <= v2_max:
                            # If guess provides valid joint angles it will be saved within valid_solutions
                            valid_solutions.append((v1_new, v2_new))

                # If there is nothing in valid solutions that means that there is no valid solution
                if not valid_solutions:
                    messagebox.showerror("Error", "Position not reachable or joint limits exceeded.")
                    return

                # Choose the solution closest to current angles
                valid_solutions.sort(key=lambda angles: abs(minimal_angle_difference(angles[0], self.v1_current)) +
                                     abs(minimal_angle_difference(angles[1], self.v2_current)))

                # The first element of valid_solutions will now be the most efficient angle movement
                v1_new, v2_new = valid_solutions[0]

                # Send angles to Arduino
                send_to_arduino(v1_new, v2_new, ser)

                # Store the target angles
                self.v1_target = v1_new
                self.v2_target = v2_new

                # Update the plot to show both current and target positions
                self.plot_arm(self.v1_current, self.v2_current, self.v1_target, self.v2_target)

                # Initialize data recording if saving is enabled
                if self.save_var.get():
                    self.initialize_recording()

            except ValueError:
                messagebox.showerror("Invalid Input", "Please enter valid numeric values for X and Y positions.")

    def send_pid(self):
        """
        Sends the PID gains to the Arduino via serial communication.
        """
        try:
            # Retrieve PID values from the input fields
            Kp_m1 = float(self.Kp_m1_entry.get())
            Ki_m1 = float(self.Ki_m1_entry.get())
            Kd_m1 = float(self.Kd_m1_entry.get())
            Kp_m2 = float(self.Kp_m2_entry.get())
            Ki_m2 = float(self.Ki_m2_entry.get())
            Kd_m2 = float(self.Kd_m2_entry.get())

            # Construct the PID command string
            pid_command = f"PID:{Kp_m1},{Ki_m1},{Kd_m1},{Kp_m2},{Ki_m2},{Kd_m2}\n"

            # Send the PID command to Arduino
            ser.write(pid_command.encode('utf-8'))
            print(f"Sending to Arduino: {pid_command.strip()}")

            # Optionally, wait for Arduino confirmation
            # Wait for a short period to receive the confirmation
            time.sleep(0.5)
            while ser.in_waiting > 0:
                response = ser.readline().decode('utf-8').strip()
                print(f"Arduino: {response}")
                if response == "PID gains updated":
                    messagebox.showinfo("Success", "PID gains successfully updated.")
                elif response == "Invalid PID command format":
                    messagebox.showerror("Error", "Failed to update PID gains. Check the format.")

        except ValueError:
            messagebox.showerror("Invalid Input", "Please enter valid numeric values for PID gains.")

    def send_home_position(self):
        """
        Sends the home position to the Arduino, moving motors back to initial angles.
        Updates the last input X and Y coordinates to the home position.
        """
        try:
            # Define home coordinates
            home_x = 0.0
            home_y = 212.25

            # Define home joint angles
            home_v1 = 90.0
            home_v2 = 0.0

            # Update last input X and Y to home coordinates
            self.last_input_x = None
            self.last_input_y = None

            # Send home position to Arduino
            send_to_arduino(home_v1, home_v2, ser)

            # Update target angles
            self.v1_target = home_v1
            self.v2_target = home_v2

            # Update the plot to show both current and target positions
            self.plot_arm(self.v1_current, self.v2_current, self.v1_target, self.v2_target)

            # Initialize data recording if saving is enabled
            if self.save_var.get():
                self.initialize_recording()

            print(f"Home position command sent. Coordinates set to X={home_x}, Y={home_y}")

        except Exception as e:
            print(f"Error sending home position: {e}")
            messagebox.showerror("Error", "Failed to send home position.")

    def initialize_recording(self):
        """
        Initializes the data recording variables to start tracking performance from t=0
        """
        self.time_data = []
        self.v1_data = []
        self.v2_data = []
        self.error_v1_data = []  # Initialize error data list for V1
        self.error_v2_data = []  # Initialize error data list for V2
        self.recording = True
        self.start_time = time.time()
        self.condition_met_time = None
        print("Data recording started.")

    def toggle_path_following(self):
        """
        Enables or disables path following based on the checkbox state.
        """
        if self.path_var.get():
            # Enable path following controls
            self.load_csv_button.config(state='normal')
            if not self.path_points:
                messagebox.showinfo("Path Following", "Please load a CSV file to start path following.")
        else:
            # Disable path following controls
            self.load_csv_button.config(state='disabled')
            self.prev_button.config(state='disabled')
            self.next_button.config(state='disabled')
            self.path_points = []
            self.current_path_index = 0
            self.current_point_label.config(text="Current Point: N/A")
            print("Path following disabled.")

    def load_csv(self):
        """
        Opens a file dialog to select a CSV file and loads the path points.
        """
        file_path = filedialog.askopenfilename(
            title="Select Path CSV",
            filetypes=(("CSV Files", "*.csv"), ("All Files", "*.*"))
        )
        if file_path:
            try:
                with open(file_path, 'r') as csvfile:
                    reader = csv.reader(csvfile)
                    self.path_points = []
                    for row in reader:
                        if len(row) != 2:
                            raise ValueError("CSV must have exactly two columns: X and Y coordinates.")
                        x, y = float(row[0]), float(row[1])
                        self.path_points.append((x, y))
                if not self.path_points:
                    raise ValueError("CSV file is empty.")
                self.current_path_index = 0
                self.current_point_label.config(text=f"Current Point: 1")
                print(f"Loaded {len(self.path_points)} path points from {file_path}")

                # Plot the path points
                path_x, path_y = zip(*self.path_points)
                self.path_points_plot.set_data(path_x, path_y)
                self.canvas.draw()

                # Enable navigation buttons
                self.prev_button.config(state='disabled')  # No previous point at start
                if len(self.path_points) > 1:
                    self.next_button.config(state='normal')
                else:
                    self.next_button.config(state='disabled')

                # Start moving to the first point
                self.send_next_path_point()

            except Exception as e:
                messagebox.showerror("Error", f"Failed to load CSV file: {e}")
                self.path_points = []
                self.current_path_index = 0
                self.current_point_label.config(text="Current Point: N/A")
                self.path_points_plot.set_data([], [])
                self.canvas.draw()
        else:
            print("No CSV file selected.")

    def send_next_path_point(self):
        """
        Sends the next point in the path to the Arduino.
        """
        if self.current_path_index < len(self.path_points):
            x, y = self.path_points[self.current_path_index]
            print(f"Moving to Point {self.current_path_index + 1}: X={x}, Y={y}")

            # Update the input fields to reflect the path point
            self.position_input_x.delete(0, tk.END)
            self.position_input_x.insert(0, str(x))
            self.position_input_y.delete(0, tk.END)
            self.position_input_y.insert(0, str(y))

            # Send the position as if the user clicked the "Set Position" button
            self.send_position()

            # Update the current point label
            self.current_point_label.config(text=f"Current Point: {self.current_path_index + 1}")

    def next_point(self):
        """
        Manually sends the next point in the path to the Arduino.
        """
        if self.current_path_index < len(self.path_points) - 1:
            self.current_path_index += 1
            self.send_next_path_point()

            # Enable Previous button since we're not at the first point anymore
            self.prev_button.config(state='normal')

            # Disable Next button if we're at the last point
            if self.current_path_index == len(self.path_points) - 1:
                self.next_button.config(state='disabled')
        else:
            messagebox.showinfo("Navigation", "Already at the last point.")

    def previous_point(self):
        """
        Manually sends the previous point in the path to the Arduino.
        """
        if self.current_path_index > 0:
            self.current_path_index -= 1
            self.send_next_path_point()

            # Enable Next button since we're not at the last point anymore
            self.next_button.config(state='normal')

            # Disable Previous button if we're at the first point
            if self.current_path_index == 0:
                self.prev_button.config(state='disabled')
        else:
            messagebox.showinfo("Navigation", "Already at the first point.")

    def send_next_point_if_reached(self):
        """
        Checks if the current target point is reached within the tolerance.
        If so, sends the next point in the path.
        """
        if self.path_var.get() and self.path_points:
            if self.current_path_index < len(self.path_points):
                # Calculate the current end-effector position
                position_current = forward_kinematics(np.radians(self.v1_current), np.radians(self.v2_current), self.r1, self.r2)
                x_current, y_current = position_current[0], position_current[1]

                # Get the target point
                x_target, y_target = self.path_points[self.current_path_index]

                # Check if current position is within ±1 unit of target
                x_diff = abs(x_current - x_target)
                y_diff = abs(y_current - y_target)

                if x_diff <= 1 and y_diff <= 1:
                    print(f"Reached Point {self.current_path_index + 1}")
                    self.current_path_index += 1
                    if self.current_path_index < len(self.path_points):
                        self.current_point_label.config(text=f"Current Point: {self.current_path_index + 1}")
                        self.send_next_path_point()

                        # Enable Previous button
                        self.prev_button.config(state='normal')

                        # Disable Next button if we're at the last point
                        if self.current_path_index == len(self.path_points) - 1:
                            self.next_button.config(state='disabled')
                    else:
                        messagebox.showinfo("Path Following", "All path points have been reached.")
                        self.path_var.set(False)
                        self.toggle_path_following()

    def save_data_and_plot(self):
        """
        Saves the recorded data to a CSV file and saves the
        comprehensive time response plot and the error plot as images.
        Also updates the best settling times and saves them to a txt file.
        """
        if self.last_input_x is None or self.last_input_y is None:
            print("No input X and Y data available for filename encoding.")
            return

        
        # Use combination_count if available, otherwise fall back to sequence_number


        with self.lock:
            if self.current_combination_count is not None:
                seq_str = f"{self.current_combination_count:03d}"
            else:
                seq_str = f"s{self.sequence_number:03d}"

        # Sanitize X and Y values for filenames (replace dots with 'p' and remove negatives)
        def sanitize(value):
            return str(value).replace('.', 'p').replace('-', 'm')

        sanitized_x = sanitize(self.last_input_x)
        sanitized_y = sanitize(self.last_input_y)


        # Define filenames with sequence number
        csv_filename = os.path.join(
            self.save_dir, f"motor_data_{seq_str}_x{sanitized_x}_y{sanitized_y}.csv")
        time_response_plot_filename = os.path.join(
            self.save_dir, f"time_response_plot_{seq_str}_x{sanitized_x}_y{sanitized_y}.png")
        error_plot_filename = os.path.join(
            self.save_dir, f"error_plot_{seq_str}_x{sanitized_x}_y{sanitized_y}.png")
        best_settling_txt_filename = os.path.join(
            self.save_dir, "best_settling_times.txt")

        # Save data to CSV
        with open(csv_filename, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(["Time (s)", "V1 Angle (°)", "V2 Angle (°)", "V1 Error (°)", "V2 Error (°)"])  # Include error columns
            for t, v1, v2, e1, e2 in zip(self.time_data, self.v1_data, self.v2_data, self.error_v1_data, self.error_v2_data):
                writer.writerow([t, v1, v2, e1, e2])  # Write error data

        print(f"Data saved to {csv_filename}")

        # Generate and save the comprehensive time response plot
        self.generate_time_response_plot(time_response_plot_filename)

        # Generate and save the error plot
        self.generate_error_plot(error_plot_filename)  # Generate error plot

        # Update and save the sequence number
        self.update_sequence_number()
        self.sequence_number += 1  # Increment for next save

        # Update the save status label instead of showing a message box
        self.save_status_label.config(text="Data Saved")
        self.root.after(2000, lambda: self.save_status_label.config(text=""))  # Clear after 2 seconds

        # Update best settling times and save to txt file
        self.update_best_settling_times(best_settling_txt_filename)

        # After saving, if path following is active, proceed to next point
        if self.path_var.get() and self.path_points:
            self.send_next_point_if_reached()

    def generate_time_response_plot(self, filename):
        """
        Generates a comprehensive time response plot with setpoint lines, 2% settling bands,
        and a settling time marker.

        Parameters:
        - filename: Path to save the plot image
        """
        if not self.time_data:
            print("No data to plot.")
            return

        # Convert data lists to numpy arrays for easier manipulation
        time_array = np.array(self.time_data)
        v1_array = np.array(self.v1_data)
        v2_array = np.array(self.v2_data)

        # Create a new figure for the time response plot
        plt.figure(figsize=(12, 6))
        plt.title("Motor Time Response")
        plt.xlabel("Time (s)")
        plt.ylabel("Angle (°)")

        # Plot V1 and V2 current angles
        plt.plot(time_array, v1_array, label='V1 Current Angle', color='red')
        plt.plot(time_array, v2_array, label='V2 Current Angle', color='blue')

        # Plot setpoint lines as horizontal dotted lines
        plt.axhline(y=self.v1_target, color='red', linestyle='--', label='V1 Setpoint')
        plt.axhline(y=self.v2_target, color='blue', linestyle='--', label='V2 Setpoint')

        # Calculate 2% of setpoint angles
        v1_2_percent = 0.02 * abs(self.v1_target)
        v2_2_percent = 0.02 * abs(self.v2_target)

        # Handle cases where setpoint is zero to avoid zero bands
        v1_2_percent = v1_2_percent if v1_2_percent != 0 else 1  # Default to 1 degree if setpoint is 0
        v2_2_percent = v2_2_percent if v2_2_percent != 0 else 1

        # Plot 2% settling bands
        plt.axhline(y=self.v1_target + v1_2_percent, color='red', linestyle=':', label='V1 ±2% Band')
        plt.axhline(y=self.v1_target - v1_2_percent, color='red', linestyle=':')
        plt.axhline(y=self.v2_target + v2_2_percent, color='blue', linestyle=':', label='V2 ±2% Band')
        plt.axhline(y=self.v2_target - v2_2_percent, color='blue', linestyle=':')

        # Determine settling time
        settling_time_v1 = self.calculate_settling_time(time_array, v1_array, self.v1_target, v1_2_percent)
        settling_time_v2 = self.calculate_settling_time(time_array, v2_array, self.v2_target, v2_2_percent)

        # Plot settling time as vertical dotted lines
        if settling_time_v1 is not None:
            plt.axvline(x=settling_time_v1, color='red', linestyle='--', label='V1 Settling Time')
            plt.text(settling_time_v1, plt.ylim()[0], f"Settling Time V1: {settling_time_v1:.2f}s", color='red', rotation=90, verticalalignment='bottom')
            self.last_settling_time_v1 = settling_time_v1  # Store for tracking
        else:
            self.last_settling_time_v1 = None

        if settling_time_v2 is not None:
            plt.axvline(x=settling_time_v2, color='blue', linestyle='--', label='V2 Settling Time')
            plt.text(settling_time_v2, plt.ylim()[0], f"Settling Time V2: {settling_time_v2:.2f}s", color='blue', rotation=90, verticalalignment='bottom')
            self.last_settling_time_v2 = settling_time_v2  # Store for tracking
        else:
            self.last_settling_time_v2 = None

        # Add legend
        plt.legend()

        # Grid for better readability
        plt.grid(True)

        # Save the plot as a PNG image
        plt.tight_layout()
        plt.savefig(filename)
        plt.close()
        print(f"Time response plot saved to {filename}")

    def generate_error_plot(self, filename):
        """
        Generates an error plot showing the difference between desired and current angles over time.

        Parameters:
        - filename: Path to save the error plot image
        """
        if not self.time_data:
            print("No data to plot.")
            return

        # Convert data lists to numpy arrays for easier manipulation
        time_array = np.array(self.time_data)
        error_v1 = np.array(self.error_v1_data)
        error_v2 = np.array(self.error_v2_data)

        # Create a new figure for the error plot
        plt.figure(figsize=(12, 6))
        plt.title("Motor Angle Error Over Time")
        plt.xlabel("Time (s)")
        plt.ylabel("Error (°)")

        # Plot V1 and V2 errors
        plt.plot(time_array, error_v1, label='V1 Error', color='magenta')
        plt.plot(time_array, error_v2, label='V2 Error', color='cyan')

        # Plot zero error line
        plt.axhline(y=0, color='black', linestyle='--', label='Zero Error')

        # Add legend
        plt.legend()

        # Grid for better readability
        plt.grid(True)

        # Save the plot as a PNG image
        plt.tight_layout()
        plt.savefig(filename)
        plt.close()
        print(f"Error plot saved to {filename}")

    def calculate_settling_time(self, time_array, angle_array, setpoint, tolerance):
        """
        Calculates the settling time for a given motor angle.

        Parameters:
        - time_array: Numpy array of time data
        - angle_array: Numpy array of angle data
        - setpoint: Desired setpoint angle
        - tolerance: 2% tolerance for settling

        Returns:
        - Settling time in seconds, or None if not settled within the data
        """
        # Determine the indices where the angle is within the tolerance
        within_tolerance = np.abs(minimal_angle_difference(angle_array, setpoint)) <= tolerance

        # Find continuous segments where the angle remains within tolerance for at least condition_duration
        condition_duration = self.condition_duration  # seconds

        # Compute sampling interval as the mean of time differences
        if len(time_array) > 1:
            time_diffs = np.diff(time_array)
            # Exclude any zero or negative time differences
            time_diffs = time_diffs[time_diffs > 0]
            if len(time_diffs) == 0:
                sampling_interval = 0.02  # Default to 50Hz
            else:
                sampling_interval = np.mean(time_diffs)
        else:
            sampling_interval = 0.02  # Default to 50Hz

        # Ensure sampling_interval is positive
        if sampling_interval <= 0:
            sampling_interval = 0.02  # Default to 50Hz

        required_samples = int(condition_duration / sampling_interval)

        count = 0
        for i, within in enumerate(within_tolerance):
            if within:
                count += 1
                if count >= required_samples:
                    return time_array[i - required_samples + 1]
            else:
                count = 0

        return None  # Not settled within the recorded data

    def update_current_position(self):
        """
        Continuously reads current position data from the Arduino via serial communication
        """
        while self.running:
            try:
                if ser.in_waiting > 0:
                    line = ser.readline().decode('utf-8').strip()
                    #print(f"Arduino: {line}")  # Debug output

                    # Parsing the line to extract motor positions
                    try:
                        # Split the line by commas
                        parts = line.split(',')

                        # Ensure there are enough parts
                        if len(parts) < 4:
                            raise ValueError("Incomplete data received.")

                        # Extracting the part of the message that has current degrees for both motors
                        motor1_data = parts[0].strip()
                        motor2_data = parts[3].strip()

                        # Extracting just the angle value
                        motor1_current = float(motor1_data.split(':')[1].split('°')[0].strip())
                        motor2_current = float(motor2_data.split(':')[1].split('°')[0].strip())

                        # Put the positions in the queue for GUI update
                        self.position_queue.put((motor1_current, motor2_current))

                    except (ValueError, IndexError) as e:
                        print(f"Error parsing position data: {e}")

            except Exception as e:
                print(f"Error reading from serial: {e}")

            time.sleep(0.02)  # Reduced sleep time for more frequent reading (50Hz)

    def update_gui(self):
        """
        Update the GUI elements, such as the current position label and plot, based on data from the queue
        Also handles data recording and condition checking for saving the plot
        """
        try:
            # Check for updates in the queue
            while True:
                current_position = self.position_queue.get_nowait()  # Non-blocking
                v1_current, v2_current = current_position

                # Update the current position label
                self.current_position_label.config(
                    text=f"Current Position: v1={v1_current:.2f}°, v2={v2_current:.2f}°"
                )

                # Update current positions
                self.v1_current = v1_current
                self.v2_current = v2_current

                # Update the plot with current position only
                self.plot_arm(self.v1_current, self.v2_current, self.v1_target, self.v2_target)

                # If recording is active, log the data
                if self.recording:
                    current_time = time.time() - self.start_time
                    self.time_data.append(current_time)
                    self.v1_data.append(self.v1_current)
                    self.v2_data.append(self.v2_current)

                    # Calculate and store error data
                    error_v1 = minimal_angle_difference(self.v1_current, self.v1_target)
                    error_v2 = minimal_angle_difference(self.v2_current, self.v2_target)
                    self.error_v1_data.append(error_v1)
                    self.error_v2_data.append(error_v2)

                    # Check if current angles are within threshold of target angles
                    v1_diff = abs(error_v1)
                    v2_diff = abs(error_v2)

                    if v1_diff <= self.degrees_threshold and v2_diff <= self.degrees_threshold:
                        if self.condition_met_time is None:
                            self.condition_met_time = time.time()
                            print("Angles within threshold. Starting condition timer.")
                        else:
                            elapsed_condition_time = time.time() - self.condition_met_time
                            if elapsed_condition_time >= self.condition_duration:
                                print("Condition met for 5 seconds. Saving data and stopping recording.")
                                if self.save_var.get():
                                    self.save_data_and_plot()
                                self.recording = False
                                # If following a path, check if next point needs to be sent
                                if self.path_var.get() and self.path_points:
                                    self.send_next_point_if_reached()
                    else:
                        if self.condition_met_time is not None:
                            print("Angles moved out of threshold. Resetting condition timer.")
                        self.condition_met_time = None  # Reset the condition timer

        except Empty:
            pass  # No new data in the queue

        self.root.after(20, self.update_gui)  # Schedule the next update (50Hz)

    def on_closing(self):
        """
        Stop the thread and close the serial port upon closing the GUI
        """
        self.running = False
        self.update_thread.join()
        ser.close()  # Close the serial port
        self.root.destroy()

    def update_best_settling_times(self, filename):
        """
        Checks if the current settling times are better than the best recorded.
        If so, updates the best settling times and saves them along with PID gains to a txt file.
        Appends to the file without overwriting.
        Parameters:
        - filename: Path to the txt file where best settling times are saved
        """
        try:
            with open(filename, mode='a') as f:
                # Check and update best settling time for Motor1
                if self.last_settling_time_v1 is not None:
                    if (self.best_settling_time_v1 is None) or (self.last_settling_time_v1 < self.best_settling_time_v1):
                        self.best_settling_time_v1 = self.last_settling_time_v1
                        self.best_settling_seq_v1 = self.current_combination_count
                        self.best_settling_pid_v1 = (
                            float(self.Kp_m1_entry.get()),
                            float(self.Ki_m1_entry.get()),
                            float(self.Kd_m1_entry.get())
                        )
                        print(f"New best settling time for Motor 1: {self.best_settling_time_v1:.2f}s (Seq: {self.best_settling_seq_v1})")
                        # Append to the file
                        f.write(f"Best Settling Time for Motor 1:\n")
                        f.write(f"Combination: {self.best_settling_seq_v1}\n")
                        f.write(f"Settling Time: {self.best_settling_time_v1:.2f} seconds\n")
                        f.write(f"PID Gains: Kp={self.best_settling_pid_v1[0]}, Ki={self.best_settling_pid_v1[1]}, Kd={self.best_settling_pid_v1[2]}\n")
                        f.write(f"Reason: New best settling time achieved.\n")
                        f.write(f"Timestamp: {time.strftime('%Y-%m-%d %H:%M:%S')}\n\n")
                
                # Check and update best settling time for Motor2
                if self.last_settling_time_v2 is not None:
                    if (self.best_settling_time_v2 is None) or (self.last_settling_time_v2 < self.best_settling_time_v2):
                        self.best_settling_time_v2 = self.last_settling_time_v2
                        self.best_settling_seq_v2 = self.current_combination_count
                        self.best_settling_pid_v2 = (
                            float(self.Kp_m2_entry.get()),
                            float(self.Ki_m2_entry.get()),
                            float(self.Kd_m2_entry.get())
                        )
                        print(f"New best settling time for Motor 2: {self.best_settling_time_v2:.2f}s (Seq: {self.best_settling_seq_v2})")
                        # Append to the file
                        f.write(f"Best Settling Time for Motor 2:\n")
                        f.write(f"Combination: {self.best_settling_seq_v2}\n")
                        f.write(f"Settling Time: {self.best_settling_time_v2:.2f} seconds\n")
                        f.write(f"PID Gains: Kp={self.best_settling_pid_v2[0]}, Ki={self.best_settling_pid_v2[1]}, Kd={self.best_settling_pid_v2[2]}\n")
                        f.write(f"Reason: New best settling time achieved.\n")
                        f.write(f"Timestamp: {time.strftime('%Y-%m-%d %H:%M:%S')}\n\n")

        except IOError as e:
            print(f"Error writing to {filename}: {e}")

    def automatic_pid_tuning_both(self):
        """
        Starts the automatic PID tuning for both Motor 1 and Motor 2 in a new thread.
        Both motors' PID gains are varied simultaneously.
        """
        tuning_thread = threading.Thread(target=self.run_pid_tuning_both, daemon=True)
        tuning_thread.start()

    def run_pid_tuning_both(self):
        """
        Performs automatic PID tuning for both Motor 1 and Motor 2 at the same time.
        Each combination of PID gains is tested on both motors simultaneously.
        The best PID gains (based on the lowest settling time) for each motor are recorded.
        """
        # Disable the auto-tune buttons (if you had separate buttons before, disable both)
        self.auto_tune_both_button.config(state='disabled')

        
        try:
            # Define ranges for PID gains for both motors
            Kp_values = [0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 5.0]
            Ki_values = [0.0, 0.1, 0.2, 0.3]
            Kd_values = [0.0, 0.1, 0.2, 0.3]

            # Variables to store the best PID gains and best performance for each motor
            best_settling_time_m1 = None
            best_Kp_m1 = None
            best_Ki_m1 = None
            best_Kd_m1 = None

            best_settling_time_m2 = None
            best_Kp_m2 = None
            best_Ki_m2 = None
            best_Kd_m2 = None

            # Total number of combinations
            total_combinations = len(Kp_values) * len(Ki_values) * len(Kd_values)
            combination_count = 0

            # Progress label
            self.progress_label = tk.Label(self.pid_frame, text="", font=("Arial", 12), fg="blue")
            self.progress_label.grid(row=5, column=0, columnspan=4, pady=5)

            # File to save history of PID attempts
            history_pid_file = os.path.join(self.save_dir, "history_PID.txt")

            with open(history_pid_file, mode='a') as history_file:
                history_file.write("\n--- Starting Combined PID Tuning ---\n")
                history_file.write(f"Timestamp: {time.strftime('%Y-%m-%d %H:%M:%S')}\n\n")

                # Loop over all combinations
                for Kp in Kp_values:
                    for Ki in Ki_values:
                        for Kd in Kd_values:
                            combination_count += 1

                            # Initialize recording
                            with self.lock:
                                self.current_combination_count = combination_count
                            
                            # Update progress label
                            progress_message = f"Tuning Both Motors PID: Combo {combination_count}/{total_combinations}"
                            self.root.after(0, self.update_progress_label, progress_message)

                            # Update PID entries in the GUI for both motors
                            self.root.after(0, self.update_pid_entries_m1, Kp, Ki, Kd)
                            self.root.after(0, self.update_pid_entries_m2, Kp, Ki, Kd)

                            # Send PID gains to Arduino for both motors
                            self.send_pid_values(Kp, Ki, Kd, Kp, Ki, Kd)

                            # Return to home position before testing new PID values
                            self.root.after(0, self.send_home_position)
                            time.sleep(5)  # Wait for the arm to reach home
                            
                            self.initialize_recording()

                            # Coordinates used when grid searching PID vlaues
                            point_x = -39
                            point_y = 79
                            self.root.after(0, self.set_position, point_x, point_y)

                            # Wait for the movement to complete and data recording
                            time.sleep(10)  # Adjust as needed for your system

                            # Compute settling times
                            time_array = np.array(self.time_data)
                            v1_array = np.array(self.v1_data)
                            v2_array = np.array(self.v2_data)

                            # Settling tolerance: 2% of target angle, or default to 1 deg if 0
                            v1_tolerance = 0.02 * abs(self.v1_target) if self.v1_target != 0 else 1.0
                            v2_tolerance = 0.02 * abs(self.v2_target) if self.v2_target != 0 else 1.0

                            settling_time_v1 = self.calculate_settling_time(time_array, v1_array, self.v1_target, v1_tolerance)
                            settling_time_v2 = self.calculate_settling_time(time_array, v2_array, self.v2_target, v2_tolerance)

                            # Write PID values and settling times to history_PID.txt
                            history_file.write(f"Combination: {self.current_combination_count}\n")
                            history_file.write(f"PID Combo: Kp={Kp}, Ki={Ki}, Kd={Kd}\n")
                            history_file.write(f"Settling Time Motor 1: {settling_time_v1}\n")
                            history_file.write(f"Settling Time Motor 2: {settling_time_v2}\n")

                            # Compute final end-effector position from the last recorded angles
                            x_final, y_final = forward_kinematics(np.radians(v1_array[-1]), np.radians(v2_array[-1]), self.r1, self.r2)

                            # Write the final position to history_PID.txt
                            history_file.write(f"Final Motor Angle: V1={v1_array[-1]:.2f}, V2={v2_array[-1]:.2f}\n")
                            history_file.write(f"Final Position using Forward Kinematics: X={x_final:.2f}, Y={y_final:.2f}\n")

                            history_file.write(f"Timestamp: {time.strftime('%Y-%m-%d %H:%M:%S')}\n\n")

                            # Update best results for Motor 1
                            if settling_time_v1 is not None:
                                if best_settling_time_m1 is None or settling_time_v1 < best_settling_time_m1:
                                    best_settling_time_m1 = settling_time_v1
                                    best_Kp_m1 = Kp
                                    best_Ki_m1 = Ki
                                    best_Kd_m1 = Kd

                            # Update best results for Motor 2
                            if settling_time_v2 is not None:
                                if best_settling_time_m2 is None or settling_time_v2 < best_settling_time_m2:
                                    best_settling_time_m2 = settling_time_v2
                                    best_Kp_m2 = Kp
                                    best_Ki_m2 = Ki
                                    best_Kd_m2 = Kd

            # After completing all combinations, set the best PID values for both motors
            if best_settling_time_m1 is not None:
                self.root.after(0, self.update_pid_entries_m1, best_Kp_m1, best_Ki_m1, best_Kd_m1)
            if best_settling_time_m2 is not None:
                self.root.after(0, self.update_pid_entries_m2, best_Kp_m2, best_Ki_m2, best_Kd_m2)

            # Send the best PID gains to Arduino
            final_Kp_m1 = best_Kp_m1 if best_Kp_m1 is not None else float(self.Kp_m1_entry.get())
            final_Ki_m1 = best_Ki_m1 if best_Ki_m1 is not None else float(self.Ki_m1_entry.get())
            final_Kd_m1 = best_Kd_m1 if best_Kd_m1 is not None else float(self.Kd_m1_entry.get())

            final_Kp_m2 = best_Kp_m2 if best_Kp_m2 is not None else float(self.Kp_m2_entry.get())
            final_Ki_m2 = best_Ki_m2 if best_Ki_m2 is not None else float(self.Ki_m2_entry.get())
            final_Kd_m2 = best_Kd_m2 if best_Kd_m2 is not None else float(self.Kd_m2_entry.get())

            self.send_pid_values(final_Kp_m1, final_Ki_m1, final_Kd_m1, final_Kp_m2, final_Ki_m2, final_Kd_m2)

            # Save the best PID values found to a file
            self.save_best_pid_values_m1(self.current_combination_count, final_Kp_m1, final_Ki_m1, final_Kd_m1, best_settling_time_m1 if best_settling_time_m1 else float('inf'))
            self.save_best_pid_values_m2(self.current_combination_count, final_Kp_m2, final_Ki_m2, final_Kd_m2, best_settling_time_m2 if best_settling_time_m2 else float('inf'))

            self.root.after(0, self.update_progress_label, "Combined PID tuning complete.")

        finally:
            self.current_combination_count = None
            # Re-enable the auto-tune button
            self.root.after(0, self.auto_tune_both_button.config, {'state': 'normal'})

    def update_pid_entries_m1(self, Kp_m1, Ki_m1, Kd_m1):
        """
        Updates the PID entries for Motor 1 in the GUI
        """
        self.Kp_m1_entry.delete(0, tk.END)
        self.Kp_m1_entry.insert(0, str(Kp_m1))
        self.Ki_m1_entry.delete(0, tk.END)
        self.Ki_m1_entry.insert(0, str(Ki_m1))
        self.Kd_m1_entry.delete(0, tk.END)
        self.Kd_m1_entry.insert(0, str(Kd_m1))

    def update_pid_entries_m2(self, Kp_m2, Ki_m2, Kd_m2):
        """
        Updates the PID entries for Motor 2 in the GUI
        """
        self.Kp_m2_entry.delete(0, tk.END)
        self.Kp_m2_entry.insert(0, str(Kp_m2))
        self.Ki_m2_entry.delete(0, tk.END)
        self.Ki_m2_entry.insert(0, str(Ki_m2))
        self.Kd_m2_entry.delete(0, tk.END)
        self.Kd_m2_entry.insert(0, str(Kd_m2))

    def send_pid_values(self, Kp_m1, Ki_m1, Kd_m1, Kp_m2, Ki_m2, Kd_m2):
        """
        Sends the PID gains to the Arduino via serial communication.
        """
        try:
            # Use current PID gains if parameters are None
            if Kp_m1 is None:
                Kp_m1 = float(self.Kp_m1_entry.get())
            if Ki_m1 is None:
                Ki_m1 = float(self.Ki_m1_entry.get())
            if Kd_m1 is None:
                Kd_m1 = float(self.Kd_m1_entry.get())
            if Kp_m2 is None:
                Kp_m2 = float(self.Kp_m2_entry.get())
            if Ki_m2 is None:
                Ki_m2 = float(self.Ki_m2_entry.get())
            if Kd_m2 is None:
                Kd_m2 = float(self.Kd_m2_entry.get())

            # Construct the PID command string
            pid_command = f"PID:{Kp_m1},{Ki_m1},{Kd_m1},{Kp_m2},{Ki_m2},{Kd_m2}\n"

            # Send the PID command to Arduino
            ser.write(pid_command.encode('utf-8'))
            print(f"Sending to Arduino: {pid_command.strip()}")

            # Optionally, wait for Arduino confirmation
            # Wait for a short period to receive the confirmation
            time.sleep(0.5)
            while ser.in_waiting > 0:
                response = ser.readline().decode('utf-8').strip()
                print(f"Arduino: {response}")
                if response == "PID gains updated":
                    pass  # No action needed
                elif response == "Invalid PID command format":
                    print("Failed to update PID gains. Check the format.")

        except ValueError:
            print("Invalid PID gains. Please enter valid numeric values.")

    def set_position(self, x_value, y_value):
        """
        Sends the position command to move to (x_value, y_value).
        """
        # Update the position input fields
        self.position_input_x.delete(0, tk.END)
        self.position_input_x.insert(0, str(x_value))
        self.position_input_y.delete(0, tk.END)
        self.position_input_y.insert(0, str(y_value))
        # Send the position as if the user clicked the "Set Position" button
        self.send_position()

    def update_progress_label(self, message):
        """
        Updates the progress label with the given message.
        """
        self.progress_label.config(text=message)

    def save_best_pid_values_m1(self, combination, Kp_m1, Ki_m1, Kd_m1, settling_time):
        """
        Saves the best PID values for Motor 1 to a txt file, appending with the reason.
        """
        filename = os.path.join(self.save_dir, "best_pid_values.txt")
        try:
            with open(filename, mode='a') as f:
                f.write(f"Combination: {combination}\n")
                f.write(f"Best PID values for Motor 1:\n")
                f.write(f"Kp={Kp_m1}, Ki={Ki_m1}, Kd={Kd_m1}\n")
                f.write(f"Settling Time: {settling_time:.2f} seconds\n")
                f.write(f"Reason: Minimum settling time\n")
                f.write(f"Timestamp: {time.strftime('%Y-%m-%d %H:%M:%S')}\n\n")
            print(f"Best PID values for Motor 1 saved to {filename}")
        except IOError as e:
            print(f"Error writing to {filename}: {e}")

    def save_best_pid_values_m2(self, combination, Kp_m2, Ki_m2, Kd_m2, settling_time):
        """
        Saves the best PID values for Motor 2 to a txt file, appending with the reason.
        """
        filename = os.path.join(self.save_dir, "best_pid_values.txt")
        try:
            with open(filename, mode='a') as f:
                f.write(f"Combination: {combination}\n")
                f.write(f"Best PID values for Motor 2:\n")
                f.write(f"Kp={Kp_m2}, Ki={Ki_m2}, Kd={Kd_m2}\n")
                f.write(f"Settling Time: {settling_time:.2f} seconds\n")
                f.write(f"Reason: Minimum settling time\n")
                f.write(f"Timestamp: {time.strftime('%Y-%m-%d %H:%M:%S')}\n\n")
            print(f"Best PID values for Motor 2 saved to {filename}")
        except IOError as e:
            print(f"Error writing to {filename}: {e}")

    def on_closing(self):
        """
        Stop the thread and close the serial port upon closing the GUI
        """
        self.running = False
        self.update_thread.join()
        ser.close()  # Close the serial port
        self.root.destroy()

# Ensure that the script runs only when executed directly
if __name__ == "__main__":
    root = tk.Tk()
    app = MotorControlGUI(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()
