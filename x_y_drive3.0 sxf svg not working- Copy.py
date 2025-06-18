# -- coding: utf-8 --
import math
import serial
import serial.tools.list_ports as port_list
import sys
import numpy as np
import time
import tkinter as tk
from tkinter import ttk, filedialog, Canvas, NW
from PIL import Image, ImageTk
import cv2
import time
import threading
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

import xml.etree.ElementTree as ET
import re
from svg.path import parse_path
from svg.path.path import Line, Move, Close, CubicBezier, QuadraticBezier, Arc
import ezdxf
from ezdxf import recover


import Drawing_ui as draw


class XYStageDriver:
    def __init__(self):
        self.port, self.serialPort = self.initialize_serial()

        self.laserpower = 10
        self.scaling_factor = 0.1
        self.speed = 800
        self.acceleration = 50
        self.xpos = 0
        self.ypos = 0
        self.distance_per_roatation = 8
        self.steps_per_revolutoin = 1600

        # Empty Arrays
        self.datax = []
        self.datay = []
        self.laserstate = []


        self.move_history_x = [0]  # Start at origin
        self.move_history_y = [0]  # Start at origin
        self.move_history_laser = [0]  # Track laser states

        self.x_len = 182.88
        self.y_len = 171.96
        self.max_x = self.x_len / 2
        self.max_y = self.y_len / 2

        # Flags
        self.laseron = False
        self.runparam = True
        self.frameshape = None
        self.framecrop = None

        # **NEW: Create matplotlib figure for move tracking**
        self.fig = Figure(figsize=(5, 5), dpi=100)
        self.ax = self.fig.add_subplot(111)
        self.ax.set_xlabel('X Position (mm)')
        self.ax.set_ylabel('Y Position (mm)')
        self.ax.set_title('Movement History')
        self.ax.grid(True)
        self.ax.set_aspect('equal')

        # Initialise Gui
        self.root = tk.Tk()
        self.setup_ui()

    ########################
    # Set Up Gui
    ########################
    def setup_ui(self):
        self.root.title("XY Stage Driver")
        self.root.geometry('1400x700')

        # Create main container with scrollbar
        main_container = ttk.Frame(self.root)
        main_container.pack(fill="both", expand=True)

        # Create canvas and scrollbar
        canvas = tk.Canvas(main_container)
        scrollbar = ttk.Scrollbar(main_container, orient="vertical", command=canvas.yview)
        scrollable_frame = ttk.Frame(canvas)

        scrollable_frame.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )

        canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)

        # Store reference to scrollable frame
        self.main_frame = scrollable_frame

        # Create frames
        self.control_frame()
        self.image_frame()
        self.move_history_frame()
        self.drawing_frame()
        self.webcam_frame()

        # Pack canvas and scrollbar
        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")

        # Bind mousewheel for scrolling
        def _on_mousewheel(event):
            canvas.yview_scroll(int(-1 * (event.delta / 120)), "units")

        canvas.bind_all("<MouseWheel>", _on_mousewheel)

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.root.mainloop()

    def move_history_frame(self):
        self.history_frame = ttk.LabelFrame(self.main_frame, text="Movement History")
        self.history_frame.grid(row=0, column=2, sticky="nsew", padx=5, pady=5)

        # Initialize matplotlib figure if not exists
        if not hasattr(self, 'fig'):
            self.fig = plt.Figure(figsize=(3.5, 3), dpi=80)
            self.ax = self.fig.add_subplot(111)
            self.ax.set_xlabel('X (mm)', fontsize=8)
            self.ax.set_ylabel('Y (mm)', fontsize=8)
            self.ax.grid(True, alpha=0.3)
            self.ax.set_aspect('equal', adjustable='box')

        # Add matplotlib canvas
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.history_frame)
        self.canvas.get_tk_widget().pack(padx=5, pady=5)

        # Compact controls
        control_frame = ttk.Frame(self.history_frame)
        control_frame.pack(pady=2)

        tk.Button(control_frame, text="Clear", command=self.clear_move_history, height=1).pack(side="left", padx=5)

        # Initialize position attributes if they don't exist
        if not hasattr(self, 'xpos'):
            self.xpos = 0.0
        if not hasattr(self, 'ypos'):
            self.ypos = 0.0

        self.position_label = tk.Label(control_frame, text=f"X={self.xpos:.1f}, Y={self.ypos:.1f}", font=("Arial", 9))
        self.position_label.pack(side="left", padx=5)

    def webcam_frame(self):
        self.webcam_frame = ttk.LabelFrame(self.main_frame, text="Webcam")
        self.webcam_frame.grid(row=0, column=3, padx=10, pady=10, sticky="nsew")

        self.webcam_label = ttk.Label(self.webcam_frame)
        self.webcam_label.pack()

        # Add camera offset controls
        tk.Label(self.webcam_frame, text="Camera Y Offset (mm):").pack()
        self.camera_y_offset_var = tk.StringVar(value="-20")  # -2cm = -20mm
        tk.Entry(self.webcam_frame, textvariable=self.camera_y_offset_var, width=10).pack()

        # Add preview controls
        preview_frame = tk.Frame(self.webcam_frame)
        preview_frame.pack(pady=5)

        self.show_design_preview = tk.BooleanVar(value=True)
        tk.Checkbutton(preview_frame, text="Show Design Preview",
                       variable=self.show_design_preview).pack(side=tk.LEFT)

        self.show_travel_moves = tk.BooleanVar(value=False)
        tk.Checkbutton(preview_frame, text="Show Travel Moves",
                       variable=self.show_travel_moves).pack(side=tk.LEFT)

        # Preview position controls
        position_frame = tk.Frame(self.webcam_frame)
        position_frame.pack(pady=5)

        tk.Label(position_frame, text="Preview Offset X:").pack(side=tk.LEFT)
        self.preview_offset_x = tk.Scale(position_frame, from_=-50, to=50, orient=tk.HORIZONTAL,
                                         length=100, command=self.update_preview_offset)
        self.preview_offset_x.set(0)
        self.preview_offset_x.pack(side=tk.LEFT)

        tk.Label(position_frame, text="Y:").pack(side=tk.LEFT)
        self.preview_offset_y = tk.Scale(position_frame, from_=-50, to=50, orient=tk.HORIZONTAL,
                                         length=100, command=self.update_preview_offset)
        self.preview_offset_y.set(0)
        self.preview_offset_y.pack(side=tk.LEFT)

        # Design alignment
        tk.Button(self.webcam_frame, text="Save Preview",
                  command=self.save_preview_snapshot).pack(pady=5)

        tk.Label(self.webcam_frame, text="Right-click to align design center").pack()

        # Add right-click binding for alignment
        self.webcam_label.bind("<Button-3>", self.align_design_to_point)  # Right-click

        # Add calibration controls
        tk.Button(self.webcam_frame, text="Start Camera", command=self.start_webcam).pack(pady=5)
        tk.Button(self.webcam_frame, text="Auto Calibrate", command=self.calibrate_camera).pack(pady=5)
        tk.Button(self.webcam_frame, text="Manual Calibrate", command=self.manual_calibrate_camera).pack(pady=5)

        # Calibration status
        self.calib_status_label = tk.Label(self.webcam_frame, text="Not Calibrated", fg="red")
        self.calib_status_label.pack(pady=5)

        # Bind click event to webcam label
        self.webcam_label.bind("<Button-1>", self.on_webcam_click)

        # Add this to webcam_frame after the sliders:
        self.preview_status_label = tk.Label(self.webcam_frame, text="Preview: No offset", fg="green")
        self.preview_status_label.pack(pady=5)

        # Initialize preview offset
        self.preview_x_offset = 0
        self.preview_y_offset = 0

        # Camera calibration variables
        self.camera_calibrated = False
        self.camera_scale_x = 1.0  # mm per pixel
        self.camera_scale_y = 1.0  # mm per pixel
        self.camera_center_x = 0
        self.camera_center_y = 0

    def control_frame(self):
        # Compact padding
        pad = 3

        # Frame for laser and motor controls
        control_frame = ttk.LabelFrame(self.main_frame, text="Sequence Controls")
        control_frame.grid(row=0, column=0, sticky='nsew', padx=5, pady=5)

        # Laser - compact layout
        self.laser_var = tk.StringVar(value="10")
        tk.Label(control_frame, text='Laser (mW):', font=("Arial", 9)).grid(padx=pad, pady=pad, column=0, row=0,
                                                                            sticky='w')
        tk.Entry(control_frame, textvariable=self.laser_var, width=8).grid(padx=pad, pady=pad, column=1, row=0)
        self.setpowerbtn = tk.Button(control_frame, text="Set", command=self.set_laser_power, height=1)
        self.setpowerbtn.grid(padx=pad, pady=pad, column=2, row=0)
        self.Laserbtn = tk.Button(control_frame, text="OFF", command=self.toggle_laser, bg="lightgray", height=1)
        self.Laserbtn.grid(padx=pad, pady=pad, column=3, row=0)

        # Speed
        self.speed_var = tk.StringVar(value="100.0")
        tk.Label(control_frame, text='Speed (%):', font=("Arial", 9)).grid(padx=pad, pady=pad, column=0, row=1,
                                                                           sticky='w')
        tk.Entry(control_frame, textvariable=self.speed_var, width=8).grid(padx=pad, pady=pad, column=1, row=1)
        self.setspeedbtn = tk.Button(control_frame, text="Set", command=self.set_speed, height=1)
        self.setspeedbtn.grid(padx=pad, pady=pad, column=2, row=1)

        # Acceleration
        self.accel_var = tk.StringVar(value="5000")
        tk.Label(control_frame, text='Accel:', font=("Arial", 9)).grid(padx=pad, pady=pad, column=0, row=2, sticky='w')
        tk.Entry(control_frame, textvariable=self.accel_var, width=8).grid(padx=pad, pady=pad, column=1, row=2)

        # Movement section
        move_frame = ttk.LabelFrame(control_frame, text="Movement")
        move_frame.grid(row=3, column=0, columnspan=4, padx=pad, pady=pad, sticky='ew')

        # Move X/Y in compact layout
        tk.Button(move_frame, text="X", command=self.move_x, width=3, height=1).grid(padx=pad, pady=pad, column=0,
                                                                                     row=0)
        self.movexentry = tk.Entry(move_frame, width=8)
        self.movexentry.insert(0, "0.0")
        self.movexentry.grid(padx=pad, pady=pad, column=1, row=0)

        tk.Button(move_frame, text="Y", command=self.move_y, width=3, height=1).grid(padx=pad, pady=pad, column=2,
                                                                                     row=0)
        self.moveyentry = tk.Entry(move_frame, width=8)
        self.moveyentry.insert(0, "0.0")
        self.moveyentry.grid(padx=pad, pady=pad, column=3, row=0)

        # Home buttons in one row
        home_frame = ttk.Frame(control_frame)
        home_frame.grid(row=4, column=0, columnspan=4, padx=pad, pady=pad)

        tk.Button(home_frame, text="Set Home", command=self.set_home, height=1).pack(side="left", padx=2)
        tk.Button(home_frame, text="Go Home", command=self.move_home, height=1).pack(side="left", padx=2)
        tk.Button(home_frame, text="Find Limits", command=self.move_home_limits, height=1).pack(side="left", padx=2)

        # Jog Controls - compact
        jog_frame = ttk.LabelFrame(control_frame, text="Jog")
        jog_frame.grid(row=5, column=0, columnspan=4, padx=pad, pady=pad, sticky='ew')

        # Jog distance
        dist_frame = ttk.Frame(jog_frame)
        dist_frame.grid(row=0, column=0, columnspan=3, padx=2, pady=2)
        ttk.Label(dist_frame, text="Dist:", font=("Arial", 9)).pack(side="left")
        self.jog_distance_entry = ttk.Entry(dist_frame, width=5)
        self.jog_distance_entry.pack(side="left", padx=2)
        self.jog_distance_entry.insert(0, "2")
        ttk.Label(dist_frame, text="mm", font=("Arial", 9)).pack(side="left")

        # Compact jog buttons
        ttk.Button(jog_frame, text="↑", command=self.jog_up, width=3).grid(row=1, column=1, padx=1, pady=1)
        ttk.Button(jog_frame, text="←", command=self.jog_left, width=3).grid(row=2, column=0, padx=1, pady=1)
        ttk.Button(jog_frame, text="→", command=self.jog_right, width=3).grid(row=2, column=2, padx=1, pady=1)
        ttk.Button(jog_frame, text="↓", command=self.jog_down, width=3).grid(row=3, column=1, padx=1, pady=1)

    def image_frame(self):
        pad = 3
        image_frame = ttk.LabelFrame(self.main_frame, text="Image")
        image_frame.grid(row=0, column=1, sticky='nsew', padx=5, pady=5)

        # Control buttons in grid
        btn_frame = ttk.Frame(image_frame)
        btn_frame.grid(row=0, column=0, columnspan=2, padx=pad, pady=pad)

        tk.Button(btn_frame, text="Start", command=self.move_pen, height=1).grid(row=0, column=0, padx=2)
        tk.Button(btn_frame, text="Stop", command=self.on_close, height=1).grid(row=0, column=1, padx=2)
        tk.Button(btn_frame, text="Create Path", command=self.generate_path_from_image, height=1).grid(row=0, column=2,
                                                                                                       padx=2)
        tk.Button(btn_frame, text="Load DXF", command=self.generate_path_from_dxf, height=1).grid(row=0, column=3,
                                                                                                  padx=2)
        tk.Button(btn_frame, text="Clear", command=self.clear_data, height=1).grid(row=1, column=0, padx=2)

        # Scaling in one row
        scale_frame = ttk.Frame(image_frame)
        scale_frame.grid(row=2, column=0, columnspan=2, padx=pad, pady=pad)

        tk.Label(scale_frame, text='Scale:', font=("Arial", 9)).pack(side="left")
        self.scalingentry = tk.Entry(scale_frame, width=6)
        self.scalingentry.insert(0, "0.1")
        self.scalingentry.pack(side="left", padx=2)
        tk.Label(scale_frame, text='mm/px', font=("Arial", 9)).pack(side="left")
        tk.Button(scale_frame, text="Apply", command=self.apply_scaling, bg="lightgreen", height=1).pack(side="left",
                                                                                                         padx=5)

        # Abort button
        self.abortbtn = tk.Button(image_frame, text="STOP", command=self.toggle_abort, bg="red", fg="white", height=1)
        self.abortbtn.grid(row=3, column=0, columnspan=2, padx=pad, pady=pad)

        # Canvas - smaller
        self.imagecanvas = tk.Canvas(image_frame, height=200, width=300, bg='lightgray')
        self.imagecanvas.grid(row=4, column=0, columnspan=2, padx=pad, pady=pad)

        # Size box controls - compact
        size_frame = ttk.LabelFrame(image_frame, text="Size Box")
        size_frame.grid(row=5, column=0, columnspan=2, padx=pad, pady=pad, sticky='ew')

        # Width and height in one row
        size_row = ttk.Frame(size_frame)
        size_row.pack(pady=2)

        tk.Label(size_row, text='W:', font=("Arial", 9)).pack(side="left", padx=2)
        self.box_width_entry = tk.Entry(size_row, width=5)
        self.box_width_entry.insert(0, "50")
        self.box_width_entry.pack(side="left", padx=2)

        tk.Label(size_row, text='H:', font=("Arial", 9)).pack(side="left", padx=2)
        self.box_height_entry = tk.Entry(size_row, width=5)
        self.box_height_entry.insert(0, "50")
        self.box_height_entry.pack(side="left", padx=2)

        tk.Label(size_row, text='mm', font=("Arial", 9)).pack(side="left", padx=2)

        # Size buttons
        btn_row = ttk.Frame(size_frame)
        btn_row.pack(pady=2)
        tk.Button(btn_row, text="Show Box", command=self.show_size_box, height=1).pack(side="left", padx=2)
        tk.Button(btn_row, text="Set Scale", command=self.set_scaling_from_size, height=1).pack(side="left", padx=2)

    def drawing_frame(self):
        """Create a frame for drawing shapes and lines"""
        self.draw_frame = ttk.LabelFrame(self.main_frame, text="Drawing Tool")
        self.draw_frame.grid(row=1, column=0, columnspan=3, sticky="nsew", padx=5, pady=5)

        # Create two columns in drawing frame
        left_frame = ttk.Frame(self.draw_frame)
        left_frame.grid(row=0, column=0, padx=5, pady=5)

        right_frame = ttk.Frame(self.draw_frame)
        right_frame.grid(row=0, column=1, padx=5, pady=5)

        # Drawing canvas - smaller
        self.drawing_canvas = tk.Canvas(left_frame, width=300, height=300, bg='white')
        self.drawing_canvas.pack()

        # Bind mouse events
        self.drawing_canvas.bind("<Button-1>", self.on_canvas_click)
        self.drawing_canvas.bind("<B1-Motion>", self.on_canvas_drag)
        self.drawing_canvas.bind("<ButtonRelease-1>", self.on_canvas_release)

        # Controls in right frame
        # Drawing mode selection - compact
        mode_frame = ttk.LabelFrame(right_frame, text="Drawing Mode")
        mode_frame.pack(fill="x", pady=5)

        self.drawing_mode = tk.StringVar(value="line")
        modes = [("Line", "line"), ("Rect", "rectangle"), ("Circle", "circle"),
                 ("Free", "free"), ("Poly", "polygon")]

        for i, (text, mode) in enumerate(modes):
            tk.Radiobutton(mode_frame, text=text, variable=self.drawing_mode,
                           value=mode, font=("Arial", 9)).grid(row=i // 2, column=i % 2, padx=2, pady=2, sticky='w')

        # Control buttons
        btn_frame = ttk.Frame(right_frame)
        btn_frame.pack(fill="x", pady=5)

        tk.Button(btn_frame, text="Clear", command=self.clear_drawing_canvas, height=1).grid(row=0, column=0, padx=2,
                                                                                             pady=2)
        tk.Button(btn_frame, text="Undo", command=self.undo_last_drawing, height=1).grid(row=0, column=1, padx=2,
                                                                                         pady=2)
        tk.Button(btn_frame, text="Generate", command=self.generate_path_from_drawing, height=1).grid(row=1, column=0,
                                                                                                      padx=2, pady=2)
        tk.Button(btn_frame, text="Save", command=self.save_drawing, height=1).grid(row=1, column=1, padx=2, pady=2)
        tk.Button(btn_frame, text="Load", command=self.load_drawing, height=1).grid(row=2, column=0, padx=2, pady=2)
        tk.Button(btn_frame, text="Export DXF", command=self.export_path_as_dxf, height=1).grid(row=2, column=1, padx=2,
                                                                                                pady=2)  # NEW BUTTON
        tk.Button(btn_frame, text="Optimize", command=self.optimize_dxf_paths, height=1).grid(row=3, column=0, padx=2,
                                                                                              pady=2)

        # Canvas size
        size_frame = ttk.Frame(right_frame)
        size_frame.pack(fill="x", pady=5)

        tk.Label(size_frame, text="Canvas:", font=("Arial", 9)).pack(side="left")
        self.canvas_size_var = tk.StringVar(value="100")
        tk.Entry(size_frame, textvariable=self.canvas_size_var, width=5).pack(side="left", padx=2)
        tk.Label(size_frame, text="mm", font=("Arial", 9)).pack(side="left")

        # Grid toggle
        self.show_grid_var = tk.BooleanVar(value=True)
        tk.Checkbutton(size_frame, text="Grid", variable=self.show_grid_var,
                       command=self.update_drawing_grid, font=("Arial", 9)).pack(side="left", padx=10)

        # Initialize drawing variables
        self.drawing_objects = []
        self.current_drawing = None
        self.start_x = None
        self.start_y = None
        self.polygon_points = []
        self.temp_line = None

        # Draw initial grid
        self.update_drawing_grid()

    ###
    # webcam stuff
    ###

    def calibrate_camera(self):
        """Start camera calibration process"""
        if not hasattr(self, 'cap') or not self.cap.isOpened():
            print("Start camera first!")
            return

        # Get current camera frame size
        ret, frame = self.cap.read()
        if not ret:
            print("Failed to read camera frame")
            return

        h, w = frame.shape[:2]
        self.camera_center_x = w // 2
        self.camera_center_y = h // 2

        # Initialize calibration state
        self.calibration_positions = [
            (0, 0),  # Center
            (20, 0),  # Right
            (0, 20),  # Up
            (-20, -20)  # Left-Down
        ]
        self.calibration_clicked_points = []
        self.calibration_current_index = 0
        self.calibration_active = True

        tk.messagebox.showinfo("Camera Calibration",
                               "The system will move to 4 positions.\n"
                               "Click where the laser appears in the camera view.")

        # Start the first calibration move
        self.next_calibration_step()

    def next_calibration_step(self):
        """Move to next calibration position"""
        if self.calibration_current_index >= len(self.calibration_positions):
            # Calibration complete
            self.finish_calibration()
            return

        x, y = self.calibration_positions[self.calibration_current_index]

        # Update status
        print(f"Moving to calibration position {self.calibration_current_index + 1}/4: ({x}, {y})")

        # Move to position
        self.serialPort.write(f"MOVXY {x},{y},0\n".encode("utf8"))
        self.update_position(x, y, 0)

        # Schedule laser on after movement (give time for move to complete)
        self.root.after(1000, self.turn_on_calibration_laser)

    def turn_on_calibration_laser(self):
        """Turn on laser for calibration"""
        self.serialPort.write(f"LASON {10}\n".encode("utf8"))
        print(f"Click on the laser spot (Position {self.calibration_current_index + 1}/4)")

    def finish_calibration(self):
        """Complete the calibration process"""
        self.calibration_active = False

        if len(self.calibration_clicked_points) >= 2:
            # Calculate X scale from positions 0->1
            dx_pixels = self.calibration_clicked_points[1][0] - self.calibration_clicked_points[0][0]
            dx_mm = self.calibration_positions[1][0] - self.calibration_positions[0][0]
            if dx_pixels != 0:
                self.camera_scale_x = abs(dx_mm / dx_pixels)

        if len(self.calibration_clicked_points) >= 3:
            # Calculate Y scale from positions 0->2
            dy_pixels = self.calibration_clicked_points[2][1] - self.calibration_clicked_points[0][1]
            dy_mm = self.calibration_positions[2][1] - self.calibration_positions[0][1]
            if dy_pixels != 0:
                self.camera_scale_y = abs(dy_mm / dy_pixels)

        self.camera_calibrated = True

        # Update calibration status label
        self.calib_status_label.config(
            text=f"Calibrated: {self.camera_scale_x:.3f} x {self.camera_scale_y:.3f} mm/px",
            fg="green"
        )

        print(f"Camera calibrated: Scale X={self.camera_scale_x:.3f} mm/pixel, Y={self.camera_scale_y:.3f} mm/pixel")

        # Return to origin
        self.serialPort.write(b"MOVXY 0,0,0\n")
        self.update_position(0, 0, 0)

        tk.messagebox.showinfo("Calibration Complete",
                               f"Camera calibrated successfully!\n"
                               f"Scale: {self.camera_scale_x:.3f} x {self.camera_scale_y:.3f} mm/pixel")

    def start_webcam(self):
        """Start the webcam with click functionality"""
        if not hasattr(self, 'cap') or not self.cap.isOpened():
            self.cap = cv2.VideoCapture(0)
            if self.cap.isOpened():
                self.webcam_running = True
                self.webcam_thread = threading.Thread(target=self.update_webcam_thread, daemon=True)
                self.webcam_thread.start()
                print("Webcam started")
            else:
                print("Failed to open webcam")

    def update_webcam_thread(self):
        """Update webcam display with crosshair and design overlay"""
        while self.webcam_running:
            ret, frame = self.cap.read()
            if ret:
                h, w = frame.shape[:2]

                # Create overlay image
                overlay = frame.copy()

                # Draw crosshair at center
                cv2.line(overlay, (w // 2 - 20, h // 2), (w // 2 + 20, h // 2), (0, 255, 0), 2)
                cv2.line(overlay, (w // 2, h // 2 - 20), (w // 2, h // 2 + 20), (0, 255, 0), 2)

                # Draw current position if calibrated
                if self.camera_calibrated:
                    # Current machine position in camera pixels
                    cam_x = int(self.camera_center_x + self.xpos / self.camera_scale_x)
                    cam_y = int(self.camera_center_y - (
                                self.ypos + float(self.camera_y_offset_var.get())) / self.camera_scale_y)

                    if 0 <= cam_x < w and 0 <= cam_y < h:
                        cv2.circle(overlay, (cam_x, cam_y), 5, (255, 0, 0), -1)
                        cv2.putText(overlay, "Current", (cam_x + 10, cam_y),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)

                    # Draw design preview if available and enabled
                    if hasattr(self, 'show_design_preview') and self.show_design_preview.get():
                        self.draw_design_preview(overlay)

                # Blend overlay with original frame
                alpha = 0.7  # Transparency
                frame = cv2.addWeighted(frame, 1 - alpha, overlay, alpha, 0)

                # Convert and display
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                img = Image.fromarray(frame)
                imgtk = ImageTk.PhotoImage(image=img)

                def update_gui():
                    self.webcam_label.config(image=imgtk)
                    self.webcam_label.image = imgtk

                self.webcam_label.after(0, update_gui)

            time.sleep(0.03)  # ~30 FPS

    def on_webcam_click(self, event):
        """Handle click on webcam image"""

        if hasattr(self, 'calibration_active') and self.calibration_active:
            # Store calibration click
            self.calibration_clicked_points.append((event.x, event.y))
            print(f"Calibration point {self.calibration_current_index + 1} clicked at ({event.x}, {event.y})")

            # Turn off laser
            self.serialPort.write(b"LASON 0\n")

            # Move to next position
            self.calibration_current_index += 1

            # Schedule next step after a short delay
            self.root.after(500, self.next_calibration_step)
            return

        if not self.camera_calibrated:
            print("Camera not calibrated! Run calibration first.")
            return

        # Get click position in pixels
        click_x = event.x
        click_y = event.y

        # Convert to mm coordinates relative to camera center
        x_mm = (click_x - self.camera_center_x) * self.camera_scale_x
        y_mm = -(click_y - self.camera_center_y) * self.camera_scale_y  # Negative because Y is inverted

        # Apply camera offset (camera is 20mm below actual position)
        y_offset = float(self.camera_y_offset_var.get())
        y_mm_actual = y_mm - y_offset  # Subtract offset to get actual position

        print(f"Camera click at pixel ({click_x}, {click_y})")
        print(f"Converted to mm: ({x_mm:.2f}, {y_mm:.2f})")
        print(f"With offset correction: ({x_mm:.2f}, {y_mm_actual:.2f})")

        # Check if position is within workspace limits
        if abs(x_mm) > self.max_x or abs(y_mm_actual) > self.max_y:
            print("Warning: Position outside workspace limits!")
            response = tk.messagebox.askyesno("Warning",
                                              f"Position ({x_mm:.1f}, {y_mm_actual:.1f}) is outside workspace.\n"
                                              f"Move anyway?")
            if not response:
                return

        # Move to clicked position
        movecommand = f"MOVXY {x_mm:.2f},{y_mm_actual:.2f},0\n"
        self.serialPort.write(movecommand.encode("utf8"))
        self.update_position(x_mm, y_mm_actual, 0)

        # Optional: Flash laser to confirm position
        self.serialPort.write(f"LASON {30}\n".encode("utf8"))
        time.sleep(0.1)
        self.serialPort.write(b"LASON 0\n")

    def draw_design_preview(self, image):
        """Draw the current design path on the camera image"""
        if not self.datax or not self.datay or not self.camera_calibrated:
            return

        h, w = image.shape[:2]
        y_offset = float(self.camera_y_offset_var.get())

        # Convert path points to camera pixels
        points = []
        for i in range(len(self.datax)):
            # Convert mm to camera pixels
            cam_x = int(self.camera_center_x + self.datax[i] / self.camera_scale_x)
            cam_y = int(self.camera_center_y - (self.datay[i] + y_offset) / self.camera_scale_y)

            # Only include points that are within the image bounds
            if 0 <= cam_x < w and 0 <= cam_y < h:
                points.append((cam_x, cam_y, self.laserstate[i]))

        # Draw the path
        for i in range(1, len(points)):
            x1, y1, laser1 = points[i - 1]
            x2, y2, laser2 = points[i]

            # Choose color based on laser state
            if laser2 > 0:  # Laser on - draw in red
                color = (0, 0, 255)
                thickness = 2
            else:  # Laser off - draw in blue dashed
                color = (255, 128, 0)
                thickness = 1

            # Draw line
            if laser2 > 0 or hasattr(self, 'show_travel_moves') and self.show_travel_moves.get():
                cv2.line(image, (x1, y1), (x2, y2), color, thickness)

        # Highlight start and end points
        if points:
            # Start point - green circle
            cv2.circle(image, points[0][:2], 8, (0, 255, 0), -1)
            cv2.putText(image, "START", (points[0][0] + 10, points[0][1]),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

            # End point - red circle
            cv2.circle(image, points[-1][:2], 8, (0, 0, 255), -1)
            cv2.putText(image, "END", (points[-1][0] + 10, points[-1][1]),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

    def update_preview_offset(self, value):
        """Update preview offset when sliders change"""
        self.preview_x_offset = self.preview_offset_x.get()
        self.preview_y_offset = self.preview_offset_y.get()

    def draw_design_preview_with_offset(self, image):
        """Draw the current design path on the camera image with adjustable offset"""
        if not self.datax or not self.datay or not self.camera_calibrated:
            return

        h, w = image.shape[:2]
        y_offset = float(self.camera_y_offset_var.get())

        # Get preview offsets
        x_offset = getattr(self, 'preview_x_offset', 0)
        y_offset_preview = getattr(self, 'preview_y_offset', 0)

        # Convert path points to camera pixels with preview offset
        points = []
        for i in range(len(self.datax)):
            # Convert mm to camera pixels with offset
            cam_x = int(self.camera_center_x + (self.datax[i] + x_offset) / self.camera_scale_x)
            cam_y = int(self.camera_center_y - (self.datay[i] + y_offset + y_offset_preview) / self.camera_scale_y)

            # Include all points for path continuity
            points.append((cam_x, cam_y, self.laserstate[i]))

        # Draw bounding box of design
        if points:
            x_coords = [p[0] for p in points if 0 <= p[0] < w]
            y_coords = [p[1] for p in points if 0 <= p[1] < h]

            if x_coords and y_coords:
                min_x, max_x = min(x_coords), max(x_coords)
                min_y, max_y = min(y_coords), max(y_coords)

                # Draw bounding box
                cv2.rectangle(image, (min_x, min_y), (max_x, max_y), (128, 128, 128), 1)

                # Show dimensions
                width_mm = (max_x - min_x) * self.camera_scale_x
                height_mm = (max_y - min_y) * self.camera_scale_y
                cv2.putText(image, f"{width_mm:.1f}mm x {height_mm:.1f}mm",
                            (min_x, min_y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (128, 128, 128), 1)

                # Draw the path
                for i in range(1, len(points)):
                    x1, y1, laser1 = points[i - 1]
                    x2, y2, laser2 = points[i]

                    # Skip if line is completely outside image bounds
                    if (x1 < 0 and x2 < 0) or (x1 >= w and x2 >= w) or \
                            (y1 < 0 and y2 < 0) or (y1 >= h and y2 >= h):
                        continue

                    # Clip line to image bounds
                    x1 = max(0, min(w - 1, x1))
                    x2 = max(0, min(w - 1, x2))
                    y1 = max(0, min(h - 1, y1))
                    y2 = max(0, min(h - 1, y2))

                    # Choose color based on laser state
                    if laser2 > 0:  # Laser on - draw in red
                        color = (0, 0, 255)
                        thickness = 2
                    else:  # Laser off - draw in blue dashed
                        color = (255, 128, 0)
                        thickness = 1

                    # Draw line
                    if laser2 > 0 or (hasattr(self, 'show_travel_moves') and self.show_travel_moves.get()):
                        cv2.line(image, (x1, y1), (x2, y2), color, thickness)

                # Highlight start and end points if visible
                if points:
                    # Start point - green circle
                    start_x, start_y = points[0][0], points[0][1]
                    if 0 <= start_x < w and 0 <= start_y < h:
                        cv2.circle(image, (start_x, start_y), 8, (0, 255, 0), -1)
                        cv2.putText(image, "START", (start_x + 10, start_y),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

                    # End point - red circle
                    end_x, end_y = points[-1][0], points[-1][1]
                    if 0 <= end_x < w and 0 <= end_y < h:
                        cv2.circle(image, (end_x, end_y), 8, (0, 0, 255), -1)
                        cv2.putText(image, "END", (end_x + 10, end_y),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

    def save_preview_snapshot(self):
        """Save current camera view with design overlay"""
        if not hasattr(self, 'cap') or not self.cap.isOpened():
            print("Camera not running!")
            return

        ret, frame = self.cap.read()
        if ret:
            # Draw overlay
            if self.camera_calibrated and hasattr(self, 'show_design_preview') and self.show_design_preview.get():
                self.draw_design_preview_with_offset(frame)

            # Save image
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            filename = f"preview_{timestamp}.png"
            cv2.imwrite(filename, frame)
            print(f"Preview saved as {filename}")

    def align_design_to_point(self, event):
        """Right-click to align design center to clicked point"""
        if not self.camera_calibrated:
            print("Camera not calibrated!")
            return

        # Get click position in mm
        click_x = event.x
        click_y = event.y

        x_mm = (click_x - self.camera_center_x) * self.camera_scale_x
        y_mm = -(click_y - self.camera_center_y) * self.camera_scale_y

        # Calculate design center
        if self.datax and self.datay:
            design_center_x = (min(self.datax) + max(self.datax)) / 2
            design_center_y = (min(self.datay) + max(self.datay)) / 2

            # Calculate required offset
            offset_x = x_mm - design_center_x
            offset_y = y_mm - design_center_y - float(self.camera_y_offset_var.get())

            # Update preview offset sliders
            self.preview_offset_x.set(offset_x)
            self.preview_offset_y.set(offset_y)

            print(f"Design aligned to ({x_mm:.1f}, {y_mm:.1f})")

    def manual_calibrate_camera(self):
        """Manual calibration without moving the machine"""
        if not hasattr(self, 'cap') or not self.cap.isOpened():
            print("Start camera first!")
            return

        dialog = tk.Toplevel(self.root)
        dialog.title("Manual Camera Calibration")
        dialog.geometry("400x300")

        tk.Label(dialog, text="Manual Camera Calibration", font=("Arial", 14, "bold")).pack(pady=10)
        tk.Label(dialog, text="Measure a known distance in the camera view").pack()

        # Method 1: Known object
        method1_frame = tk.LabelFrame(dialog, text="Method 1: Known Object Size")
        method1_frame.pack(padx=10, pady=10, fill="x")

        tk.Label(method1_frame, text="Place a ruler or known object in view").pack()
        tk.Label(method1_frame, text="Object width (mm):").pack()
        width_var = tk.StringVar(value="50")
        tk.Entry(method1_frame, textvariable=width_var).pack()

        tk.Label(method1_frame, text="Object width in pixels:").pack()
        pixels_var = tk.StringVar(value="100")
        tk.Entry(method1_frame, textvariable=pixels_var).pack()

        # Method 2: Full view estimation
        method2_frame = tk.LabelFrame(dialog, text="Method 2: Estimate Full View")
        method2_frame.pack(padx=10, pady=10, fill="x")

        tk.Label(method2_frame, text="Estimate visible area (mm):").pack()
        view_width_var = tk.StringVar(value="150")
        tk.Entry(method2_frame, textvariable=view_width_var).pack()

        def apply_method1():
            try:
                obj_width_mm = float(width_var.get())
                obj_width_pixels = float(pixels_var.get())

                # Get camera dimensions
                ret, frame = self.cap.read()
                if ret:
                    h, w = frame.shape[:2]

                    # Calculate scale
                    scale = obj_width_mm / obj_width_pixels
                    self.camera_scale_x = scale
                    self.camera_scale_y = scale
                    self.camera_center_x = w // 2
                    self.camera_center_y = h // 2
                    self.camera_calibrated = True

                    print(f"Camera calibrated: {scale:.3f} mm/pixel")
                    dialog.destroy()
            except Exception as e:
                print(f"Calibration error: {e}")

        def apply_method2():
            try:
                view_width_mm = float(view_width_var.get())

                # Get camera dimensions
                ret, frame = self.cap.read()
                if ret:
                    h, w = frame.shape[:2]

                    # Calculate scale
                    scale = view_width_mm / w
                    self.camera_scale_x = scale
                    self.camera_scale_y = scale
                    self.camera_center_x = w // 2
                    self.camera_center_y = h // 2
                    self.camera_calibrated = True

                    print(f"Camera calibrated: {scale:.3f} mm/pixel")
                    dialog.destroy()
            except Exception as e:
                print(f"Calibration error: {e}")

        tk.Button(method1_frame, text="Apply Method 1", command=apply_method1).pack(pady=5)
        tk.Button(method2_frame, text="Apply Method 2", command=apply_method2).pack(pady=5)

    #####
    # drawing helpers
    ####

    def update_drawing_grid(self):
        """Draw or remove grid on canvas"""
        # Remove existing grid lines
        self.drawing_canvas.delete("grid")

        if self.show_grid_var.get():
            # Draw grid
            canvas_width = 400
            canvas_height = 400
            grid_spacing = 20  # pixels

            # Vertical lines
            for x in range(0, canvas_width, grid_spacing):
                self.drawing_canvas.create_line(x, 0, x, canvas_height,
                                                fill="lightgray", tags="grid")

            # Horizontal lines
            for y in range(0, canvas_height, grid_spacing):
                self.drawing_canvas.create_line(0, y, canvas_width, y,
                                                fill="lightgray", tags="grid")

            # Center lines
            self.drawing_canvas.create_line(canvas_width // 2, 0, canvas_width // 2, canvas_height,
                                            fill="gray", tags="grid")
            self.drawing_canvas.create_line(0, canvas_height // 2, canvas_width, canvas_height // 2,
                                            fill="gray", tags="grid")

        # Make sure grid is behind other objects
        self.drawing_canvas.tag_lower("grid")

    def on_canvas_click(self, event):
        """Handle mouse click on drawing canvas"""
        self.start_x = event.x
        self.start_y = event.y

        mode = self.drawing_mode.get()

        if mode == "polygon":
            # Add point to polygon
            self.polygon_points.extend([event.x, event.y])

            if len(self.polygon_points) >= 4:
                # Draw temporary line to show next segment
                if self.temp_line:
                    self.drawing_canvas.delete(self.temp_line)
                self.temp_line = self.drawing_canvas.create_line(
                    self.polygon_points[-2], self.polygon_points[-1],
                    event.x, event.y, fill="gray", dash=(5, 5))

            # Draw a small circle at each point
            self.drawing_canvas.create_oval(event.x - 3, event.y - 3, event.x + 3, event.y + 3,
                                            fill="red", tags="polygon_point")

            # Check if polygon should be closed (click near start point)
            if len(self.polygon_points) >= 6:
                dist = ((self.polygon_points[0] - event.x) ** 2 +
                        (self.polygon_points[1] - event.y) ** 2) ** 0.5
                if dist < 10:
                    # Close polygon
                    self.finish_polygon()

        elif mode == "free":
            # Start free drawing
            self.current_drawing = []

    def on_canvas_drag(self, event):
        """Handle mouse drag on drawing canvas"""
        mode = self.drawing_mode.get()

        if mode == "line":
            # Update temporary line
            if self.current_drawing:
                self.drawing_canvas.delete(self.current_drawing)
            self.current_drawing = self.drawing_canvas.create_line(
                self.start_x, self.start_y, event.x, event.y, fill="black", width=2)

        elif mode == "rectangle":
            # Update temporary rectangle
            if self.current_drawing:
                self.drawing_canvas.delete(self.current_drawing)
            self.current_drawing = self.drawing_canvas.create_rectangle(
                self.start_x, self.start_y, event.x, event.y,
                outline="black", width=2, fill="")

        elif mode == "circle":
            # Update temporary circle
            if self.current_drawing:
                self.drawing_canvas.delete(self.current_drawing)
            radius = ((event.x - self.start_x) ** 2 + (event.y - self.start_y) ** 2) ** 0.5
            self.current_drawing = self.drawing_canvas.create_oval(
                self.start_x - radius, self.start_y - radius,
                self.start_x + radius, self.start_y + radius,
                outline="black", width=2, fill="")

        elif mode == "free":
            # Add line segment
            line = self.drawing_canvas.create_line(
                self.start_x, self.start_y, event.x, event.y,
                fill="black", width=2, tags="free_draw")
            self.current_drawing.append(line)
            self.start_x = event.x
            self.start_y = event.y

    def on_canvas_release(self, event):
        """Handle mouse release on drawing canvas"""
        mode = self.drawing_mode.get()

        if mode in ["line", "rectangle", "circle"]:
            if self.current_drawing:
                # Finalize the shape
                self.drawing_objects.append({
                    'type': mode,
                    'id': self.current_drawing,
                    'coords': self.drawing_canvas.coords(self.current_drawing)
                })
                self.current_drawing = None

        elif mode == "free":
            if self.current_drawing:
                # Store free drawing as a series of lines
                self.drawing_objects.append({
                    'type': 'free',
                    'id': self.current_drawing,
                    'coords': [self.drawing_canvas.coords(line) for line in self.current_drawing]
                })
                self.current_drawing = None

    def finish_polygon(self):
        """Finish drawing a polygon"""
        if len(self.polygon_points) >= 6:
            # Create polygon
            polygon = self.drawing_canvas.create_polygon(
                self.polygon_points, outline="black", width=2, fill="")

            self.drawing_objects.append({
                'type': 'polygon',
                'id': polygon,
                'coords': self.polygon_points.copy()
            })

            # Clean up
            self.drawing_canvas.delete("polygon_point")
            if self.temp_line:
                self.drawing_canvas.delete(self.temp_line)
            self.polygon_points = []
            self.temp_line = None

    def clear_drawing_canvas(self):
        """Clear all drawings from canvas"""
        # Delete all drawn objects except grid
        for obj in self.drawing_objects:
            if obj['type'] == 'free':
                for line_id in obj['id']:
                    self.drawing_canvas.delete(line_id)
            else:
                self.drawing_canvas.delete(obj['id'])

        self.drawing_objects = []
        self.polygon_points = []
        if self.temp_line:
            self.drawing_canvas.delete(self.temp_line)
        self.drawing_canvas.delete("polygon_point")

    def undo_last_drawing(self):
        """Remove the last drawn object"""
        if self.drawing_objects:
            last_obj = self.drawing_objects.pop()
            if last_obj['type'] == 'free':
                for line_id in last_obj['id']:
                    self.drawing_canvas.delete(line_id)
            else:
                self.drawing_canvas.delete(last_obj['id'])

    def generate_path_from_drawing(self):
        """Convert drawn shapes to machine path"""
        if not self.drawing_objects:
            print("No drawings to convert!")
            return

        # Clear existing path data
        self.datax = []
        self.datay = []
        self.laserstate = []

        # Get canvas size in mm
        try:
            canvas_size_mm = float(self.canvas_size_var.get())
        except:
            canvas_size_mm = 100
            print("Invalid canvas size, using 100mm")

        # Canvas dimensions in pixels
        canvas_width = 400
        canvas_height = 400

        # Scaling factor (mm per pixel)
        scale = canvas_size_mm / canvas_width

        # Center offset
        center_x = canvas_width / 2
        center_y = canvas_height / 2

        # Ensure laser power is set
        if self.laserpower == 0:
            self.laserpower = 255

        print(f"Generating path from {len(self.drawing_objects)} objects")
        print(f"Canvas size: {canvas_size_mm}mm, Scale: {scale:.3f} mm/pixel")

        for obj in self.drawing_objects:
            if obj['type'] == 'line':
                # Convert line
                x1, y1, x2, y2 = obj['coords']

                # Move to start with laser off
                self.datax.append((x1 - center_x) * scale)
                self.datay.append((center_y - y1) * scale)  # Invert Y
                self.laserstate.append(0)

                # Draw line with laser on
                self.datax.append((x2 - center_x) * scale)
                self.datay.append((center_y - y2) * scale)  # Invert Y
                self.laserstate.append(self.laserpower)

            elif obj['type'] == 'rectangle':
                # Convert rectangle
                x1, y1, x2, y2 = obj['coords']

                # Move to start with laser off
                self.datax.append((x1 - center_x) * scale)
                self.datay.append((center_y - y1) * scale)
                self.laserstate.append(0)

                # Draw rectangle with laser on
                points = [(x1, y1), (x2, y1), (x2, y2), (x1, y2), (x1, y1)]
                for x, y in points:
                    self.datax.append((x - center_x) * scale)
                    self.datay.append((center_y - y) * scale)
                    self.laserstate.append(self.laserpower)

            elif obj['type'] == 'circle':
                # Convert circle to polygon approximation
                x1, y1, x2, y2 = obj['coords']
                cx = (x1 + x2) / 2
                cy = (y1 + y2) / 2
                radius = abs(x2 - x1) / 2

                # Generate circle points (36 segments)
                num_segments = 36
                angle_step = 2 * 3.14159 / num_segments

                # Move to first point with laser off
                first_x = cx + radius
                first_y = cy
                self.datax.append((first_x - center_x) * scale)
                self.datay.append((center_y - first_y) * scale)
                self.laserstate.append(0)

                # Draw circle with laser on
                for i in range(num_segments + 1):
                    angle = i * angle_step
                    x = cx + radius * math.cos(angle)
                    y = cy + radius * math.sin(angle)
                    self.datax.append((x - center_x) * scale)
                    self.datay.append((center_y - y) * scale)
                    self.laserstate.append(self.laserpower)

            elif obj['type'] == 'polygon':
                # Convert polygon
                coords = obj['coords']

                # Move to first point with laser off
                self.datax.append((coords[0] - center_x) * scale)
                self.datay.append((center_y - coords[1]) * scale)
                self.laserstate.append(0)

                # Draw polygon with laser on
                for i in range(0, len(coords), 2):
                    x = coords[i]
                    y = coords[i + 1]
                    self.datax.append((x - center_x) * scale)
                    self.datay.append((center_y - y) * scale)
                    self.laserstate.append(self.laserpower)

                # Close polygon
                self.datax.append((coords[0] - center_x) * scale)
                self.datay.append((center_y - coords[1]) * scale)
                self.laserstate.append(self.laserpower)

            elif obj['type'] == 'free':
                # Convert free drawing
                first = True
                for line_coords in obj['coords']:
                    x1, y1, x2, y2 = line_coords

                    if first:
                        # Move to start with laser off
                        self.datax.append((x1 - center_x) * scale)
                        self.datay.append((center_y - y1) * scale)
                        self.laserstate.append(0)
                        first = False

                    # Draw segment with laser on
                    self.datax.append((x2 - center_x) * scale)
                    self.datay.append((center_y - y2) * scale)
                    self.laserstate.append(self.laserpower)

            # Print summary
            if self.datax:
                x_min, x_max = min(self.datax), max(self.datax)
                y_min, y_max = min(self.datay), max(self.datay)
                print(f"Generated {len(self.datax)} points")
                print(f"Output size: {x_max - x_min:.1f} x {y_max - y_min:.1f} mm")
                print(f"X range: {x_min:.1f} to {x_max:.1f} mm")
                print(f"Y range: {y_min:.1f} to {y_max:.1f} mm")

                # Update the plot to show the generated path
                self.update_move_plot()

    def add_drawing_shortcuts(self):
        """Add keyboard shortcuts for drawing"""
        self.root.bind('<Control-z>', lambda e: self.undo_last_drawing())
        self.root.bind('<Delete>', lambda e: self.clear_drawing_canvas())
        self.root.bind('<Escape>', lambda e: self.cancel_current_drawing())

    def cancel_current_drawing(self):
        """Cancel current drawing operation"""
        if self.drawing_mode.get() == "polygon" and self.polygon_points:
            self.polygon_points = []
            self.drawing_canvas.delete("polygon_point")
            if self.temp_line:
                self.drawing_canvas.delete(self.temp_line)
                self.temp_line = None
        elif self.current_drawing:
            if isinstance(self.current_drawing, list):
                for line_id in self.current_drawing:
                    self.drawing_canvas.delete(line_id)
            else:
                self.drawing_canvas.delete(self.current_drawing)
            self.current_drawing = None

    def save_drawing(self):
        """Save drawing to file"""
        filename = filedialog.asksaveasfilename(
            defaultextension=".json",
            filetypes=[("JSON files", "*.json"), ("All files", "*.*")]
        )
        if filename:
            import json
            data = {
                'objects': self.drawing_objects,
                'canvas_size': self.canvas_size_var.get()
            }
            with open(filename, 'w') as f:
                json.dump(data, f)
            print(f"Drawing saved to {filename}")

    def load_drawing(self):
        """Load drawing from file"""
        filename = filedialog.askopenfilename(
            filetypes=[("JSON files", "*.json"), ("All files", "*.*")]
        )
        if filename:
            import json
            with open(filename, 'r') as f:
                data = json.load(f)

            # Clear current drawing
            self.clear_drawing_canvas()

            # Load canvas size
            self.canvas_size_var.set(data.get('canvas_size', '100'))

            # Recreate objects
            for obj_data in data['objects']:
                # Recreate the visual object
                if obj_data['type'] == 'line':
                    obj_id = self.drawing_canvas.create_line(
                        obj_data['coords'], fill="black", width=2)
                elif obj_data['type'] == 'rectangle':
                    obj_id = self.drawing_canvas.create_rectangle(
                        obj_data['coords'], outline="black", width=2, fill="")
                elif obj_data['type'] == 'circle':
                    obj_id = self.drawing_canvas.create_oval(
                        obj_data['coords'], outline="black", width=2, fill="")
                elif obj_data['type'] == 'polygon':
                    obj_id = self.drawing_canvas.create_polygon(
                        obj_data['coords'], outline="black", width=2, fill="")

                # Store in drawing_objects
                self.drawing_objects.append(obj_data)

            print(f"Drawing loaded from {filename}")


    ########################
    # GUI functions
    ########################

    # **NEW METHOD: Update position tracking**
    def update_position(self, new_x, new_y, laser_power=None):
        """Update internal position tracking and plot"""
        self.xpos = new_x
        self.ypos = new_y

        # Add to history
        self.move_history_x.append(new_x)
        self.move_history_y.append(new_y)
        self.move_history_laser.append(laser_power if laser_power is not None else 0)

        # Update position label
        self.position_label.config(text=f"Current Position: X={self.xpos:.2f}, Y={self.ypos:.2f}")

        # Update plot
        self.update_move_plot()


    def update_move_plot(self):
        """Update the matplotlib plot showing movement history"""
        self.ax.clear()

        # Plot movements with different colors based on laser state
        for i in range(1, len(self.move_history_x)):
            x_vals = [self.move_history_x[i - 1], self.move_history_x[i]]
            y_vals = [self.move_history_y[i - 1], self.move_history_y[i]]

            # Color based on laser power
            if self.move_history_laser[i] > 0:
                color = 'red'
                linewidth = 2
            else:
                color = 'blue'
                linewidth = 1

            self.ax.plot(x_vals, y_vals, color=color, linewidth=linewidth)

        # Mark current position
        if len(self.move_history_x) > 0:
            self.ax.scatter(self.xpos, self.ypos, color='green', s=100, marker='o', label='Current Position')

        # Mark origin
        self.ax.scatter(0, 0, color='black', s=100, marker='x', label='Origin')

        self.ax.set_xlabel('X Position (mm)')
        self.ax.set_ylabel('Y Position (mm)')
        self.ax.set_title('Movement History (Red=Laser On, Blue=Laser Off)')
        self.ax.grid(True, alpha=0.3)
        self.ax.set_aspect('equal')
        self.ax.legend()

        # Set axis limits with some padding
        if len(self.move_history_x) > 1:
            x_min, x_max = min(self.move_history_x), max(self.move_history_x)
            y_min, y_max = min(self.move_history_y), max(self.move_history_y)
            padding = 10
            self.ax.set_xlim(x_min - padding, x_max + padding)
            self.ax.set_ylim(y_min - padding, y_max + padding)

        self.canvas.draw()


    def clear_move_history(self):
        """Clear the movement history and reset plot"""
        self.move_history_x = [self.xpos]
        self.move_history_y = [self.ypos]
        self.move_history_laser = [0]
        self.update_move_plot()

    def start_webcam(self):
        self.cap = cv2.VideoCapture(0)
        if self.cap.isOpened():
            self.webcam_running = True
            self.webcam_thread = threading.Thread(target=self.update_webcam_thread, daemon=True)
            self.webcam_thread.start()
        else:
            print("Webcam failed to open.")

    def show_size_box(self):
        try:
            box_width_mm = float(self.box_width_entry.get())
            box_height_mm = float(self.box_height_entry.get())
            # Redraw the current image and overlay the box
            if self.framecrop is not None:
                self.update_canvas(self.framecrop)
                self.draw_size_box(box_width_mm, box_height_mm)
            else:
                print("No image loaded to display box.")
        except Exception as e:
            print(f"Error: {e}")

    def generate_path_from_image(self):
        # open file and ask for interest
        file_path = filedialog.askopenfilename(filetypes=[("image files", "*.jpg"), ("all files", "*.*")])
        frame = cv2.imread(file_path)

        # Create a resizable window
        cv2.namedWindow("Select region of interest", cv2.WINDOW_NORMAL)

        # Get image dimensions
        h, w = frame.shape[:2]

        # Set window size to fit image, but not larger than screen
        screen_width = self.root.winfo_screenwidth()
        screen_height = self.root.winfo_screenheight()

        # Calculate appropriate window size (90% of screen max)
        max_width = int(screen_width * 0.9)
        max_height = int(screen_height * 0.9)

        if w > max_width or h > max_height:
            # Scale down to fit
            scale = min(max_width / w, max_height / h)
            window_width = int(w * scale)
            window_height = int(h * scale)
        else:
            # Use original size
            window_width = w
            window_height = h

        # Resize window
        cv2.resizeWindow("Select region of interest", window_width, window_height)

        # Center the window on screen
        x = (screen_width - window_width) // 2
        y = (screen_height - window_height) // 2
        cv2.moveWindow("Select region of interest", x, y)

        # Select ROI
        r = cv2.selectROI("Select region of interest", frame, False)
        cv2.destroyWindow("Select region of interest")

        self.framecrop = frame[int(r[1]):int(r[1] + r[3]), int(r[0]):int(r[0] + r[2])]
        self.frameshape = np.shape(self.framecrop)
        imagedata = self.framecrop[:, :, 1]
        thresh = cv2.threshold(imagedata, np.mean(imagedata) - 25, 255, cv2.THRESH_BINARY)[1]
        thresh = cv2.dilate(thresh, np.ones((3, 3), np.uint8), iterations=1)
        thresh = cv2.erode(thresh, np.ones((3, 3), np.uint8), iterations=1)

        # Keep RETR_LIST to get all contours
        contours = cv2.findContours(thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_TC89_L1)[0]

        # Filter out the largest contour if it's close to the image boundary
        filtered_contours = []
        image_height, image_width = self.frameshape[0], self.frameshape[1]

        for contour in contours:
            # Get bounding box of contour
            x, y, w, h = cv2.boundingRect(contour)

            # Check if this contour touches the image edges (within 5 pixels)
            touches_edge = (x <= 5 or y <= 5 or
                            (x + w) >= image_width - 5 or
                            (y + h) >= image_height - 5)

            # Check if it's very large
            contour_area = cv2.contourArea(contour)
            image_area = image_height * image_width
            is_very_large = contour_area > 0.8 * image_area

            # Keep contour if it doesn't touch edges OR isn't very large
            if not (touches_edge and is_very_large):
                filtered_contours.append(contour)

        # STORE THE CONTOURS WITHOUT SCALING
        self.raw_contours = sorted(filtered_contours, key=cv2.contourArea)

        self.update_canvas(self.framecrop)
        for c in self.raw_contours:
            cv2.drawContours(self.framecrop, c, -1, (255, 0, 0), thickness=2)

        # Apply initial scaling
        self.generate_contour_data()

###
    ###
    # svg stuff
    ###

    def generate_path_from_svg(self):
        """Load and process SVG file"""
        file_path = filedialog.askopenfilename(
            title="Select SVG file",
            filetypes=[("SVG files", "*.svg"), ("All files", "*.*")]
        )

        if not file_path:
            return

        try:
            # Parse SVG file
            tree = ET.parse(file_path)
            root = tree.getroot()

            # Get SVG dimensions
            width = self._get_svg_dimension(root.get('width', '100'))
            height = self._get_svg_dimension(root.get('height', '100'))

            # Get viewBox if present
            viewBox = root.get('viewBox')
            if viewBox:
                vb_parts = viewBox.split()
                if len(vb_parts) == 4:
                    vb_width = float(vb_parts[2])
                    vb_height = float(vb_parts[3])
                    # Use viewBox dimensions if available
                    width = vb_width
                    height = vb_height

            print(f"SVG dimensions: {width} x {height}")

            # Clear existing data
            self.clear_data()

            # Process all path elements
            paths = []
            for element in root.iter():
                if element.tag.endswith('path'):
                    d = element.get('d')
                    if d:
                        paths.append(d)
                elif element.tag.endswith('rect'):
                    # Convert rectangle to path
                    x = float(element.get('x', 0))
                    y = float(element.get('y', 0))
                    w = float(element.get('width', 0))
                    h = float(element.get('height', 0))
                    rect_path = f"M {x},{y} L {x + w},{y} L {x + w},{y + h} L {x},{y + h} Z"
                    paths.append(rect_path)
                elif element.tag.endswith('circle'):
                    # Convert circle to path
                    cx = float(element.get('cx', 0))
                    cy = float(element.get('cy', 0))
                    r = float(element.get('r', 0))
                    # Approximate circle with bezier curves
                    circle_path = self._circle_to_path(cx, cy, r)
                    paths.append(circle_path)
                elif element.tag.endswith('ellipse'):
                    # Convert ellipse to path
                    cx = float(element.get('cx', 0))
                    cy = float(element.get('cy', 0))
                    rx = float(element.get('rx', 0))
                    ry = float(element.get('ry', 0))
                    ellipse_path = self._ellipse_to_path(cx, cy, rx, ry)
                    paths.append(ellipse_path)
                elif element.tag.endswith('line'):
                    # Convert line to path
                    x1 = float(element.get('x1', 0))
                    y1 = float(element.get('y1', 0))
                    x2 = float(element.get('x2', 0))
                    y2 = float(element.get('y2', 0))
                    line_path = f"M {x1},{y1} L {x2},{y2}"
                    paths.append(line_path)
                elif element.tag.endswith('polyline') or element.tag.endswith('polygon'):
                    # Convert polyline/polygon to path
                    points = element.get('points', '')
                    if points:
                        poly_path = self._polyline_to_path(points, element.tag.endswith('polygon'))
                        paths.append(poly_path)

            # Process all paths
            for path_data in paths:
                self._process_svg_path(path_data, width, height)

            # Update the canvas with the processed paths
            self._draw_svg_preview()

            print(f"Loaded {len(paths)} paths from SVG")
            print(f"Total points: {len(self.datax)}")

        except Exception as e:
            tk.messagebox.showerror("Error", f"Failed to load SVG file: {str(e)}")
            print(f"Error loading SVG: {e}")

    def _get_svg_dimension(self, dim_str):
        """Extract numeric value from SVG dimension string"""
        if not dim_str:
            return 100
        # Remove units (px, mm, etc.)
        match = re.match(r'([\d.]+)', dim_str)
        if match:
            return float(match.group(1))
        return 100

    def _circle_to_path(self, cx, cy, r):
        """Convert circle to SVG path using bezier curves"""
        # Magic number for circle approximation with cubic beziers
        k = 0.552284749831

        path = f"M {cx + r},{cy} "
        path += f"C {cx + r},{cy + k * r} {cx + k * r},{cy + r} {cx},{cy + r} "
        path += f"C {cx - k * r},{cy + r} {cx - r},{cy + k * r} {cx - r},{cy} "
        path += f"C {cx - r},{cy - k * r} {cx - k * r},{cy - r} {cx},{cy - r} "
        path += f"C {cx + k * r},{cy - r} {cx + r},{cy - k * r} {cx + r},{cy} Z"

        return path

    def _ellipse_to_path(self, cx, cy, rx, ry):
        """Convert ellipse to SVG path using bezier curves"""
        k = 0.552284749831

        path = f"M {cx + rx},{cy} "
        path += f"C {cx + rx},{cy + k * ry} {cx + k * rx},{cy + ry} {cx},{cy + ry} "
        path += f"C {cx - k * rx},{cy + ry} {cx - rx},{cy + k * ry} {cx - rx},{cy} "
        path += f"C {cx - rx},{cy - k * ry} {cx - k * rx},{cy - ry} {cx},{cy - ry} "
        path += f"C {cx + k * rx},{cy - ry} {cx + rx},{cy - k * ry} {cx + rx},{cy} Z"

        return path

    def _polyline_to_path(self, points_str, close=False):
        """Convert polyline/polygon points to SVG path"""
        points = re.findall(r'([\d.-]+)[,\s]+([\d.-]+)', points_str)
        if not points:
            return ""

        path = f"M {points[0][0]},{points[0][1]}"
        for x, y in points[1:]:
            path += f" L {x},{y}"

        if close:
            path += " Z"

        return path

    def _process_svg_path(self, path_data, svg_width, svg_height):
        """Process SVG path data and convert to machine coordinates"""
        try:
            path = parse_path(path_data)

            # Sample points along the path
            current_x, current_y = 0, 0
            laser_on = False

            for segment in path:
                if isinstance(segment, Move):
                    # Move without drawing
                    current_x = segment.end.real
                    current_y = segment.end.imag

                    # Convert SVG coordinates to machine coordinates
                    machine_x, machine_y = self._svg_to_machine_coords(
                        current_x, current_y, svg_width, svg_height
                    )

                    self.datax.append(machine_x)
                    self.datay.append(machine_y)
                    self.laserstate.append(0)  # Laser off for moves

                elif isinstance(segment, Line):
                    # Draw line
                    # First move to start point with laser off if needed
                    if not laser_on or (current_x != segment.start.real or current_y != segment.start.imag):
                        start_x, start_y = self._svg_to_machine_coords(
                            segment.start.real, segment.start.imag, svg_width, svg_height
                        )
                        self.datax.append(start_x)
                        self.datay.append(start_y)
                        self.laserstate.append(0)

                    # Then draw to end point with laser on
                    end_x, end_y = self._svg_to_machine_coords(
                        segment.end.real, segment.end.imag, svg_width, svg_height
                    )
                    self.datax.append(end_x)
                    self.datay.append(end_y)
                    self.laserstate.append(1)  # Laser on for drawing

                    current_x = segment.end.real
                    current_y = segment.end.imag
                    laser_on = True

                elif isinstance(segment, (CubicBezier, QuadraticBezier, Arc)):
                    # Sample curves at regular intervals
                    num_samples = 20  # Adjust for smoothness

                    for i in range(num_samples + 1):
                        t = i / num_samples
                        point = segment.point(t)

                        machine_x, machine_y = self._svg_to_machine_coords(
                            point.real, point.imag, svg_width, svg_height
                        )

                        # First point of curve might need laser off
                        if i == 0 and not laser_on:
                            self.datax.append(machine_x)
                            self.datay.append(machine_y)
                            self.laserstate.append(0)
                        else:
                            self.datax.append(machine_x)
                            self.datay.append(machine_y)
                            self.laserstate.append(1)

                        laser_on = True

                    current_x = segment.end.real
                    current_y = segment.end.imag

                elif isinstance(segment, Close):
                    # Close path - draw line back to start
                    # This is handled by the Z command in SVG
                    laser_on = False

        except Exception as e:
            print(f"Error processing SVG path: {e}")

    def _svg_to_machine_coords(self, svg_x, svg_y, svg_width, svg_height):
        """Convert SVG coordinates to machine coordinates"""
        # Apply scaling factor
        scaling = float(self.scalingentry.get())

        # SVG origin is top-left, machine origin is center
        # Center the design
        machine_x = (svg_x - svg_width / 2) * scaling
        machine_y = (svg_height / 2 - svg_y) * scaling  # Flip Y axis

        return machine_x, machine_y

    def _draw_svg_preview(self):
        """Draw the loaded SVG paths on the canvas"""
        if not self.datax:
            return

        # Clear canvas
        self.imagecanvas.delete("all")

        # Get canvas dimensions
        canvas_width = self.imagecanvas.winfo_width()
        canvas_height = self.imagecanvas.winfo_height()

        if canvas_width <= 1:  # Canvas not yet rendered
            canvas_width = 300
            canvas_height = 200

        # Find bounds of the path
        min_x = min(self.datax)
        max_x = max(self.datax)
        min_y = min(self.datay)
        max_y = max(self.datay)

        # Calculate scale to fit canvas
        path_width = max_x - min_x
        path_height = max_y - min_y

        if path_width == 0 or path_height == 0:
            return

        # Add padding
        padding = 20
        scale_x = (canvas_width - 2 * padding) / path_width
        scale_y = (canvas_height - 2 * padding) / path_height
        scale = min(scale_x, scale_y)

        # Calculate offset to center
        offset_x = canvas_width / 2 - (min_x + max_x) / 2 * scale
        offset_y = canvas_height / 2 + (min_y + max_y) / 2 * scale  # Note: + because canvas Y is inverted

        # Draw the paths
        for i in range(1, len(self.datax)):
            x1 = self.datax[i - 1] * scale + offset_x
            y1 = -self.datay[i - 1] * scale + offset_y  # Negative because canvas Y is inverted
            x2 = self.datax[i] * scale + offset_x
            y2 = -self.datay[i] * scale + offset_y

            # Draw based on laser state
            if self.laserstate[i] == 1:
                # Drawing move - blue line
                self.imagecanvas.create_line(x1, y1, x2, y2, fill="blue", width=2)
            else:
                # Travel move - gray dashed line
                self.imagecanvas.create_line(x1, y1, x2, y2, fill="gray", width=1, dash=(2, 2))

        # Draw bounds
        bounds_x1 = min_x * scale + offset_x
        bounds_y1 = -min_y * scale + offset_y
        bounds_x2 = max_x * scale + offset_x
        bounds_y2 = -max_y * scale + offset_y

        self.imagecanvas.create_rectangle(bounds_x1, bounds_y2, bounds_x2, bounds_y1,
                                          outline="red", width=1, dash=(5, 5))

        # Update info
        info_text = f"Size: {path_width:.1f} x {path_height:.1f} mm\nPoints: {len(self.datax)}"
        self.imagecanvas.create_text(10, 10, text=info_text, anchor="nw", fill="black")

    def export_drawing_as_svg(self):
        """Export current drawing/path as SVG file"""
        if not self.datax:
            tk.messagebox.showwarning("No Data", "No path data to export!")
            return

        file_path = filedialog.asksaveasfilename(
            defaultextension=".svg",
            filetypes=[("SVG files", "*.svg"), ("All files", "*.*")]
        )

        if not file_path:
            return

        try:
            # Find bounds
            min_x = min(self.datax)
            max_x = max(self.datax)
            min_y = min(self.datay)
            max_y = max(self.datay)

            width = max_x - min_x
            height = max_y - min_y

            # Create SVG content
            svg_content = f'''<?xml version="1.0" encoding="UTF-8"?>
    <svg width="{width:.2f}mm" height="{height:.2f}mm" 
         viewBox="{min_x:.2f} {min_y:.2f} {width:.2f} {height:.2f}"
         xmlns="http://www.w3.org/2000/svg">
    '''

            # Build path data
            path_data = ""
            for i in range(len(self.datax)):
                if i == 0 or self.laserstate[i] == 0:
                    # Move to position
                    path_data += f"M {self.datax[i]:.2f},{-self.datay[i]:.2f} "
                else:
                    # Line to position
                    path_data += f"L {self.datax[i]:.2f},{-self.datay[i]:.2f} "

            svg_content += f'  <path d="{path_data}" stroke="black" fill="none" stroke-width="0.1"/>\n'
            svg_content += '</svg>'

            # Write to file
            with open(file_path, 'w') as f:
                f.write(svg_content)

            print(f"Exported SVG to {file_path}")
            tk.messagebox.showinfo("Export Complete", f"SVG saved to {file_path}")

        except Exception as e:
            tk.messagebox.showerror("Export Error", f"Failed to export SVG: {str(e)}")

    # Add method to optimize SVG paths
    def optimize_svg_paths(self):
        """Optimize path order to minimize travel moves"""
        if not self.datax:
            return

        print("Optimizing paths...")

        # Group continuous paths (where laser is on)
        paths = []
        current_path = []

        for i in range(len(self.datax)):
            if self.laserstate[i] == 1:
                current_path.append(i)
            else:
                if current_path:
                    paths.append(current_path)
                    current_path = []

        if current_path:
            paths.append(current_path)

        print(f"Found {len(paths)} separate paths")

        # Optimize path order using nearest neighbor
        if len(paths) > 1:
            optimized_order = [0]  # Start with first path
            remaining = list(range(1, len(paths)))

            while remaining:
                current_path_idx = optimized_order[-1]
                current_end = paths[current_path_idx][-1]
                current_x = self.datax[current_end]
                current_y = self.datay[current_end]

                # Find nearest path start
                min_dist = float('inf')
                nearest_idx = None

                for idx in remaining:
                    path_start = paths[idx][0]
                    start_x = self.datax[path_start]
                    start_y = self.datay[path_start]

                    dist = ((start_x - current_x) ** 2 + (start_y - current_y) ** 2) ** 0.5

                    if dist < min_dist:
                        min_dist = dist
                        nearest_idx = idx

                optimized_order.append(nearest_idx)
                remaining.remove(nearest_idx)

            # Rebuild arrays in optimized order
            new_datax = []
            new_datay = []
            new_laserstate = []

            for path_idx in optimized_order:
                path = paths[path_idx]

                # Add move to start of path
                if new_datax:
                    new_datax.append(self.datax[path[0]])
                    new_datay.append(self.datay[path[0]])
                    new_laserstate.append(0)

                # Add path points
                for point_idx in path:
                    new_datax.append(self.datax[point_idx])
                    new_datay.append(self.datay[point_idx])
                    new_laserstate.append(self.laserstate[point_idx])

            # Update arrays
            self.datax = new_datax
            self.datay = new_datay
            self.laserstate = new_laserstate

            print("Path optimization complete")

            # Redraw preview
            self._draw_svg_preview()

    ###
    ## dxf stuff
    ##


    # Add these methods to your XYStageDriver class

    def generate_path_from_dxf(self):
        """Load and process DXF file"""
        file_path = filedialog.askopenfilename(
            title="Select DXF file",
            filetypes=[("DXF files", "*.dxf"), ("All files", "*.*")]
        )

        if not file_path:
            return

        try:
            # Try to load DXF file, use recover for damaged files
            try:
                doc = ezdxf.readfile(file_path)
            except:
                doc, auditor = recover.readfile(file_path)
                if auditor.has_errors:
                    print("DXF file has errors, attempting to recover...")

            # Get the modelspace
            msp = doc.modelspace()

            # Clear existing data
            self.clear_data()

            # Get DXF bounds
            bounds = self._get_dxf_bounds(msp)
            if bounds:
                min_x, min_y, max_x, max_y = bounds
                width = max_x - min_x
                height = max_y - min_y
                center_x = (min_x + max_x) / 2
                center_y = (min_y + max_y) / 2
                print(f"DXF bounds: {width:.2f} x {height:.2f} units, center: ({center_x:.2f}, {center_y:.2f})")
            else:
                center_x = center_y = 0
                width = height = 100

            # Process all entities
            entity_count = 0
            for entity in msp:
                if entity.dxftype() == 'LINE':
                    self._process_dxf_line(entity, center_x, center_y)
                    entity_count += 1
                elif entity.dxftype() == 'POLYLINE':
                    self._process_dxf_polyline(entity, center_x, center_y)
                    entity_count += 1
                elif entity.dxftype() == 'LWPOLYLINE':
                    self._process_dxf_lwpolyline(entity, center_x, center_y)
                    entity_count += 1
                elif entity.dxftype() == 'CIRCLE':
                    self._process_dxf_circle(entity, center_x, center_y)
                    entity_count += 1
                elif entity.dxftype() == 'ARC':
                    self._process_dxf_arc(entity, center_x, center_y)
                    entity_count += 1
                elif entity.dxftype() == 'ELLIPSE':
                    self._process_dxf_ellipse(entity, center_x, center_y)
                    entity_count += 1
                elif entity.dxftype() == 'SPLINE':
                    self._process_dxf_spline(entity, center_x, center_y)
                    entity_count += 1
                elif entity.dxftype() == 'INSERT':
                    # Block reference - need to process block entities
                    self._process_dxf_insert(entity, msp, center_x, center_y)
                    entity_count += 1

            # Optimize path order if we have data
            if self.datax:
                print(f"Loaded {entity_count} entities, {len(self.datax)} points")
                if tk.messagebox.askyesno("Optimize", "Optimize path order to minimize travel?"):
                    self.optimize_dxf_paths()

            # Update the canvas preview
            self._draw_dxf_preview()

        except Exception as e:
            tk.messagebox.showerror("Error", f"Failed to load DXF file: {str(e)}")
            print(f"Error loading DXF: {e}")

    def _get_dxf_bounds(self, modelspace):
        """Calculate bounds of all entities in DXF"""
        min_x = min_y = float('inf')
        max_x = max_y = float('-inf')
        has_entities = False

        for entity in modelspace:
            try:
                if hasattr(entity, 'dxf'):
                    if entity.dxftype() == 'LINE':
                        start = entity.dxf.start
                        end = entity.dxf.end
                        min_x = min(min_x, start.x, end.x)
                        max_x = max(max_x, start.x, end.x)
                        min_y = min(min_y, start.y, end.y)
                        max_y = max(max_y, start.y, end.y)
                        has_entities = True
                    elif entity.dxftype() == 'CIRCLE':
                        center = entity.dxf.center
                        radius = entity.dxf.radius
                        min_x = min(min_x, center.x - radius)
                        max_x = max(max_x, center.x + radius)
                        min_y = min(min_y, center.y - radius)
                        max_y = max(max_y, center.y + radius)
                        has_entities = True
                    elif entity.dxftype() in ['POLYLINE', 'LWPOLYLINE']:
                        for point in entity.points():
                            min_x = min(min_x, point[0])
                            max_x = max(max_x, point[0])
                            min_y = min(min_y, point[1])
                            max_y = max(max_y, point[1])
                            has_entities = True
            except:
                pass

        if has_entities:
            return min_x, min_y, max_x, max_y
        return None

    def _dxf_to_machine_coords(self, x, y, center_x, center_y):
        """Convert DXF coordinates to machine coordinates"""
        # Apply scaling factor
        scaling = float(self.scalingentry.get())

        # Center the design
        machine_x = (x - center_x) * scaling
        machine_y = (y - center_y) * scaling

        return machine_x, machine_y

    def _process_dxf_line(self, entity, center_x, center_y):
        """Process DXF LINE entity"""
        start = entity.dxf.start
        end = entity.dxf.end

        # Move to start (laser off)
        x1, y1 = self._dxf_to_machine_coords(start.x, start.y, center_x, center_y)
        self.datax.append(x1)
        self.datay.append(y1)
        self.laserstate.append(0)

        # Draw to end (laser on)
        x2, y2 = self._dxf_to_machine_coords(end.x, end.y, center_x, center_y)
        self.datax.append(x2)
        self.datay.append(y2)
        self.laserstate.append(1)

    def _process_dxf_polyline(self, entity, center_x, center_y):
        """Process DXF POLYLINE entity"""
        points = list(entity.points())
        if not points:
            return

        # Move to first point (laser off)
        x, y = self._dxf_to_machine_coords(points[0][0], points[0][1], center_x, center_y)
        self.datax.append(x)
        self.datay.append(y)
        self.laserstate.append(0)

        # Draw remaining points (laser on)
        for point in points[1:]:
            x, y = self._dxf_to_machine_coords(point[0], point[1], center_x, center_y)
            self.datax.append(x)
            self.datay.append(y)
            self.laserstate.append(1)

        # Close polyline if needed
        if entity.is_closed:
            x, y = self._dxf_to_machine_coords(points[0][0], points[0][1], center_x, center_y)
            self.datax.append(x)
            self.datay.append(y)
            self.laserstate.append(1)

    def _process_dxf_lwpolyline(self, entity, center_x, center_y):
        """Process DXF LWPOLYLINE entity"""
        points = list(entity.get_points())
        if not points:
            return

        # Move to first point (laser off)
        x, y = self._dxf_to_machine_coords(points[0][0], points[0][1], center_x, center_y)
        self.datax.append(x)
        self.datay.append(y)
        self.laserstate.append(0)

        # Process points with bulge for arcs
        for i in range(1, len(points)):
            if len(points[i - 1]) > 4 and points[i - 1][4] != 0:  # Has bulge
                # Create arc between points
                self._add_bulge_arc(points[i - 1], points[i], center_x, center_y)
            else:
                # Straight line
                x, y = self._dxf_to_machine_coords(points[i][0], points[i][1], center_x, center_y)
                self.datax.append(x)
                self.datay.append(y)
                self.laserstate.append(1)

        # Close polyline if needed
        if entity.closed:
            if len(points[-1]) > 4 and points[-1][4] != 0:  # Last point has bulge
                self._add_bulge_arc(points[-1], points[0], center_x, center_y)
            else:
                x, y = self._dxf_to_machine_coords(points[0][0], points[0][1], center_x, center_y)
                self.datax.append(x)
                self.datay.append(y)
                self.laserstate.append(1)

    def _add_bulge_arc(self, start_point, end_point, center_x, center_y):
        """Add arc defined by bulge between two points"""
        x1, y1 = start_point[0], start_point[1]
        x2, y2 = end_point[0], end_point[1]
        bulge = start_point[4]

        # Calculate arc parameters from bulge
        angle = 4 * math.atan(bulge)
        dx = x2 - x1
        dy = y2 - y1
        chord = math.sqrt(dx * dx + dy * dy)
        radius = chord / (2 * math.sin(angle / 2))

        # Sample arc
        steps = max(10, int(abs(angle) * 10))
        for i in range(1, steps + 1):
            t = i / steps
            # Interpolate along arc
            # This is simplified - proper bulge arc calculation would be more complex
            x = x1 + dx * t
            y = y1 + dy * t

            mx, my = self._dxf_to_machine_coords(x, y, center_x, center_y)
            self.datax.append(mx)
            self.datay.append(my)
            self.laserstate.append(1)

    def _process_dxf_circle(self, entity, center_x, center_y):
        """Process DXF CIRCLE entity"""
        cx = entity.dxf.center.x
        cy = entity.dxf.center.y
        radius = entity.dxf.radius

        # Sample circle
        steps = max(32, int(radius * 2))  # More steps for larger circles

        for i in range(steps + 1):
            angle = (i / steps) * 2 * math.pi
            x = cx + radius * math.cos(angle)
            y = cy + radius * math.sin(angle)

            mx, my = self._dxf_to_machine_coords(x, y, center_x, center_y)
            self.datax.append(mx)
            self.datay.append(my)
            self.laserstate.append(0 if i == 0 else 1)  # First point laser off

    def _process_dxf_arc(self, entity, center_x, center_y):
        """Process DXF ARC entity"""
        cx = entity.dxf.center.x
        cy = entity.dxf.center.y
        radius = entity.dxf.radius
        start_angle = math.radians(entity.dxf.start_angle)
        end_angle = math.radians(entity.dxf.end_angle)

        # Handle angle wrap
        if end_angle < start_angle:
            end_angle += 2 * math.pi

        # Sample arc
        angle_span = end_angle - start_angle
        steps = max(16, int(abs(angle_span) * radius / math.pi))

        for i in range(steps + 1):
            angle = start_angle + (i / steps) * angle_span
            x = cx + radius * math.cos(angle)
            y = cy + radius * math.sin(angle)

            mx, my = self._dxf_to_machine_coords(x, y, center_x, center_y)
            self.datax.append(mx)
            self.datay.append(my)
            self.laserstate.append(0 if i == 0 else 1)  # First point laser off

    def _process_dxf_ellipse(self, entity, center_x, center_y):
        """Process DXF ELLIPSE entity"""
        cx = entity.dxf.center.x
        cy = entity.dxf.center.y
        major_axis = entity.dxf.major_axis
        ratio = entity.dxf.ratio
        start_param = entity.dxf.start_param
        end_param = entity.dxf.end_param

        # Calculate ellipse parameters
        major_length = math.sqrt(major_axis.x ** 2 + major_axis.y ** 2)
        minor_length = major_length * ratio
        rotation = math.atan2(major_axis.y, major_axis.x)

        # Sample ellipse
        steps = max(32, int((major_length + minor_length) * 2))

        for i in range(steps + 1):
            t = start_param + (i / steps) * (end_param - start_param)

            # Parametric ellipse
            x_local = major_length * math.cos(t)
            y_local = minor_length * math.sin(t)

            # Rotate and translate
            x = cx + x_local * math.cos(rotation) - y_local * math.sin(rotation)
            y = cy + x_local * math.sin(rotation) + y_local * math.cos(rotation)

            mx, my = self._dxf_to_machine_coords(x, y, center_x, center_y)
            self.datax.append(mx)
            self.datay.append(my)
            self.laserstate.append(0 if i == 0 else 1)

    def _process_dxf_spline(self, entity, center_x, center_y):
        """Process DXF SPLINE entity"""
        try:
            # Get control points
            control_points = list(entity.control_points)
            if len(control_points) < 2:
                return

            # Simple linear interpolation between control points
            # For better results, implement proper B-spline evaluation
            total_points = len(control_points) * 10

            for i in range(total_points):
                t = i / (total_points - 1)
                idx = int(t * (len(control_points) - 1))
                local_t = (t * (len(control_points) - 1)) - idx

                if idx < len(control_points) - 1:
                    x = control_points[idx].x * (1 - local_t) + control_points[idx + 1].x * local_t
                    y = control_points[idx].y * (1 - local_t) + control_points[idx + 1].y * local_t
                else:
                    x = control_points[-1].x
                    y = control_points[-1].y

                mx, my = self._dxf_to_machine_coords(x, y, center_x, center_y)
                self.datax.append(mx)
                self.datay.append(my)
                self.laserstate.append(0 if i == 0 else 1)

        except Exception as e:
            print(f"Error processing spline: {e}")

    def _process_dxf_insert(self, entity, modelspace, center_x, center_y):
        """Process DXF INSERT (block reference) entity"""
        try:
            block = entity.dxf.name
            insert_point = entity.dxf.insert
            x_scale = entity.dxf.xscale if hasattr(entity.dxf, 'xscale') else 1
            y_scale = entity.dxf.yscale if hasattr(entity.dxf, 'yscale') else 1
            rotation = entity.dxf.rotation if hasattr(entity.dxf, 'rotation') else 0

            # Get block definition
            block_def = modelspace.doc.blocks.get(block)
            if not block_def:
                return

            # Process entities in block
            for block_entity in block_def:
                # Transform block entity coordinates
                # This is simplified - proper transformation would include rotation
                if block_entity.dxftype() == 'LINE':
                    start = block_entity.dxf.start
                    end = block_entity.dxf.end

                    # Apply transformation
                    x1 = insert_point.x + start.x * x_scale
                    y1 = insert_point.y + start.y * y_scale
                    x2 = insert_point.x + end.x * x_scale
                    y2 = insert_point.y + end.y * y_scale

                    # Convert to machine coordinates
                    mx1, my1 = self._dxf_to_machine_coords(x1, y1, center_x, center_y)
                    mx2, my2 = self._dxf_to_machine_coords(x2, y2, center_x, center_y)

                    self.datax.extend([mx1, mx2])
                    self.datay.extend([my1, my2])
                    self.laserstate.extend([0, 1])

        except Exception as e:
            print(f"Error processing block insert: {e}")

    def _draw_dxf_preview(self):
        """Draw the loaded DXF paths on the canvas"""
        if not self.datax:
            return

        # Clear canvas
        self.imagecanvas.delete("all")

        # Get canvas dimensions
        canvas_width = self.imagecanvas.winfo_width()
        canvas_height = self.imagecanvas.winfo_height()

        if canvas_width <= 1:  # Canvas not yet rendered
            canvas_width = 300
            canvas_height = 200

        # Find bounds of the path
        min_x = min(self.datax)
        max_x = max(self.datax)
        min_y = min(self.datay)
        max_y = max(self.datay)

        # Calculate scale to fit canvas
        path_width = max_x - min_x
        path_height = max_y - min_y

        if path_width == 0 or path_height == 0:
            return

        # Add padding
        padding = 20
        scale_x = (canvas_width - 2 * padding) / path_width
        scale_y = (canvas_height - 2 * padding) / path_height
        scale = min(scale_x, scale_y)

        # Calculate offset to center
        offset_x = canvas_width / 2 - (min_x + max_x) / 2 * scale
        offset_y = canvas_height / 2 + (min_y + max_y) / 2 * scale

        # Draw the paths
        for i in range(1, len(self.datax)):
            x1 = self.datax[i - 1] * scale + offset_x
            y1 = -self.datay[i - 1] * scale + offset_y
            x2 = self.datax[i] * scale + offset_x
            y2 = -self.datay[i] * scale + offset_y

            # Draw based on laser state
            if self.laserstate[i] == 1:
                # Drawing move - blue line
                self.imagecanvas.create_line(x1, y1, x2, y2, fill="blue", width=2)
            else:
                # Travel move - gray dashed line
                self.imagecanvas.create_line(x1, y1, x2, y2, fill="gray", width=1, dash=(2, 2))

        # Draw bounds
        bounds_x1 = min_x * scale + offset_x
        bounds_y1 = -min_y * scale + offset_y
        bounds_x2 = max_x * scale + offset_x
        bounds_y2 = -max_y * scale + offset_y

        self.imagecanvas.create_rectangle(bounds_x1, bounds_y2, bounds_x2, bounds_y1,
                                          outline="red", width=1, dash=(5, 5))

        # Update info
        info_text = f"Size: {path_width:.1f} x {path_height:.1f} mm\nPoints: {len(self.datax)}"
        self.imagecanvas.create_text(10, 10, text=info_text, anchor="nw", fill="black")

    def optimize_dxf_paths(self):
        """Optimize DXF path order to minimize travel moves"""
        if not self.datax:
            return

        print("Optimizing DXF paths...")

        # Group continuous paths (where laser is on)
        paths = []
        current_path = []

        for i in range(len(self.datax)):
            if self.laserstate[i] == 1:
                current_path.append(i)
            else:
                if current_path:
                    paths.append(current_path)
                    current_path = []

        if current_path:
            paths.append(current_path)

        print(f"Found {len(paths)} separate paths")

        # Optimize path order using nearest neighbor
        if len(paths) > 1:
            optimized_order = [0]  # Start with first path
            remaining = list(range(1, len(paths)))

            while remaining:
                current_path_idx = optimized_order[-1]
                current_end = paths[current_path_idx][-1]
                current_x = self.datax[current_end]
                current_y = self.datay[current_end]

                # Find nearest path start
                min_dist = float('inf')
                nearest_idx = None

                for idx in remaining:
                    path_start = paths[idx][0]
                    start_x = self.datax[path_start]
                    start_y = self.datay[path_start]

                    dist = ((start_x - current_x) ** 2 + (start_y - current_y) ** 2) ** 0.5

                    if dist < min_dist:
                        min_dist = dist
                        nearest_idx = idx

                optimized_order.append(nearest_idx)
                remaining.remove(nearest_idx)

            # Rebuild arrays in optimized order
            new_datax = []
            new_datay = []
            new_laserstate = []

            for path_idx in optimized_order:
                path = paths[path_idx]

                # Add move to start of path
                if new_datax:
                    new_datax.append(self.datax[path[0]])
                    new_datay.append(self.datay[path[0]])
                    new_laserstate.append(0)

                # Add path points
                for point_idx in path:
                    new_datax.append(self.datax[point_idx])
                    new_datay.append(self.datay[point_idx])
                    new_laserstate.append(self.laserstate[point_idx])

            # Update arrays
            self.datax = new_datax
            self.datay = new_datay
            self.laserstate = new_laserstate

            print("Path optimization complete")

            # Redraw preview
            self._draw_dxf_preview()

    # Add export to DXF functionality
    def export_path_as_dxf(self):
        """Export current path as DXF file"""
        if not self.datax:
            tk.messagebox.showwarning("No Data", "No path data to export!")
            return

        file_path = filedialog.asksaveasfilename(
            defaultextension=".dxf",
            filetypes=[("DXF files", "*.dxf"), ("All files", "*.*")]
        )

        if not file_path:
            return

        try:
            # Create new DXF document
            doc = ezdxf.new('R2010')
            msp = doc.modelspace()

            # Track continuous paths
            current_path = []

            for i in range(len(self.datax)):
                if i == 0 or self.laserstate[i] == 0:
                    # Start new path
                    if current_path and len(current_path) >= 2:
                        # Add previous path as polyline
                        msp.add_lwpolyline(current_path)
                    current_path = [(self.datax[i], self.datay[i])]
                else:
                    # Continue current path
                    current_path.append((self.datax[i], self.datay[i]))

            # Don't forget the last path
            if current_path and len(current_path) >= 2:
                msp.add_lwpolyline(current_path)

            # Save DXF
            doc.saveas(file_path)

            print(f"Exported DXF to {file_path}")
            tk.messagebox.showinfo("Export Complete", f"DXF saved to {file_path}")

        except Exception as e:
            tk.messagebox.showerror("Export Error", f"Failed to export DXF: {str(e)}")

            # Create new DXF document
            doc = ezdxf.new('R2010')
            msp = doc.modelspace()

            # Track continuous paths
            current_path = []

            for i in range(len(self.datax)):
                if i == 0 or self.laserstate[i] == 0:
                    # Start new path
                    if current_path and len(current_path) >= 2:
                        # Add previous path as polyline
                        msp.add_lwpolyline(current_path)
                    current_path = [(self.datax[i], self.datay[i])]
                else:
                    # Continue current path
                    current_path.append((self.datax[i], self.datay[i]))

            # Don't forget the last path
            if current_path and len(current_path) >= 2:
                msp.add_lwpolyline(current_path)

            # Save DXF
            doc.saveas(file_path)

            print(f"Exported DXF to {file_path}")
            tk.messagebox.showinfo("Export Complete", f"DXF saved to {file_path}")

        except Exception as e:
            tk.messagebox.showerror("Export Error", f"Failed to export DXF: {str(e)}")

    ###
    ## other
    ###

    def generate_contour_data(self):
        # Clear existing data
        self.datax = []
        self.datay = []
        self.laserstate = []

        # Get current scaling
        try:
            scaling = float(self.scalingentry.get())
        except:
            scaling = 0.1
            print("Invalid scaling, using 0.1")

        # Ensure laser power is set
        if self.laserpower == 0:
            self.laserpower = 255

        for c in range(len(self.raw_contours)):
            if len(self.raw_contours[c]) > 0:
                # First move to contour start with laser OFF
                first_x = (self.raw_contours[c][0, 0, 1] - int(self.frameshape[0] / 2)) * scaling
                first_y = (self.raw_contours[c][0, 0, 0] - int(self.frameshape[1] / 2)) * scaling

                self.datax.append(first_x)
                self.datay.append(first_y)
                self.laserstate.append(0)  # Laser OFF only for initial positioning

                # Draw the entire contour with laser ON (no turning off between points)
                for n in range(len(self.raw_contours[c])):
                    self.datax.append((self.raw_contours[c][n, 0, 1] - int(self.frameshape[0] / 2)) * scaling)
                    self.datay.append((self.raw_contours[c][n, 0, 0] - int(self.frameshape[1] / 2)) * scaling)
                    self.laserstate.append(self.laserpower)  # Keep laser ON

                # Close the contour by returning to first point (laser still ON)
                self.datax.append(first_x)
                self.datay.append(first_y)
                self.laserstate.append(self.laserpower)  # Keep laser ON

        # Print size info
        if self.datax:
            x_min, x_max = min(self.datax), max(self.datax)
            y_min, y_max = min(self.datay), max(self.datay)
            print(f"Scaling: {scaling} mm/pixel")
            print(f"Output size: {x_max - x_min:.1f} x {y_max - y_min:.1f} mm")

    def optimize_path_for_small_moves(self, min_distance=0.5):
        """Combine very short moves to prevent laser flickering"""
        if len(self.datax) < 2:
            return

        optimized_x = [self.datax[0]]
        optimized_y = [self.datay[0]]
        optimized_laser = [self.laserstate[0]]

        i = 1
        while i < len(self.datax):
            # Calculate distance from last point
            dx = self.datax[i] - optimized_x[-1]
            dy = self.datay[i] - optimized_y[-1]
            distance = (dx * dx + dy * dy) ** 0.5

            # If move is very short and laser states match, skip intermediate point
            if (distance < min_distance and
                    i < len(self.datax) - 1 and
                    self.laserstate[i] == optimized_laser[-1] and
                    self.laserstate[i] == self.laserstate[i + 1]):
                # Skip this point
                i += 1
                continue

            # Otherwise, add the point
            optimized_x.append(self.datax[i])
            optimized_y.append(self.datay[i])
            optimized_laser.append(self.laserstate[i])
            i += 1

        # Update with optimized path
        original_points = len(self.datax)
        self.datax = optimized_x
        self.datay = optimized_y
        self.laserstate = optimized_laser

        print(f"Path optimized: {original_points} points reduced to {len(self.datax)} points")

    def apply_scaling(self):
        """Apply new scaling, laser power, and speed to existing contours"""
        if hasattr(self, 'raw_contours'):
            # Update laser power from GUI
            self.set_laser_power()

            # Update speed from GUI
            self.set_speed()

            # Update acceleration from GUI
            self.set_acceleration()

            # Regenerate path data with new settings
            self.generate_contour_data()

            self.update_status_display()

            print(f"Settings applied!")
            print(f"  Scaling: {self.scalingentry.get()} mm/pixel")
            print(f"  Laser power: {self.laserpower}")
            print(f"  Speed: {self.speed_var.get()}%")
            print(f"  Acceleration: {self.accel_var.get()}")
        elif self.datax and hasattr(self, '_original_svg_data'):
            # For SVG files, reload with new scaling
            self._reload_svg_with_scaling()
        # Redraw preview
        if self.datax:
            self._draw_svg_preview()

    def _save_original_svg_data(self):
        """Save original data before scaling for later rescaling"""
        self._original_svg_data = {
            'datax': self.datax.copy(),
            'datay': self.datay.copy(),
            'laserstate': self.laserstate.copy()
        }

    def _reload_svg_with_scaling(self):
        """Reload SVG data with new scaling factor"""
        if hasattr(self, '_original_svg_data'):
            new_scale = float(self.scalingentry.get())
            old_scale = getattr(self, '_last_svg_scale', 1.0)
            scale_factor = new_scale / old_scale

            # Apply scale factor to all coordinates
            self.datax = [x * scale_factor for x in self._original_svg_data['datax']]
            self.datay = [y * scale_factor for y in self._original_svg_data['datay']]
            self.laserstate = self._original_svg_data['laserstate'].copy()

            self._last_svg_scale = new_scale

    def update_status_display(self):
        """Update a status label with current settings"""
        if hasattr(self, 'status_label'):
            status_text = f"Current: {self.scalingentry.get()} mm/px | Power: {self.laserpower} | Speed: {self.speed_var.get()}%"
            self.status_label.config(text=status_text)

    def set_scaling_from_size(self):
        """Set scaling based on desired output size"""
        if self.frameshape is None:
            print("Load an image first!")
            return

        try:
            target_width = float(self.box_width_entry.get())
            target_height = float(self.box_height_entry.get())

            new_scaling = self.calculate_scaling_for_target_size(target_width, target_height)

            self.scalingentry.delete(0, tk.END)
            self.scalingentry.insert(0, f"{new_scaling:.3f}")

            print(f"Scaling set to {new_scaling:.3f} mm/pixel")
            print(f"This will produce output approximately {target_width} x {target_height} mm")

        except Exception as e:
            print(f"Error setting scaling: {e}")

    def calculate_scaling_for_target_size(self, target_width_mm=None, target_height_mm=None):
        """Calculate scaling factor to achieve target output size"""
        if self.frameshape is None:
            return 0.1

        image_height, image_width = self.frameshape[0], self.frameshape[1]

        if target_width_mm and target_height_mm:
            # Calculate scale to fit both dimensions
            scale_x = target_width_mm / image_width
            scale_y = target_height_mm / image_height
            # Use the smaller scale to ensure it fits
            return min(scale_x, scale_y)
        elif target_width_mm:
            return target_width_mm / image_width
        elif target_height_mm:
            return target_height_mm / image_height
        else:
            return 0.1  # Default

    def update_canvas(self, img):
        im = Image.fromarray(img)
        im2 = im.resize((400, 300), Image.LANCZOS)
        self.imgtk = ImageTk.PhotoImage(image=im2)
        self.imagecanvas.create_image(0, 0, anchor=NW, image=self.imgtk)
        self.root.update()


    def move_pen(self):
        """Send all movement commands to the machine with preview offset option"""
        if not self.datax:
            print("No path data to send!")
            return

        # Ask if user wants to apply the preview offset
        if hasattr(self, 'preview_x_offset') and (self.preview_x_offset != 0 or self.preview_y_offset != 0):
            response = tk.messagebox.askyesno("Apply Offset",
                                              f"Apply preview offset ({self.preview_x_offset:.1f}, {self.preview_y_offset:.1f}) mm?")
            if response:
                self.apply_preview_offset_to_path()

        # Group consecutive moves with same laser state
        command_groups = []
        current_group = []
        last_laser_state = None

        for i in range(len(self.datax)):
            laser_state = self.laserstate[i]

            if laser_state != last_laser_state and current_group:
                # Save current group and start new one
                command_groups.append(current_group)
                current_group = []

            current_group.append({
                'x': self.datax[i],
                'y': self.datay[i],
                'laser': laser_state
            })
            last_laser_state = laser_state

        # Don't forget the last group
        if current_group:
            command_groups.append(current_group)

        print(f"Grouped {len(self.datax)} moves into {len(command_groups)} groups")

        # Process all commands
        total_commands = len(self.datax)
        command_index = 0

        # Send commands group by group
        for group_idx, group in enumerate(command_groups):
            print(
                f"Processing group {group_idx + 1}/{len(command_groups)} with {len(group)} moves, laser={group[0]['laser']}")

            for move in group:
                cmd = f"MOVXY {move['x']},{move['y']},{move['laser']}\n"

                # Clear any pending data before sending
                self.serialPort.reset_input_buffer()

                # Send command
                self.serialPort.write(cmd.encode("utf8"))
                self.serialPort.flush()

                command_index += 1
                print(f"Sent [{command_index}/{total_commands}]: {cmd.strip()}")

                # Update position tracking
                self.update_position(move['x'], move['y'], move['laser'])

                # Wait for READY response
                start_time = time.time()
                received_ready = False
                timeout_seconds = 30

                while not received_ready:
                    # Check for timeout
                    if time.time() - start_time > timeout_seconds:
                        print(f"Timeout waiting for READY after command {command_index}")
                        response = tk.messagebox.askyesno("Timeout",
                                                          "Communication timeout. Continue anyway?")
                        if response:
                            break
                        else:
                            print("Operation cancelled by user")
                            return

                    # Check for response
                    if self.serialPort.inWaiting():
                        try:
                            line = self.serialPort.readline().decode().strip()
                            print(f"Arduino: {line}")

                            if line == "READY":
                                received_ready = True
                            elif "POS:" in line:
                                # Parse position feedback if needed
                                pass
                            elif "limit" in line.lower():
                                # Handle limit switch warnings
                                print(f"WARNING: {line}")

                        except Exception as e:
                            print(f"Error reading serial: {e}")

                    # Small delay to prevent CPU spinning
                    time.sleep(0.001)

        print("ALL MOVES SENT - Waiting for final confirmation")

        # Wait for final READY
        start_time = time.time()
        while time.time() - start_time < 10:
            if self.serialPort.inWaiting():
                line = self.serialPort.readline().decode().strip()
                if line == "READY":
                    break

        # Return to origin
        print("Returning to origin...")
        self.serialPort.reset_input_buffer()
        movecommand = f"MOVXY 0,0,0\n"
        self.serialPort.write(movecommand.encode("utf8"))
        self.serialPort.flush()
        self.update_position(0, 0, 0)

        # Wait for final READY
        start_time = time.time()
        while time.time() - start_time < 10:
            if self.serialPort.inWaiting():
                line = self.serialPort.readline().decode().strip()
                if line == "READY":
                    print("MOVED BACK TO ORIGINAL POSITION")
                    break

        print("Job complete!")

        # Show completion message
        tk.messagebox.showinfo("Complete", "All moves completed successfully!")


    # # **MODIFIED: move_pen method to track positions**
    # def move_pen(self):
    #     commands = [f"MOVXY {self.datax[n]},{self.datay[n]},100\n" for n in range(len(self.datax))]
    #     command_index = 0
    #
    #     def send_next_command():
    #         nonlocal command_index
    #         if command_index < len(commands):
    #             cmd = commands[command_index]
    #             self.serialPort.write(cmd.encode("utf8"))
    #             print(f"Sent: {cmd.strip()}")
    #
    #             # **NEW: Extract position from command and update tracking**
    #             parts = cmd.strip().split(' ')[1].split(',')
    #             x_pos = float(parts[0])
    #             y_pos = float(parts[1])
    #             laser_power = int(parts[2])
    #             self.update_position(x_pos, y_pos, laser_power)
    #
    #             command_index += 1
    #
    #     # Prime the first command
    #     send_next_command()
    #
    #     while command_index < len(commands):
    #         if self.serialPort.inWaiting():
    #             line = self.serialPort.readline().decode().strip()
    #             if line == "READY":
    #                 send_next_command()
    #             else:
    #                 print(f"Arduino said: {line}")
    #
    #     print("ALL MOVES SENT")
    #
    #     # **MODIFIED: Track return to origin**
    #     movecommand = f"MOVXY {0},{0},0\n"
    #     self.serialPort.write(movecommand.encode("utf8"))
    #     self.update_position(0, 0, 0)
    #     print("MOVED BACK TO ORIGINAL POSITION")
    #
    #

    def draw_workspace_bounds(self, image):
        """Draw workspace boundaries on camera view"""
        h, w = image.shape[:2]
        y_offset = float(self.camera_y_offset_var.get())

        # Workspace corners in mm
        corners_mm = [
            (-self.max_x, -self.max_y),
            (self.max_x, -self.max_y),
            (self.max_x, self.max_y),
            (-self.max_x, self.max_y),
            (-self.max_x, -self.max_y)
        ]

        # Convert to camera pixels
        points = []
        for x_mm, y_mm in corners_mm:
            cam_x = int(self.camera_center_x + x_mm / self.camera_scale_x)
            cam_y = int(self.camera_center_y - (y_mm + y_offset) / self.camera_scale_y)
            points.append((cam_x, cam_y))

        # Draw workspace boundary
        for i in range(len(points) - 1):
            x1, y1 = points[i]
            x2, y2 = points[i + 1]

            # Clip to image bounds
            x1 = max(0, min(w - 1, x1))
            x2 = max(0, min(w - 1, x2))
            y1 = max(0, min(h - 1, y1))
            y2 = max(0, min(h - 1, y2))

            cv2.line(image, (x1, y1), (x2, y2), (64, 64, 64), 2)

    def draw_size_box(self, box_width_mm, box_height_mm):
        # Check if the ROI has been set
        if self.frameshape is None:
            print("ROI not set yet!")
            return

        # Unpack ROI dimensions (note: rows are height, columns are width)
        roi_height, roi_width = self.frameshape[0], self.frameshape[1]

        # Canvas dimensions (same as in update_canvas)
        canvas_width, canvas_height = 400, 300

        # Calculate conversion ratios from original ROI to canvas
        ratio_x = canvas_width / roi_width
        ratio_y = canvas_height / roi_height

        # Get the current scaling factor from the entry (mm per pixel)
        scaling_factor = float(self.scalingentry.get())

        # Convert physical box dimensions (mm) to pixels in the original ROI
        box_width_pixels = box_width_mm / scaling_factor
        box_height_pixels = box_height_mm / scaling_factor

        # Convert those to canvas (display) coordinates
        box_width_canvas = box_width_pixels * ratio_x
        box_height_canvas = box_height_pixels * ratio_y

        # Center the box on the canvas
        center_x = canvas_width / 2
        center_y = canvas_height / 2
        x0 = center_x - box_width_canvas / 2
        y0 = center_y - box_height_canvas / 2
        x1 = center_x + box_width_canvas / 2
        y1 = center_y + box_height_canvas / 2

        # Draw the rectangle overlay; adjust outline color or width as desired.
        self.imagecanvas.create_rectangle(x0, y0, x1, y1, outline="red", width=2)

    ####
    # Laser controls
    ####
    def set_laser_power(self, *args):
        try:
            # self.laser_var
            power = float(self.laser_var.get())

            self.laserpower = int((power))
            if self.laserpower > 255:
                self.laserpower = 255
        except:
            pass
        print(self.laserpower)

    def toggle_laser(self):
        if not self.laseron:
            self.laseron = True
            self.Laserbtn.configure(text="Laser is On")
            newcommand = f"LASON {self.laserpower}\n"
        else:
            self.laseron = False
            self.Laserbtn.configure(text="Laser is Off")
            newcommand = "LASON 0\n"

        self.serialPort.write(newcommand.encode("utf8"))

    ####
    # movement functions
    ####
    # **MODIFIED: All movement functions to track positions**
    def move_x(self):
        xmove = float(self.movexentry.get())
        movecommand = f"MOVEX {xmove}\n"
        self.serialPort.write(movecommand.encode("utf8"))
        # **NEW: Update position tracking**
        self.update_position(xmove, self.ypos, 0)

    def move_y(self):
        ymove = float(self.moveyentry.get())
        movecommand = f"MOVEY {ymove}\n"
        self.serialPort.write(movecommand.encode("utf8"))
        # **NEW: Update position tracking**
        self.update_position(self.xpos, ymove, 0)

    def move_home(self):
        movecommand = "MOVXY 0,0,0\n"
        self.serialPort.write(movecommand.encode("utf8"))
        # **NEW: Update position tracking**
        self.update_position(0, 0, 0)

    def set_home(self):
        homecommand = "SETHO\n"
        self.serialPort.write(homecommand.encode("utf8"))
        # **NEW: Update position tracking**
        self.update_position(0, 0, 0)

    def clear_data(self):
        self.datax = []
        self.datay = []
        self.laserstate = []

    def jog_up(self):
        move = float(self.jog_distance_entry.get())
        new_yposition = self.ypos + move
        movecommand = f"MOVEY {new_yposition}\n"
        self.serialPort.write(movecommand.encode("utf8"))
        self.update_position(self.xpos, new_yposition, 0)
        print(self.xpos,self.ypos)

    def jog_down(self):
        move = float(self.jog_distance_entry.get())
        new_yposition = self.ypos - move
        print("expected pos:",new_yposition)
        movecommand = f"MOVEY {new_yposition}\n"
        self.serialPort.write(movecommand.encode("utf8"))
        self.update_position(self.xpos, new_yposition, 0)
        print(self.xpos, self.ypos)

    def jog_left(self):
        move = float(self.jog_distance_entry.get())
        new_xposition = self.xpos - move
        movecommand = f"MOVEX {new_xposition}\n"
        self.serialPort.write(movecommand.encode("utf8"))
        self.update_position(new_xposition, self.ypos, 0)
        print(self.xpos, self.ypos)

    def jog_right(self):
        move = float(self.jog_distance_entry.get())
        new_xposition = self.xpos + move
        movecommand = f"MOVEX {new_xposition}\n"
        self.serialPort.write(movecommand.encode("utf8"))
        self.update_position(new_xposition, self.ypos, 0)
        print(self.xpos, self.ypos)

    def move_home_limits(self):
        # find x low
        movecommand = f"HOMEY \n"
        self.serialPort.write(movecommand.encode("utf8"))
        movecommand = f"HOMEX \n"
        self.serialPort.write(movecommand.encode("utf8"))
        self.update_position(0, 0, 0)
        print("homed")

    def set_acceleration(self,*args):
        try:
            self.acceleration = float(self.accel_var.get())
            homespeedcommand = f"ACCEL {self.acceleration}\n"
            self.serialPort.write(homespeedcommand.encode("utf8"))
            print("Acceleration set too:", self.acceleration)
        except:
            pass


    def set_speed(self, *args):
        try:
            speed_percent = int(float(self.speed_var.get()))
            max_speed = 5000
            min_speed = 100

            # Clamp percentage to 1-100
            if speed_percent > 100:
                speed_percent = 100
            elif speed_percent < 1:
                speed_percent = 1

            # Calculate actual speed from percentage
            # Using logarithmic scale for better control at low speeds
            # Linear: actual_speed = min_speed + (max_speed - min_speed) * speed_percent / 100

            # Logarithmic scaling for better low-speed control:
            import math
            log_min = math.log(min_speed)
            log_max = math.log(max_speed)
            log_speed = log_min + (log_max - log_min) * speed_percent / 100
            actual_speed = int(math.exp(log_speed))

            # Send percentage to Arduino
            speedcommand = f"SPEED {speed_percent}\n"
            self.serialPort.write(speedcommand.encode("utf8"))

            print(f"Speed set to: {speed_percent}% ({actual_speed} steps/s)")

            time.sleep(0.5)
            response = self.serialPort.readline().decode()
            print(response)

        except Exception as e:
            print(f"Error setting speed: {e}")
            pass


    def toggle_abort(self):
        self.runparam = not self.runparam
        self.abortbtn.configure(text="Restart Scan" if not self.runparam else "Stop Scan")

    def on_close(self):
        self.serialPort.close()
        #self.cap.release()
        time.sleep(1)
        self.root.destroy()

    ########################
    # Connections
    ########################

    def initialize_serial(self):
        print("Initialising COM ports")
        ports = list(port_list.comports())
        port = None

        try:
            for n in range(len(ports)):
                if "Arduino" in str(ports[n]):
                    port = ports[n].device
                    break
            if not port:
                raise Exception("No Arduino found")
        except Exception as e:
            print(str(e))
            sys.exit(1)

        try:
            serialPort = serial.Serial(port=port, baudrate=115200, timeout=2)
            serialPort.set_buffer_size(rx_size=4096, tx_size=4096)
            time.sleep(0.1)
        except:
            print("Unable to detect Arduino")
            print(ports)
            sys.exit(1)

        print(f'COM port for Arduino is {port}')
        return port, serialPort

if __name__ == "__main__":
    driver = XYStageDriver()
