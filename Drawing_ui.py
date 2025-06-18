import math
import numpy as np
import time
import tkinter as tk
from tkinter import ttk, filedialog, Canvas, NW
from PIL import Image, ImageTk
import cv2



def drawing_frame(self):
    """Create a frame for drawing shapes and lines"""
    self.draw_frame = ttk.LabelFrame(self.root, text="Drawing Tool")
    self.draw_frame.grid(row=0, column=5, padx=10, pady=10, sticky="nsew")

    # Drawing canvas
    self.drawing_canvas = tk.Canvas(self.draw_frame, width=400, height=400, bg='white')
    self.drawing_canvas.grid(row=0, column=0, columnspan=4, padx=5, pady=5)

    # Bind mouse events
    self.drawing_canvas.bind("<Button-1>", self.on_canvas_click)
    self.drawing_canvas.bind("<B1-Motion>", self.on_canvas_drag)
    self.drawing_canvas.bind("<ButtonRelease-1>", self.on_canvas_release)

    # Drawing mode selection
    tk.Label(self.draw_frame, text="Drawing Mode:").grid(row=1, column=0, padx=5, pady=5)
    self.drawing_mode = tk.StringVar(value="line")
    modes = [("Line", "line"), ("Rectangle", "rectangle"), ("Circle", "circle"),
             ("Free Draw", "free"), ("Polygon", "polygon")]

    for i, (text, mode) in enumerate(modes):
        tk.Radiobutton(self.draw_frame, text=text, variable=self.drawing_mode,
                       value=mode).grid(row=1, column=i + 1, padx=5)

    # Control buttons
    button_row = 2
    tk.Button(self.draw_frame, text="Clear Canvas",
              command=self.clear_drawing_canvas).grid(row=button_row, column=0, padx=5, pady=5)
    tk.Button(self.draw_frame, text="Undo Last",
              command=self.undo_last_drawing).grid(row=button_row, column=1, padx=5, pady=5)
    tk.Button(self.draw_frame, text="Generate Path",
              command=self.generate_path_from_drawing).grid(row=button_row, column=2, padx=5, pady=5)

    # Canvas size in mm
    tk.Label(self.draw_frame, text="Canvas Size (mm):").grid(row=3, column=0, padx=5, pady=5)
    self.canvas_size_var = tk.StringVar(value="100")
    tk.Entry(self.draw_frame, textvariable=self.canvas_size_var, width=10).grid(row=3, column=1, padx=5, pady=5)

    # Grid toggle
    self.show_grid_var = tk.BooleanVar(value=True)
    tk.Checkbutton(self.draw_frame, text="Show Grid", variable=self.show_grid_var,
                   command=self.update_drawing_grid).grid(row=3, column=2, padx=5, pady=5)

    # In drawing_frame method, add these buttons:
    tk.Button(self.draw_frame, text="Save Drawing",
              command=self.save_drawing).grid(row=button_row, column=3, padx=5, pady=5)
    tk.Button(self.draw_frame, text="Load Drawing",
              command=self.load_drawing).grid(row=button_row, column=4, padx=5, pady=5)

    # Initialize drawing variables
    self.drawing_objects = []  # Store all drawn objects
    self.current_drawing = None
    self.start_x = None
    self.start_y = None
    self.polygon_points = []
    self.temp_line = None

    # Draw initial grid
    self.update_drawing_grid()


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
        self.laserpower = 100

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