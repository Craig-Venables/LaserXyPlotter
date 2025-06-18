# -*- coding: utf-8 -*-
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


class XYStageDriver:
    def __init__(self):
        self.port, self.serialPort = self.initialize_serial()

        self.laserpower = 0
        self.scaling_factor = 0.1
        self.speed = 32.0
        self.acceleration = 50
        self.xpos = 0
        self.ypos = 0
        self.distance_per_roatation = 8
        self.steps_per_revolutoin = 200

        # Empty Arrays
        self.datax = []
        self.datay = []
        self.laserstate = []

        self.x_len = 182.88
        self.y_len = 171.96
        self.max_x = self.x_len / 2
        self.max_y = self.y_len / 2

        # Flags
        self.laseron = False
        self.runparam = True
        self.frameshape = None
        self.framecrop = None

        # Initialise Gui
        self.root = tk.Tk()
        self.setup_ui()


    ########################
    # Set Up Gui
    ########################
    def setup_ui(self):
        self.root.title("XY Stage Driver")
        self.root.geometry('1500x700')

        self.control_frame()
        self.image_frame()
        self.webcam_frame()

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.root.mainloop()
    def webcam_frame(self):

        self.webcam_frame = ttk.LabelFrame(self.root, text="Webcam")
        self.webcam_frame.grid(row=0, column=3, padx=10, pady=10, sticky="nsew")

        self.webcam_label = ttk.Label(self.webcam_frame)
        self.webcam_label.pack()

        # Start webcam feed
        self.cap = None
        #self.root.after(500, self.start_webcam)

    def control_frame(self):
        # Frame for laser and motor controls
        control_frame = ttk.LabelFrame(self.root,text="Sequence Controls")
        control_frame.grid(row=0, column=0, padx=10, pady=10, sticky='n')

        # Laser
        self.laser_var = tk.StringVar(value="1")
        tk.Label(control_frame, text='Laser Power (mW)').grid(padx=10, pady=10, column=0, row=2)
        tk.Entry(control_frame, textvariable=self.laser_var).grid(padx=10, pady=10, column=1, row=2)
        self.setpowerbtn = tk.Button(control_frame, text="Set power", command=self.set_laser_power)
        self.setpowerbtn.grid(padx=10, pady=10, column=2, row=2)
        self.Laserbtn = tk.Button(control_frame, text="Laser is Off", command=self.toggle_laser)
        self.Laserbtn.grid(padx=10, pady=10, column=3, row=2)

        # Speed
        self.speed_var = tk.StringVar(value="32.0")
        tk.Label(control_frame, text='Speed (mms-1) 32').grid(padx=10, pady=10, column=0, row=3)
        tk.Entry(control_frame, textvariable=self.speed_var).grid(padx=10, pady=10, column=1, row=3)
        self.setspeedbtn = tk.Button(control_frame, text="Set Speed", command=self.set_speed)
        self.setspeedbtn.grid(padx=10, pady=10, column=2, row=3)

        #Acceleration
        self.accel_var = tk.StringVar(value="500")
        tk.Label(control_frame, text='Set Acceleration').grid(padx=10, pady=10, column=0, row=4)
        tk.Entry(control_frame, textvariable=self.accel_var).grid(padx=10, pady=10, column=1, row=4)

        # Move x
        tk.Button(control_frame, text="Move X", command=self.move_x).grid(padx=10, pady=10, column=0, row=5)
        self.movexentry = tk.Entry(control_frame)
        self.movexentry.insert(0, "0.0")
        self.movexentry.grid(padx=10, pady=10, column=1, row=5)

        # Move Y
        tk.Button(control_frame, text="Move Y", command=self.move_y).grid(padx=10, pady=10, column=0, row=6)
        self.moveyentry = tk.Entry(control_frame)
        self.moveyentry.insert(0, "0.0")
        self.moveyentry.grid(padx=10, pady=10, column=1, row=6)

        tk.Button(control_frame, text="Set Home to Current Position", command=self.set_home).grid(padx=10,pady=10,column=1,row=7)
        tk.Button(control_frame, text="Move to Home", command=self.move_home).grid(padx=10, pady=10, column=1,row=8)

        tk.Button(control_frame, text="FIND Home via limis", command=self.move_home_limits).grid(padx=10, pady=10, column=1, row=9)

        jog_frame = ttk.LabelFrame(control_frame, text="jog Controls")
        jog_frame.grid(row=10, column=0,rowspan=3, padx=10, pady=10,columnspan=3, sticky='n')

        # Jog Controls
        frame_jog = ttk.Frame(jog_frame, padding="10")
        frame_jog.grid(row=3, column=0, padx=10, pady=10, sticky="ew")
        ttk.Label(frame_jog, text="Jog Distance (mm):").grid(row=0, column=0, sticky="w")
        self.jog_distance_entry = ttk.Entry(frame_jog, width=10)
        self.jog_distance_entry.grid(row=0, column=1, padx=5)
        self.jog_distance_entry.insert(0, "2")

        ttk.Button(frame_jog, text="Jog Up", command=self.jog_up).grid(row=1, column=1, padx=5)
        ttk.Button(frame_jog, text="Jog Down", command=self.jog_down).grid(row=3, column=1, padx=5)
        ttk.Button(frame_jog, text="Jog Left", command=self.jog_left).grid(row=2, column=0, padx=5)
        ttk.Button(frame_jog, text="Jog Right", command=self.jog_right).grid(row=2, column=2, padx=5)

    def image_frame(self):
        image_frame = ttk.LabelFrame(self.root, text="Image")
        image_frame.grid(row=0, column=2, padx=10, pady=10, sticky='n')

        tk.Button(image_frame, text="Start", command=self.move_pen).grid(padx=10, pady=10, column=5, row=0)
        tk.Button(image_frame, text="Stop", command=self.on_close).grid(padx=10, pady=10, column=6, row=0)
        tk.Button(image_frame, text="Create Path from Image", command=self.generate_path_from_image).grid(padx=10,pady=10,column=3, row=0)
        tk.Button(image_frame, text="Clear Path Data", command=self.clear_data).grid(padx=10, pady=10, column=4, row=0)


        tk.Label(image_frame, text='Scaling factor (mm/pixel)').grid(padx=10, pady=10, column=3, row=2)
        self.scalingentry = tk.Entry(image_frame)
        self.scalingentry.insert(0, "0.1")
        self.scalingentry.grid(padx=10, pady=10, column=4, row=2)

        self.abortbtn = tk.Button(image_frame, text="Stop Scan", command=self.toggle_abort)
        self.abortbtn.grid(padx=10, pady=10, column=3, row=9)

        self.imagecanvas = tk.Canvas(image_frame, height=300, width=400)
        self.imagecanvas.grid(column=3, row=3, columnspan=2, rowspan=6)

        # In the image_frame method
        tk.Label(image_frame, text='Box Width (mm):').grid(padx=10, pady=5, column=3, row=10)
        self.box_width_entry = tk.Entry(image_frame)
        self.box_width_entry.insert(0, "50")
        self.box_width_entry.grid(padx=10, pady=5, column=4, row=10)

        tk.Label(image_frame, text='Box Height (mm):').grid(padx=10, pady=5, column=3, row=11)
        self.box_height_entry = tk.Entry(image_frame)
        self.box_height_entry.insert(0, "50")
        self.box_height_entry.grid(padx=10, pady=5, column=4, row=11)

        tk.Button(image_frame, text="Show Size Box", command=self.show_size_box).grid(padx=10, pady=5, column=3, row=12)

    ########################
    # GUI functions
    ########################

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
        # open file and ask fro interest
        file_path = filedialog.askopenfilename(filetypes=[("image files", "*.jpg"), ("all files", "*.*")])
        frame = cv2.imread(file_path)
        r = cv2.selectROI("Select region of interest", frame, False)
        cv2.destroyWindow("Select region of interest")


        self.framecrop = frame[int(r[1]):int(r[1] + r[3]), int(r[0]):int(r[0] + r[2])]
        self.frameshape = np.shape(self.framecrop)
        imagedata = self.framecrop[:, :, 1]
        thresh = cv2.threshold(imagedata, np.mean(imagedata) - 25, 255, cv2.THRESH_BINARY)[1]
        thresh = cv2.dilate(thresh, np.ones((3, 3), np.uint8), iterations=1)
        thresh = cv2.erode(thresh, np.ones((3, 3), np.uint8), iterations=1)

        contours = cv2.findContours(thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_TC89_L1)[0]
        #print(contours)
        contours = sorted(contours, key=cv2.contourArea)
        self.update_canvas(self.framecrop)
        for c in contours:
            cv2.drawContours(self.framecrop, c, -1, (255, 0, 0), thickness=2)

        self.generate_contour_data(contours)

    def generate_contour_data(self, contours):
        for c in range(len(contours)):
            for n in range(len(contours[c])):
                self.datax.append((contours[c][n, 0, 1] - int(self.frameshape[0] / 2)) * float(self.scalingentry.get()))
                self.datay.append((contours[c][n, 0, 0] - int(self.frameshape[1] / 2)) * float(self.scalingentry.get()))
                #self.laserstate.append(0 if n == 0 else self.laserpower)

            self.datax.append((contours[c][0, 0, 1] - int(self.frameshape[0] / 2)) * float(self.scalingentry.get()))
            self.datay.append((contours[c][0, 0, 0] - int(self.frameshape[1] / 2)) * float(self.scalingentry.get()))
            self.laserstate.append(self.laserpower)

    def update_canvas(self, img):
        im = Image.fromarray(img)
        im2 = im.resize((400, 300), Image.LANCZOS)
        self.imgtk = ImageTk.PhotoImage(image=im2)
        self.imagecanvas.create_image(0, 0, anchor=NW, image=self.imgtk)
        self.root.update()

    def move_pen(self):
        commands = [f"MOVXY {self.datax[n]},{self.datay[n]},100\n" for n in range(len(self.datax))]
        command_index = 0

        def send_next_command():
            nonlocal command_index
            if command_index < len(commands):
                cmd = commands[command_index]
                self.serialPort.write(cmd.encode("utf8"))
                print(f"Sent: {cmd.strip()}")
                command_index += 1

        # Prime the first command
        send_next_command()

        while command_index < len(commands):
            if self.serialPort.inWaiting():
                line = self.serialPort.readline().decode().strip()
                if line == "READY":
                    send_next_command()
                else:
                    print(f"Arduino said: {line}")

        print("ALL MOVES SENT")

        movecommand = f"MOVXY {self.xpos},{self.ypos},0\n"
        self.serialPort.write(movecommand.encode("utf8"))
        print("MOVED BACK TO ORIGINAL POSITION")

    # def move_pen(self):
    #     commands = []
    #     for n in range(len(self.datax)):
    #         if self.runparam:
    #             commands.append(f"MOVXY {self.datax[n]},{self.datay[n]},{100}\n")
    #             #message = self.serialPort.readline().decode()
    #             self.framecrop2 = cv2.circle(self.framecrop,
    #                                         (int(self.datay[n] / float(self.scalingentry.get()) + self.frameshape[
    #                                             1] / 2),
    #                                          int(self.datax[n] / float(self.scalingentry.get()) + self.frameshape[
    #                                              0] / 2)),
    #                                         1, (0, 255, 0), -1)
    #             self.update_canvas(self.framecrop2)
    #
    #     # Send all commands at once
    #     complete_command = ''.join(commands)
    #     self.serialPort.write(complete_command.encode("utf8"))
    #     print("sent commands")
    #     print(complete_command)
    #
    #
    #     print("MOVES MADE")
    #
    #     # Asynchronously read responses
    #     while self.serialPort.inWaiting() > 0:
    #         message = self.serialPort.readline().decode()
    #         # Handle the response as needed
    #         print(message)
    #
    #     print("MOVEs DONE")
    #     time.sleep(1)
    #     movecommand = f"MOVXY {self.xpos},{self.ypos},0\n"
    #     self.serialPort.write(movecommand.encode("utf8"))
    #     #self.move_home()
    #     print("MOVED BACK TO ORIGINAL POSITION ")

    def update_webcam_thread(self):
        while self.webcam_running:
            ret, frame = self.cap.read()
            if ret:
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                img = Image.fromarray(frame)
                imgtk = ImageTk.PhotoImage(image=img)

                def update_gui():
                    self.webcam_label.config(image=imgtk)
                    self.webcam_label.image = imgtk  # Prevent garbage collection

                self.webcam_label.after(0, update_gui)

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
            #self.laser_var
            power = float(self.laser_var.get())

            self.laserpower = int((power))
            if self.laserpower > 100:
                self.laserpower = 100
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
    def move_x(self):
        #self.set_speed()
        xmove = float(self.movexentry.get())
        movecommand = f"MOVEX {xmove}\n"
        self.serialPort.write(movecommand.encode("utf8"))
        # Read the updated position from serial
        self.read_serial()

    def move_y(self):
        #self.set_speed()
        ymove = float(self.moveyentry.get())
        movecommand = f"MOVEY {ymove}\n"
        self.serialPort.write(movecommand.encode("utf8"))

        self.read_serial()

    def move_home(self):
        #self.set_speed()
        movecommand = "MOVXY 0,0,0\n"
        self.serialPort.write(movecommand.encode("utf8"))
        self.ypos = 0
        self.xpos = 0

        self.read_serial()

    def set_home(self):
        #self.set_speed()
        homecommand = "SETHO\n"
        self.serialPort.write(homecommand.encode("utf8"))
        self.xpos =0
        self.ypos =0

        self.read_serial()

    def clear_data(self):
        self.datax = []
        self.datay = []
        self.laserstate = []
        print('Path data cleared')

    def jog_up(self):

        move = float(self.jog_distance_entry.get())
        new_yposition = int(self.ypos) - move
        movecommand = f"MOVEY {new_yposition}\n"
        self.serialPort.write(movecommand.encode("utf8"))

        self.read_serial()

    def jog_down(self):
        move = float(self.jog_distance_entry.get())
        new_yposition = int(self.ypos) + move
        print("expected pos:",new_yposition)
        movecommand = f"MOVEY {new_yposition}\n"
        self.serialPort.write(movecommand.encode("utf8"))

        self.read_serial()

    def jog_left(self):
        move = float(self.jog_distance_entry.get())
        new_xposition = int(self.xpos) + move
        movecommand = f"MOVEX {new_xposition}\n"
        self.serialPort.write(movecommand.encode("utf8"))

        self.read_serial()

    def jog_right(self):
        move = float(self.jog_distance_entry.get())
        new_xposition = int(self.xpos) - move
        movecommand = f"MOVEX {new_xposition}\n"
        self.serialPort.write(movecommand.encode("utf8"))

        self.read_serial()

    def move_home_limits(self):
        # find x low
        movecommand = f"HOMEY \n"
        self.serialPort.write(movecommand.encode("utf8"))
        movecommand = f"HOMEX \n"
        self.serialPort.write(movecommand.encode("utf8"))
        self.xpos =0
        self.ypos =0
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
            self.speed = int(self.speed_var.get())
            if self.speed > 32.0:
                self.speed = 32
            homespeedcommand = f"SPEED {self.speed}\n"
            self.serialPort.write(homespeedcommand.encode("utf8"))
            print("speed set too:", self.speed)
            #print(homespeedcommand)
            time.sleep(0.5)
            x=self.serialPort.readline().decode()
            time.sleep(0.5)
            print(x)
        except:
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

    def read_serial(self):
        start_time = time.time()
        timeout = 2  # seconds
        while time.time() - start_time < timeout:
            if self.serialPort.inWaiting() > 0:
                line = self.serialPort.readline().decode().strip()
                if line.startswith("POS:"):
                    try:
                        _, coord_string = line.split("POS:")
                        x_str, y_str = coord_string.split(',')
                        self.xpos = int(float(x_str))
                        self.ypos = int(float(y_str))
                        print(f"Updated position: x={self.xpos}, y={self.ypos}")
                        return
                    except ValueError as e:
                        print(f"Error parsing position: {line} — {e}")
                else:
                    # Still read and print other lines like "READY"
                    print(f"Ignoring non-position serial line: {line}")
            else:
                time.sleep(0.01)
        print("Timed out waiting for position update.")

if __name__ == "__main__":
    driver = XYStageDriver()