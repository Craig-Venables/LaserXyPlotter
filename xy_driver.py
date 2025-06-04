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


class XYStageDriver:
    def __init__(self):
        self.datax = []
        self.datay = []
        self.laserstate = []
        self.laserpower = 0
        self.laseron = False
        self.runparam = True
        self.port, self.serialPort = self.initialize_serial()
        self.frameshape = None
        self.framecrop = None
        self.scaling_factor = 0.1
        self.speed = 32.0
        self.acceleration = 50
        self.xpos = 0
        self.ypos = 0
        self.distance_per_roatation = 8
        self.steps_per_revolutoin = 200
        self.x_len = 182.88
        self.y_len = 171.96
        self.max_x = self.x_len / 2
        self.max_y = self.y_len / 2

        self.root = tk.Tk()
        self.setup_ui()




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
            serialPort = serial.Serial(port=port, baudrate=9600, timeout=2)
            serialPort.set_buffer_size(rx_size=4096, tx_size=4096)
            time.sleep(0.1)
        except:
            print("Unable to detect Arduino")
            print(ports)
            sys.exit(1)

        print(f'COM port for Arduino is {port}')
        return port, serialPort

    # def setup_ui(self):
    #     self.root.title("XY Stage Driver")
    #     self.root.geometry('1200x600')
    #
    #     # Frame for laser and motor controls
    #     control_frame = tk.Frame(self.root)
    #     control_frame.grid(row=0, column=0, padx=10, pady=10, sticky='n')
    #
    #     tk.Label(control_frame, text='Laser Power (mW)').grid(row=0, column=0, padx=10, pady=5)
    #     self.laser_var = tk.StringVar(value="1")
    #     tk.Entry(control_frame, textvariable=self.laser_var).grid(row=0, column=1, padx=10, pady=5)
    #     tk.Button(control_frame, text="Apply Power", command=self.set_laser_power).grid(row=0, column=2, padx=10,
    #                                                                                     pady=5)
    #     tk.Button(control_frame, text="Toggle Laser", command=self.toggle_laser).grid(row=0, column=3, padx=10, pady=5)
    #
    #     tk.Label(control_frame, text='Carriage Speed (mms-1) [Max: 8.0]').grid(row=1, column=0, padx=10, pady=5)
    #     self.speed_var = tk.StringVar(value="8.0")
    #     tk.Entry(control_frame, textvariable=self.speed_var).grid(row=1, column=1, padx=10, pady=5)
    #     tk.Button(control_frame, text="Set Speed", command=self.set_speed).grid(row=1, column=2, padx=10, pady=5)
    #
    #     tk.Label(control_frame, text='Set Acceleration').grid(row=2, column=0, padx=10, pady=5)
    #     self.accel_var = tk.StringVar(value="500")
    #     tk.Entry(control_frame, textvariable=self.accel_var).grid(row=2, column=1, padx=10, pady=5)
    #     tk.Button(control_frame, text="Set Acceleration", command=self.set_acceleration).grid(row=2, column=2, padx=10,
    #                                                                                           pady=5)
    #
    #     tk.Button(control_frame, text="Home Motors", command=self.move_home).grid(row=3, columnspan=3, padx=10, pady=10)
    #
    #     # Frame for movement controls
    #     motion_frame = tk.Frame(self.root)
    #     motion_frame.grid(row=1, column=0, padx=10, pady=10, sticky='n')
    #
    #     tk.Button(motion_frame, text="Start", command=self.move_pen).grid(row=0, column=0, padx=10, pady=5)
    #     tk.Button(motion_frame, text="Stop", command=self.on_close).grid(row=0, column=1, padx=10, pady=5)
    #     tk.Button(motion_frame, text="Create Path from Image", command=self.generate_path_from_image).grid(row=1,
    #                                                                                                        column=0,
    #                                                                                                        padx=10,
    #                                                                                                        pady=5)
    #     tk.Button(motion_frame, text="Clear Path Data", command=self.clear_data).grid(row=1, column=1, padx=10, pady=5)
    #
    #     tk.Button(motion_frame, text="Move X", command=self.move_x).grid(row=2, column=0, padx=10, pady=5)
    #     self.movexentry = tk.Entry(motion_frame)
    #     self.movexentry.insert(0, "0.0")
    #     self.movexentry.grid(row=2, column=1, padx=10, pady=5)
    #
    #     tk.Button(motion_frame, text="Move Y", command=self.move_y).grid(row=3, column=0, padx=10, pady=5)
    #     self.moveyentry = tk.Entry(motion_frame)
    #     self.moveyentry.insert(0, "0.0")
    #     self.moveyentry.grid(row=3, column=1, padx=10, pady=5)
    #
    #     tk.Button(motion_frame, text="Set Home to Current Position", command=self.set_home).grid(row=4, column=0,
    #                                                                                              columnspan=2, padx=10,
    #                                                                                              pady=5)
    #     tk.Button(motion_frame, text="Find Home", command=self.find_home).grid(row=5, column=0, columnspan=2, padx=10,
    #                                                                            pady=5)
    #
    #     tk.Button(motion_frame, text="Stop Scan", command=self.toggle_abort).grid(row=6, column=0, columnspan=2,
    #                                                                               padx=10, pady=5)
    #
    #     # Frame for image display
    #     canvas_frame = tk.Frame(self.root)
    #     canvas_frame.grid(row=0, column=1, rowspan=2, padx=10, pady=10, sticky='n')
    #
    #     self.imagecanvas = Canvas(canvas_frame, height=300, width=400)
    #     self.imagecanvas.grid(row=0, column=0, padx=10, pady=5)
    #
    #     self.root.protocol("WM_DELETE_WINDOW", self.on_close)
    #     self.root.mainloop()



    def setup_ui(self):
        self.root.title("XY Stage Driver")
        self.root.geometry('1000x500')

        self.control_frame()

        tk.Button(self.root, text="Start", command=self.move_pen).grid(padx=10, pady=10, column=5, row=0)
        tk.Button(self.root, text="Stop", command=self.on_close).grid(padx=10, pady=10, column=6, row=0)
        tk.Button(self.root, text="Create Path from Image", command=self.generate_path_from_image).grid(padx=10,
                                                                                                        pady=10,
                                                                                                        column=3, row=0)
        tk.Button(self.root, text="Clear Path Data", command=self.clear_data).grid(padx=10, pady=10, column=4, row=0)



        tk.Label(self.root, text='Scaling factor (mm/pixel)').grid(padx=10, pady=10, column=3, row=2)
        self.scalingentry = tk.Entry(self.root)
        self.scalingentry.insert(0, "0.1")
        self.scalingentry.grid(padx=10, pady=10, column=4, row=2)



        self.abortbtn = tk.Button(self.root, text="Stop Scan", command=self.toggle_abort)
        self.abortbtn.grid(padx=10, pady=10, column=3, row=9)

        self.imagecanvas = Canvas(self.root, height=300, width=400)
        self.imagecanvas.grid(column=3, row=3, columnspan=2, rowspan=6)

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.root.mainloop()

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
        tk.Label(control_frame, text='Carriage Speed (mms-1) [Max Val: 32.0 mms-1]').grid(padx=10, pady=10, column=0, row=3)
        tk.Entry(control_frame, textvariable=self.speed_var).grid(padx=10, pady=10, column=1, row=3)
        self.setspeedbtn = tk.Button(control_frame, text="Set Speed", command=self.set_speed)
        self.setspeedbtn.grid(padx=10, pady=10, column=2, row=3)

        #Acceleration
        self.accel_var = tk.StringVar(value="500")
        self.accel_var.trace('w', self.set_acceleration)
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

        tk.Button(control_frame, text="Set Home to Current Position", command=self.set_home).grid(padx=10,pady=10,column=2,row=6)
        tk.Button(control_frame, text="Move to Home", command=self.move_home).grid(padx=10, pady=10, column=1,row=7)

        tk.Button(control_frame, text="FIND Home via limis", command=self.move_home_limits).grid(padx=10, pady=10, column=1, row=8)

        jog_frame = ttk.LabelFrame(control_frame, text="jog Controls")
        jog_frame.grid(row=8, column=0,rowspan=3, padx=10, pady=10, sticky='n')

        # Jog Controls
        frame_jog = ttk.Frame(jog_frame, padding="10")
        frame_jog.grid(row=4, column=0, padx=10, pady=10, sticky="ew")
        ttk.Label(frame_jog, text="Jog Distance (mm):").grid(row=0, column=0, sticky="w")
        self.jog_distance_entry = ttk.Entry(frame_jog, width=10)
        self.jog_distance_entry.grid(row=0, column=1, padx=5)
        self.jog_distance_entry.insert(0, "2")

        ttk.Button(frame_jog, text="Jog Up", command=self.jog_up).grid(row=1, column=1, padx=5)
        ttk.Button(frame_jog, text="Jog Down", command=self.jog_down).grid(row=3, column=1, padx=5)
        ttk.Button(frame_jog, text="Jog Left", command=self.jog_left).grid(row=2, column=0, padx=5)
        ttk.Button(frame_jog, text="Jog Right", command=self.jog_right).grid(row=2, column=2, padx=5)

    def find_home(self):
        print("finding home")

    def generate_path_from_image(self):
        self.datax = []
        self.datay = []
        self.laserstate = []
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

        for c in contours:
            cv2.drawContours(self.framecrop, c, -1, (255, 0, 0), thickness=2)

        self.update_canvas(self.framecrop)
        self.generate_contour_data(contours)

    def generate_contour_data(self, contours):
        for c in range(len(contours)):
            for n in range(len(contours[c])):
                self.datax.append((contours[c][n, 0, 1] - int(self.frameshape[0] / 2)) * float(self.scalingentry.get()))
                self.datay.append((contours[c][n, 0, 0] - int(self.frameshape[1] / 2)) * float(self.scalingentry.get()))
                self.laserstate.append(0 if n == 0 else self.laserpower)

            self.datax.append((contours[c][0, 0, 1] - int(self.frameshape[0] / 2)) * float(self.scalingentry.get()))
            self.datay.append((contours[c][0, 0, 0] - int(self.frameshape[1] / 2)) * float(self.scalingentry.get()))
            self.laserstate.append(self.laserpower)

    def update_canvas(self, img):
        im = Image.fromarray(img)
        im2 = im.resize((400, 300), Image.LANCZOS)
        imgtk = ImageTk.PhotoImage(image=im2)
        self.imagecanvas.create_image(0, 0, anchor=NW, image=imgtk)
        self.root.update()

    def move_pen(self):
        commands = []
        for n in range(len(self.datax)):
            if self.runparam:
                commands.append(f"MOVXY {self.datax[n]},{self.datay[n]},{self.laserstate[n]}\n")
                #message = self.serialPort.readline().decode()
                self.framecrop = cv2.circle(self.framecrop,
                                            (int(self.datay[n] / float(self.scalingentry.get()) + self.frameshape[
                                                1] / 2),
                                             int(self.datax[n] / float(self.scalingentry.get()) + self.frameshape[
                                                 0] / 2)),
                                            1, (0, 255, 0), -1)
                self.update_canvas(self.framecrop)

        # Send all commands at once
        complete_command = ''.join(commands)
        self.serialPort.write(complete_command.encode("utf8"))

        # Asynchronously read responses
        while self.serialPort.inWaiting() > 0:
            message = self.serialPort.readline().decode()
            # Handle the response as needed

        self.move_home()

    # def move_pen(self):
    #     for n in range(len(self.datax)):
    #         if self.runparam:
    #             newcommand = f"MOVXY {self.datax[n]},{self.datay[n]},{self.laserstate[n]}\n"
    #             start = time.perf_counter()
    #             currenttime = 0
    #             timeout = 2
    #             self.serialPort.write(newcommand.encode("utf8"))
    #
    #             while (self.serialPort.inWaiting() == 0) & (currenttime < timeout):
    #                 currenttime = time.perf_counter() - start
    #
                # message = self.serialPort.readline().decode()
                # self.framecrop = cv2.circle(self.framecrop,
                #                             (int(self.datay[n] / float(self.scalingentry.get()) + self.frameshape[
                #                                 1] / 2),
                #                              int(self.datax[n] / float(self.scalingentry.get()) + self.frameshape[
                #                                  0] / 2)),
                #                             1, (0, 255, 0), -1)
                # self.update_canvas(self.framecrop)
    #
    #     self.move_home()

    def set_laser_power(self, *args):
        try:
            #self.laser_var
            power = float(self.laser_var.get())
            V = (0.57293088182473 + 0.020850373782878 * power -
                 3.80944309285036e-5 * power ** 2 +
                 4.98300866491694e-8 * power ** 3)
            self.laserpower = int(V * 255 / 5)
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

    def move_x(self):
        #self.set_speed()
        xmove = float(self.movexentry.get())
        movecommand = f"MOVEX {xmove}\n"
        self.serialPort.write(movecommand.encode("utf8"))

    def move_y(self):
        #self.set_speed()
        ymove = float(self.moveyentry.get())
        movecommand = f"MOVEY {ymove}\n"
        self.serialPort.write(movecommand.encode("utf8"))

    def move_home(self):
        #self.set_speed()
        movecommand = "MOVXY 0,0,0\n"
        self.serialPort.write(movecommand.encode("utf8"))

    def set_home(self):
        #self.set_speed()
        homecommand = "SETHO\n"
        self.serialPort.write(homecommand.encode("utf8"))

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
        #update new location
        self.ypos = new_yposition

    def jog_down(self):
        move = float(self.jog_distance_entry.get())
        new_yposition = int(self.ypos) + move
        movecommand = f"MOVEY {new_yposition}\n"
        self.serialPort.write(movecommand.encode("utf8"))
        # update new location
        self.ypos = new_yposition

    def jog_left(self):
        move = float(self.jog_distance_entry.get())
        new_xposition = int(self.xpos) + move
        movecommand = f"MOVEX {new_xposition}\n"
        self.serialPort.write(movecommand.encode("utf8"))
        # update new location
        self.xpos = new_xposition
    def jog_right(self):
        move = float(self.jog_distance_entry.get())
        new_xposition = int(self.xpos) - move
        movecommand = f"MOVEX {new_xposition}\n"
        self.serialPort.write(movecommand.encode("utf8"))
        # update new location
        self.xpos = new_xposition

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
        time.sleep(1)
        self.root.destroy()


if __name__ == "__main__":
    driver = XYStageDriver()