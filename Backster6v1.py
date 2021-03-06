
import matplotlib
matplotlib.use("TkAgg")
import Tkinter as tk
import ttk as ttk
import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
import matplotlib.animation as animation
import matplotlib.patches as patches
import serial


# GUI Class
class Input(tk.Tk):

    def __init__(self, *args, **kwargs):
        tk.Tk.__init__(self, *args, **kwargs)

        tk.Tk.wm_title(self, "Backster v4.0")

        container = tk.Frame(self)
        container.grid_rowconfigure(0, weight=1)
        container.grid_columnconfigure(0, weight=1)
        container.grid_columnconfigure(5, weight=1)

        container.grid(row=0, column=0, sticky="nsew")
        container.config(bg="grey91")


        # initialize variables
        self.l1 = tk.StringVar()
        self.l2 = tk.StringVar()
        self.l3 = tk.StringVar()
        self.l4 = tk.StringVar()
        self.l5 = tk.StringVar()
        self.length1 = tk.StringVar(value=0.0)
        self.length2 = tk.StringVar(value=0.0)
        self.length3 = tk.StringVar(value=0.0)
        self.length4 = tk.StringVar(value=0.0)
        self.length5 = tk.StringVar(value=0.0)
        self.exercise = tk.StringVar(value="Select Exercise")

        # create column 1 features
        ttk.Label(container, text="Exercise").grid(column=1, row=1, sticky='n')
        ttk.Button(container, text="Toe Touch", command=lambda: self.exercise_select(1)).grid(column=1, row=2, sticky='n')
        ttk.Button(container, text="Back Bend", command=lambda: self.exercise_select(2)).grid(column=1, row=3, sticky='n')
        ttk.Button(container, text="Torsional Twist", command=lambda: self.exercise_select(3)).grid(column=1, row=4, sticky='n')
        ttk.Button(container, text="Free Mode", command=lambda: self.exercise_select(4)).grid(column=1, row=5, sticky='n')
        ttk.Label(container, textvariable=self.exercise).grid(column=1, row=6, sticky='s')

        # column 2
        ttk.Label(container, text="Link 1:").grid(column=2, row=2, sticky='e')
        ttk.Label(container, text="Link 2:").grid(column=2, row=3, sticky='e')
        ttk.Label(container, text="Link 3:").grid(column=2, row=4, sticky='e')
        ttk.Label(container, text="Link 4:").grid(column=2, row=5, sticky='e')
        ttk.Label(container, text="Link 5:").grid(column=2, row=6, sticky='e')

        # column 3
        ttk.Label(container, text="Link Lengths (cm)").grid(column=3, row=1, sticky='n')
        l1_entry = ttk.Entry(container, width=4, textvariable=self.l1)
        l1_entry.grid(column=3, row=2, sticky=('w', 'e'))
        l2_entry = ttk.Entry(container, width=4, textvariable=self.l2)
        l2_entry.grid(column=3, row=3, sticky=('w', 'e'))
        l3_entry = ttk.Entry(container, width=4, textvariable=self.l3)
        l3_entry.grid(column=3, row=4, sticky=('w', 'e'))
        l4_entry = ttk.Entry(container, width=4, textvariable=self.l4)
        l4_entry.grid(column=3, row=5, sticky=('w', 'e'))
        l5_entry = ttk.Entry(container, width=4, textvariable=self.l5)
        l5_entry.grid(column=3, row=6, sticky=('w', 'e'))
        ttk.Button(container, text="Set Lengths", command=self.setLengths).grid(column=3, row=7, sticky='w')

        # column 4
        ttk.Label(container, textvariable=self.length1).grid(column=4, row=2, sticky=('w', 'e'))
        ttk.Label(container, textvariable=self.length2).grid(column=4, row=3, sticky=('w', 'e'))
        ttk.Label(container, textvariable=self.length3).grid(column=4, row=4, sticky=('w', 'e'))
        ttk.Label(container, textvariable=self.length4).grid(column=4, row=5, sticky=('w', 'e'))
        ttk.Label(container, textvariable=self.length5).grid(column=4, row=6, sticky=('w', 'e'))
        ttk.Button(container, text="Start", command=lambda: self.start(self.exercise.get()))\
            .grid(column=4, row=7, sticky='w')

        # make it look pretty
        for child in container.winfo_children(): child.grid_configure(padx=5, pady=5)
        # put cursor in the first box
        l1_entry.focus()
        # allow hitting enter to set Lengths
        container.bind('<Return>', self.setLengths)

    def start(self, ex):
        self.setLengths()

        # initialize at 0
        link1 = 0.0
        link2 = 0.0
        link3 = 0.0
        link4 = 0.0
        link5 = 0.0

        if ex == "Select Exercise":
            return
        else:
            # set values in script to these
            try:
                link1 = float(self.l1.get())
                link2 = float(self.l2.get())
                link3 = float(self.l3.get())
                link4 = float(self.l4.get())
                link5 = float(self.l5.get())
            except ValueError:
                pass
                return

            # close window
            self.destroy()

        return link1, link2, link3, link4, link5, ex

    def setLengths(self, *args):
        try:
            value1 = float(self.l1.get())
            self.length1.set(value1)
        except ValueError:
            pass
        try:
            value2 = float(self.l2.get())
            self.length2.set(value2)
        except ValueError:
            pass
        try:
            value3 = float(self.l3.get())
            self.length3.set(value3)
        except ValueError:
            pass
        try:
            value4 = float(self.l4.get())
            self.length4.set(value4)
        except ValueError:
            pass
        try:
            value5 = float(self.l5.get())
            self.length5.set(value5)
        except ValueError:
            pass

    def exercise_select(self, enum):
        if enum == 1:
            self.exercise.set("Toe Touch")
        elif enum == 2:
            self.exercise.set("Back Bend")
        elif enum == 3:
            self.exercise.set("Twist")
        elif enum == 4:
            self.exercise.set("Free")

        return


def offsetAngle(samples):
    qw1_init = 0
    qx1_init = 0
    qy1_init = 0
    qz1_init = 0
    qw2_init = 0
    qx2_init = 0
    qy2_init = 0
    qz2_init = 0
    qw3_init = 0
    qx3_init = 0
    qy3_init = 0
    qz3_init = 0
    qw4_init = 0
    qx4_init = 0
    qy4_init = 0
    qz4_init = 0
    qw5_init = 0
    qx5_init = 0
    qy5_init = 0
    qz5_init = 0
    qw6_init = 0
    qx6_init = 0
    qy6_init = 0
    qz6_init = 0

    for i in range(0, samples):
        data = ser.readline()
        split_data = data.split()
        qw1_init += float(split_data[1])
        qx1_init += float(split_data[2])
        qy1_init += float(split_data[3])
        qz1_init += float(split_data[4])
        qw2_init += float(split_data[5])
        qx2_init += float(split_data[6])
        qy2_init += float(split_data[7])
        qz2_init += float(split_data[8])
        qw3_init += float(split_data[9])
        qx3_init += float(split_data[10])
        qy3_init += float(split_data[11])
        qz3_init += float(split_data[12])
        qw4_init += float(split_data[13])
        qx4_init += float(split_data[14])
        qy4_init += float(split_data[15])
        qz4_init += float(split_data[16])
        qw5_init += float(split_data[17])
        qx5_init += float(split_data[18])
        qy5_init += float(split_data[19])
        qz5_init += float(split_data[20])
        qw6_init += float(split_data[21])
        qx6_init += float(split_data[22])
        qy6_init += float(split_data[23])
        qz6_init += float(split_data[24])

    qw1_init = qw1_init/100.0
    qx1_init = qx1_init/100.0
    qy1_init = qy1_init/100.0
    qz1_init = qz1_init/100.0
    qw2_init = qw2_init/100.0
    qx2_init = qx2_init/100.0
    qy2_init = qy2_init/100.0
    qz2_init = qz2_init/100.0
    qw3_init = qw3_init/100.0
    qx3_init = qx3_init/100.0
    qy3_init = qy3_init/100.0
    qz3_init = qz3_init/100.0
    qw4_init = qw4_init/100.0
    qx4_init = qx4_init/100.0
    qy4_init = qy4_init/100.0
    qz4_init = qz4_init/100.0
    qw5_init = qw5_init/100.0
    qx5_init = qx5_init/100.0
    qy5_init = qy5_init/100.0
    qz5_init = qz5_init/100.0
    qw6_init = qw6_init/100.0
    qx6_init = qx6_init/100.0
    qy6_init = qy6_init/100.0
    qz6_init = qz6_init/100.0

    # create arrays of each quaternion
    quat1 = np.array([qw1_init, qx1_init, qy1_init, qz1_init])
    quat2 = np.array([qw2_init, qx2_init, qy2_init, qz2_init])
    quat3 = np.array([qw3_init, qx3_init, qy3_init, qz3_init])
    quat4 = np.array([qw4_init, qx4_init, qy4_init, qz4_init])
    quat5 = np.array([qw5_init, qx5_init, qy5_init, qz5_init])
    quat6 = np.array([qw6_init, qx6_init, qy6_init, qz6_init])

    # create skeleton line transformations
    nqv2to1 = quatRotate(quat2, qv2to1)
    nqv3to2 = quatRotate(quat3, qv3to2)
    nqv4to3 = quatRotate(quat4, qv4to3)
    nqv5to4 = quatRotate(quat5, qv5to4)
    nqv6to4 = quatRotate(quat6, qv6to4)

    # create points for actual skeleton lines
    skx21 = nqv2to1[1]  # SKeleton line, X point, sensor 2 to 1
    sky21 = nqv2to1[2]
    skz21 = nqv2to1[3]
    skx32 = nqv3to2[1]  # SKeleton line, X point, sensor 3 to 2
    sky32 = nqv3to2[2]
    skz32 = nqv3to2[3]
    skx43 = nqv4to3[1]  # SKeleton line, X point, sensor 4 to 3
    sky43 = nqv4to3[2]
    skz43 = nqv4to3[3]
    skx54 = nqv5to4[1]  # SKeleton line, X point, sensor 5 to 3
    sky54 = nqv5to4[2]
    skz54 = nqv5to4[3]
    skx64 = nqv6to4[1]  # SKeleton line, X point, sensor 5 to 3
    sky64 = nqv6to4[2]
    skz64 = nqv6to4[3]

    # get 2d "heading" angles of the sensors along the spine
    angle1 = np.arctan2(sky21, skx21)
    angle2 = np.arctan2(sky32, skx32)
    angle3 = np.arctan2(sky43, skx43)

    theta = (angle1 + angle2 + angle3)/3

    return theta


def quatRotate(q, va0q):
    vb0q = qmultiply(q, qmultiply(va0q, qconjugate(q)));
    return vb0q


def qconjugate(q):
    # Pull out the scalar part of our input quaternion.
    q0 = q[0]

    # Pull out the vector part of our input quaternions.
    # qv = q[1:4] # means you want the 1-3 python indexes

    # Calculate the conjugate of this input quaternion for output.
    qstar = np.array([q0, -q[1], -q[2], -q[3]])

    return qstar


def qmultiply(q1, q2):
    # Pull out the scalar part of each of our input quaternions.
    x0 = q1[0]
    y0 = q2[0]

    # Pull out the vector part of each of our input quaternions.
    xv = q1[1:4]
    yv = q2[1:4]

    # Calculate the scalar part of the output quaternion.
    z0 = x0 * y0 - np.dot(xv, yv)

    # Calculate the vector part of the output quaternion.
    zv = x0 * yv + y0 * xv + np.cross(xv, yv)

    # Put them together in a row vector.
    z = np.array([z0, zv[0], zv[1], zv[2]])
    return z


def boxCondition(condition):

    if condition == 'Yes':
        rect.set_linewidth(5)
        rect.set_edgecolor([0.0, 0.6, 0.0])
        rect.set_facecolor([0.5, 0.88, 0.5])

    elif condition == 'No':
        rect.set_linewidth(2)
        rect.set_edgecolor([0.6, 0.0, 0.0])
        rect.set_facecolor([0.95, 0.75, 0.75])
    return


# initialization function
def init():

    line1.set_data([], [])
    line2.set_data([], [])
    line3.set_data([], [])
    line4.set_data([], [])
    line5.set_data([], [])

    line1.set_3d_properties([])
    line2.set_3d_properties([])
    line3.set_3d_properties([])
    line4.set_3d_properties([])
    line5.set_3d_properties([])

    return line1, line2, line3, line4, line5


# animation function: this is called sequentially
def animate(i):
    # -------------- READ DATA AND CREATE VARIABLES -------------- #

    # read data from serial port and convert to usable data
    current_line = ser.readline()
    split = current_line.split()

    # create variables for each quaternion, and time
    time = float(split[0])
    WriteCheck = float(split[25])

    # frame 1
    q1w = float(split[1])
    q1x = float(split[2])
    q1y = float(split[3])
    q1z = float(split[4])
    # frame 2
    q2w = float(split[5])
    q2x = float(split[6])
    q2y = float(split[7])
    q2z = float(split[8])
    # frame 3
    q3w = float(split[9])
    q3x = float(split[10])
    q3y = float(split[11])
    q3z = float(split[12])
    # frame 4
    q4w = float(split[13])
    q4x = float(split[14])
    q4y = float(split[15])
    q4z = float(split[16])
    # frame 5
    q5w = float(split[17])
    q5x = float(split[18])
    q5y = float(split[19])
    q5z = float(split[20])
    # frame 6
    q6w = float(split[21])
    q6x = float(split[22])
    q6y = float(split[23])
    q6z = float(split[24])

    # create arrays of each quaternion
    quat1 = np.array([q1w, q1x, q1y, q1z])
    quat2 = np.array([q2w, q2x, q2y, q2z])
    quat3 = np.array([q3w, q3x, q3y, q3z])
    quat4 = np.array([q4w, q4x, q4y, q4z])
    quat5 = np.array([q5w, q5x, q5y, q5z])
    quat6 = np.array([q6w, q6x, q6y, q6z])

    # create unit vectors representing axes in local frame (for twist need z-axis of sensor 1)
    qvectz = np.array([0.0, 0.0, 0.0, 1.0])

    # reinsert vectors on lines 13-17 here if you want to put them back where they were #

    # ----------- TRANSFORM UNIT VECTORS INTO GLOBAL FRAME FOR EACH SENSOR ----------- #

    # sensor 1
    nqvz1 = quatRotate(quat1, qvectz)  # New Quaternion representation of Vector X for sensor 1

    # create skeleton line transformations
    nqv2to1 = quatRotate(quat2, qv2to1)
    nqv3to2 = quatRotate(quat3, qv3to2)
    nqv4to3 = quatRotate(quat4, qv4to3)
    nqv5to4 = quatRotate(quat5, qv5to4)
    nqv6to4 = quatRotate(quat6, qv6to4)

    # -------------- OFFSET RESULTS -------------- #

    # get offset angle

    # calculate Rotation matrix values based on offset angle
    r1 = np.cos(theta)
    r2 = -np.sin(theta)
    r3 = np.sin(theta)
    r4 = np.cos(theta)

    # -------------- CREATE RELATIVE POSITION POINTS -------------- #

    # First create the axes for all the sensors
    zx1 = nqvz1[1]  # z axis, x point for sensor 1
    zy1 = nqvz1[2]
    zz1 = nqvz1[3]

    # create points for actual skeleton lines
    skx21 = nqv2to1[1]  # SKeleton line, X point, sensor 2 to 1
    sky21 = nqv2to1[2]
    skz21 = nqv2to1[3]
    skx32 = nqv3to2[1]  # SKeleton line, X point, sensor 3 to 2
    sky32 = nqv3to2[2]
    skz32 = nqv3to2[3]
    skx43 = nqv4to3[1]  # SKeleton line, X point, sensor 4 to 3
    sky43 = nqv4to3[2]
    skz43 = nqv4to3[3]
    skx54 = nqv5to4[1]  # SKeleton line, X point, sensor 5 to 3
    sky54 = nqv5to4[2]
    skz54 = nqv5to4[3]
    skx64 = nqv6to4[1]  # SKeleton line, X point, sensor 5 to 3
    sky64 = nqv6to4[2]
    skz64 = nqv6to4[3]

    skx21 = r1*skx21 + r2*sky21
    sky21 = r3*skx21 + r4*sky21
    skx32 = r1*skx32 + r2*sky32
    sky32 = r3*skx32 + r4*sky32
    skx43 = r1*skx43 + r2*sky43
    sky43 = r3*skx43 + r4*sky43
    skx54 = r1*skx54 + r2*sky54
    sky54 = r3*skx54 + r4*sky54
    skx64 = r1*skx64 + r2*sky64
    sky64 = r3*skx64 + r4*sky64

    # -------------- CREATE OFFSETS AND ABSOLUTE POSITION POINTS -------------- #

    # Create Offset variables for readability
    off3x = skx21 + skx32  # offset for sensor 3 lines' position in x direction
    off3y = sky21 + sky32
    off3z = skz21 + skz32
    off4x = off3x + skx43
    off4y = off3y + sky43
    off4z = off3z + skz43

    # line connecting 1 to 2
    line1.set_data([0, skx21], [0, sky21])
    line1.set_3d_properties([0, skz21])

    # line connecting 2 to 3
    line2.set_data([skx21, skx21 + skx32], [sky21, sky21 + sky32])
    line2.set_3d_properties([skz21, skz21 + skz32])

    # line connecting 3 to 4
    line3.set_data([off3x, off3x + skx43], [off3y, off3y + sky43])
    line3.set_3d_properties([off3z, off3z + skz43])

    # line connecting 4 to 5
    line4.set_data([off4x, off4x + skx54], [off4y, off4y + sky54])
    line4.set_3d_properties([off4z, off4z + skz54])

    # line connecting 4 to 6
    line5.set_data([off4x, off4x + skx64], [off4y, off4y + sky64])
    line5.set_3d_properties([off4z, off4z + skz64])

    # -------------- IF STATEMENTS TO CHECK FOR CONDITIONS -------------- #

    # updated back bend included the new top sensor
    vbb = nqv2to1 + nqv3to2 + nqv4to3  # position of sensor 4 (2to1 + 3to2 + 4to3) - pos of sensor 1 (0,0,0)
    bendAngle = np.arctan2(np.sqrt(vbb[1]**2 + vbb[2]**2), vbb[3]) * (180.0 / np.pi)

    # twist calculation
    vtw1 = [zx1, zy1, zz1]
    # 6-5
    vtw2 = [(off4x + skx64) - (off4x + skx54), (off4y + sky64) - (off4y + sky54), (off4z + skz64) - (off4z + skz54)]
    twistAngle = (np.arctan2(np.linalg.norm(np.cross(vtw1, vtw2)), np.dot(vtw1, vtw2)) * (180.0 / np.pi))

    # updated toe touch calculation to include the new top sensor
    vtt1 = nqv4to3[1:4]  # uses 4to3 instead of 3to2 here
    vtt2 = nqv2to1[1:4]
    toeAngle = np.arctan2(np.linalg.norm(np.cross(vtt1, vtt2)), np.dot(vtt1, vtt2)) * (180.0 / np.pi)

    if exercise == 'Toe Touch':
        ttext = 'Toe Touch' + ' = ' + str("%.2f" % toeAngle)
        if toeAngle < 50.0:  # make up some parameters
            line1.set_color([0.0, 0.6, 0.0])
            boxCondition('Yes')

        else:
            line1.set_color([0.6, 0.0, 0.0])
            boxCondition('No')

    elif exercise == 'Back Bend':
        ttext = 'Back Bend' + ' = ' + str("%.2f" % bendAngle)
        if bendAngle > 25.0 and bendAngle < 40.0:
            line2.set_color([0.0, 0.6, 0.0])
            boxCondition('Yes')

        else:
            line2.set_color([0.6, 0.0, 0.0])
            boxCondition('No')

    elif exercise == 'Twist':
        ttext = 'Twist' + ' = ' + str("%.2f" % twistAngle)

        if abs(twistAngle) > 25.0:
            line3.set_color([0.0, 0.6, 0.0])
            line4.set_color([0.0, 0.6, 0.0])
            boxCondition('Yes')

        else:
            line3.set_color([0.6, 0.0, 0.0])
            line4.set_color([0.6, 0.0, 0.0])
            boxCondition('No')
    elif exercise == 'Free':
        ttext = 'Free Mode'

    # -------------- DATA LOGGING AND SCREEN DISPLAY -------------- #

    # plotting angle values on screen
    textbox.set_text(ttext)

    # writing text file

    # create strings of quaternions
    qraw1 = str(q1w) + ', ' + str(q1x) + ', ' + str(q1y) + ', ' + str(q1z)
    qraw2 = str(q2w) + ', ' + str(q2x) + ', ' + str(q2y) + ', ' + str(q2z)
    qraw3 = str(q3w) + ', ' + str(q3x) + ', ' + str(q3y) + ', ' + str(q3z)
    qraw4 = str(q4w) + ', ' + str(q4x) + ', ' + str(q4y) + ', ' + str(q4z)
    qraw5 = str(q5w) + ', ' + str(q5x) + ', ' + str(q5y) + ', ' + str(q5z)
    qraw6 = str(q6w) + ', ' + str(q6x) + ', ' + str(q6y) + ', ' + str(q6z)
    #combine raw quaternion string into one
    writeQuat = qraw1 + ', ' + qraw2 + ', ' + qraw3 + ', ' + qraw4 + ', ' + qraw5 + ', ' + qraw6

    # create strings of raw position xyz points for each sensor
    raw1 = str(skx21) + ', ' + str(sky21) + ', ' + str(skz21)
    raw2 = str(skx21 + skx32) + ', ' + str(sky21 + sky32) + ', ' + str(skz21 + skz32)
    raw3 = str(off3x + skx43) + ', ' + str(off3y + sky43) + ', ' + str(off3z + skz43)
    raw4 = str(off4x + skx54) + ', ' + str(off4y + sky54) + ', ' + str(off4z + skz54)
    raw5 = str(off4x + skx64) + ', ' + str(off4y + sky64) + ', ' + str(off4z + skz64)
    rawz = str(zx1) + ', ' + str(zy1) + ', ' + str(zz1)
    # combine raw data into one string
    writeRaw = raw1 + ', ' + raw2 + ', ' + raw3 + ', ' + raw4 + ', ' + raw5 + ', ' + rawz

    # add time and "conditional" data
    writeVar = str(time) + ', ' + writeQuat + ', ' + writeRaw + \
               ', ' + str(bendAngle) + ', ' + str(twistAngle) + ', ' + str(toeAngle)

    if WriteCheck == 1:
        file.write(writeVar)
        file.write("\n")
    elif WriteCheck == 0:
        x = 1

    return line1, line2, line3, line4, line5


# ------------------ INITIAL VALUES ----------------- #

# # call user input gui
# user_in = Input()
# user_in.mainloop()
#
# # read in values from user input GUI
# exercise = user_in.exercise.get()
# l1 = float(user_in.length1.get())
# l2 = float(user_in.length2.get())
# l3 = float(user_in.length3.get())
# l4 = float(user_in.length4.get())
# l5 = float(user_in.length5.get())

# if you don't want to use the GUI use these (cm)
# exercise = "Free"
# exercise = "Toe Touch"
# exercise = "Back Bend"
exercise = "Twist"
l1 = 13.5
l2 = 13
l3 = 13.5
l4 = 15
l5 = 15

# create vectors representing line connecting each sensor
qv2to1 = np.array([0.0, l1, 0.0, 0.0])  # quaternion vector connecting 2 to 1
qv3to2 = np.array([0.0, l2, 0.0, 0.0])
qv4to3 = np.array([0.0, l3, 0.0, 0.0])
qv5to4 = np.array([0.0, 0.0, l4, 0.0])
qv6to4 = np.array([0.0, 0.0, -l5, 0.0])

# set number of initialization samples
samples = 20

# text file stuff
# for mocap, options are below, take them out of mocap2 folder and into this folder so they work
# names subject to change during mocap visit based on order we do things
# textvar = "0calibration.txt"
# textvar = "1toetouch.txt"
# textvar = "2backbend.txt"
textvar = "3twist.txt"
#textvar = "3twist.txt"

# --------------------- SETUP --------------------- #

file = open(textvar, "w")

# set up the figure, the axis, and the plot element we want to animate
fig = plt.figure(figsize=(7, 4), dpi=100)
fig.suptitle("Backster" + u"\N{TRADE MARK SIGN}")

ax = p3.Axes3D(fig, [0, 0.05, 0.7, 0.95])

# declare text line
ax1 = fig.add_axes([0.75, 0.4, 0.2, 0.2])
ax1.axis('off')  # removes all the lines

# create rectangle for text
rect = patches.Rectangle((0, 0), 1, 1, fc=[1.0, 1.0, 1.0], linewidth=2.0, edgecolor=[0.0, 0.0, 0.0])
ax1.add_patch(rect)
textbox = plt.text(0.5, 0.5, "Initial", horizontalalignment='center', verticalalignment='center')

# must connect to serial port
# baud rate must match that of arduino
ser = serial.Serial("/dev/cu.usbmodem1411", 9600)

# -------------- SETUP LINE WIDTHS AND COLORS -------------- #

# Line from 1 to 2
line1, = ax.plot([], [], [], linewidth=4, color='k')
# Line from 2 to 3
line2, = ax.plot([], [], [], linewidth=4, color='k')
# Line from 3 to 4
line3, = ax.plot([], [], [], linewidth=4, color='k')
# Line from 3 to 5
line4, = ax.plot([], [], [], linewidth=4, color='k')
# Line from 3 to 6
line5, = ax.plot([], [], [], linewidth=4, color='k')

# set axis labels
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

# set axes limits
ax.set_xlim3d([-30, 30])
ax.set_ylim3d([-30, 30])
ax.set_zlim3d([-10, 50])

# -------------- CALL OFFSET AND ANIMATION FUNCTIONS -------------- #

theta = offsetAngle(samples)
# call the animator.  blit=True means only re-draw the parts that have changed.
anim = animation.FuncAnimation(fig, animate, init_func=init,
                               frames=200, interval=20, blit=False)

plt.show()
