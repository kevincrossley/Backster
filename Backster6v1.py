
import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
import matplotlib.animation as animation
import matplotlib.patches as patches
import serial

'' 'All the initial setup variables that can change'
# Select Exercise
# possible values are "toeTouch", "backBend", "twist"
exercise = "toeTouch"

# create vectors representing line connecting each sensor
qv2to1 = np.array([0.0, 7.0, 0.0, 0.0])  # quaternion vector connecting 2 to 1
qv3to2 = np.array([0.0, 4.0, 0.0, 0.0])
qv4to3 = np.array([0.0, 0.0, 2.0, 2.0])
qv5to3 = np.array([0.0, 0.0, -2.0, 2.0])
qv6to3 = np.array([0.0, 2.0, 0.0, 0.0])

# text file stuff
file = open("DataLog1.txt", "w")

# set up the figure, the axis, and the plot element we want to animate
fig = plt.figure(figsize=(7, 4), dpi=100)

ax = p3.Axes3D(fig, [0, 0.05, 0.7, 0.95])

# declare text line
ax1 = fig.add_axes([0.75, 0.4, 0.2, 0.2])
ax1.axis('off')  # removes all the lines

# create rectangle for text
rect = patches.Rectangle((0, 0), 1, 1, fc=[1.0, 1.0, 1.0], linewidth=2.0, edgecolor=[0.0, 0.0, 0.0])
ax1.add_patch(rect)
textbox = plt.text(0.5, 0.5, "Initial", horizontalalignment='center', verticalalignment='center')

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

# must connect to serial port
# baud rate must match that of arduino
ser = serial.Serial("/dev/cu.usbmodem1411", 9600)

'' ' -----------Setup line widths and colors----------- '

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
    '' ' -----------Read data and create variables----------- '

    # read data from serial port and convert to usable data
    current_line = ser.readline()
    split = current_line.split()

    # create variables for each quaternion, and time
    time = float(split[0])
    WriteCheck = 1  # float(split[25])

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

    # reinsert vectors on lines 13-17 here if you want to put them back where they were #

    '' ' -----------Transform unit vectors into global frame for each sensor----------- '

    # create skeleton line transformations
    nqv2to1 = quatRotate(quat2, qv2to1)
    nqv3to2 = quatRotate(quat3, qv3to2)
    nqv4to3 = quatRotate(quat4, qv4to3)
    nqv5to3 = quatRotate(quat5, qv5to3)
    nqv6to3 = quatRotate(quat6, qv6to3)

    '' ' -----------Create Relative Position Points----------- '

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
    skx53 = nqv5to3[1]  # SKeleton line, X point, sensor 5 to 3
    sky53 = nqv5to3[2]
    skz53 = nqv5to3[3]
    skx63 = nqv6to3[1]  # SKeleton line, X point, sensor 5 to 3
    sky63 = nqv6to3[2]
    skz63 = nqv6to3[3]

    '' ' -----------Create Offsets and Absolute Position Points----------- '

    # Create Offset variables for readability
    off3x = skx21 + skx32  # offset for sensor 3 lines' position in x direction
    off3y = sky21 + sky32
    off3z = skz21 + skz32

    # line connecting 1 to 2
    line1.set_data([0, skx21], [0, sky21])
    line1.set_3d_properties([0, skz21])

    # line connecting 2 to 3
    line2.set_data([skx21, skx21 + skx32], [sky21, sky21 + sky32])
    line2.set_3d_properties([skz21, skz21 + skz32])

    # line connecting 3 to 4
    line3.set_data([off3x, off3x + skx43], [off3y, off3y + sky43])
    line3.set_3d_properties([off3z, off3z + skz43])

    # line connecting 3 to 5
    line4.set_data([off3x, off3x + skx53], [off3y, off3y + sky53])
    line4.set_3d_properties([off3z, off3z + skz53])

    # line connecting 3 to 6
    line5.set_data([off3x, off3x + skx63], [off3y, off3y + sky63])
    line5.set_3d_properties([off3z, off3z + skz63])

    '' ' -----------If Statements to check for some conditions----------- '

    # NEW CALCULATION METHOD BASED ON MOCAP INFO

    # back bend calculation
    vbb = nqv2to1 + nqv3to2  # position of sensor 3 (2to1 + 3to2) - pos of sensor 1 (0,0,0)
    bendAngle = np.arctan2(np.sqrt(vbb[1]**2 + vbb[2]**2), vbb[3]) * (180.0 / np.pi)

    # twist calculation
    vtw1 = [zx1, zy1, zz1]
    vtw2 = [(off3x + skx53) - (off3x + skx43), (off3y + sky53) - (off3y + sky43), (off3z + skz53) - (off3z + skz43)]  # 5-4
    twistAngle = (np.arctan2(np.linalg.norm(np.cross(vtw1, vtw2)), np.dot(vtw1, vtw2)) * (180.0 / np.pi))

    # toe touch calculation
    vtt1 = nqv3to2[1:4]
    vtt2 = nqv2to1[1:4]
    toeAngle = np.arctan2(np.linalg.norm(np.cross(vtt1, vtt2)), np.dot(vtt1, vtt2)) * (180.0 / np.pi)

    if exercise == 'toeTouch':
        ttext = 'Toe Touch' + ' = ' + str("%.2f" % toeAngle)
        if toeAngle < 50.0:  # make up some parameters
            line1.set_color([0.0, 0.6, 0.0])
            boxCondition('Yes')

        else:
            line1.set_color([0.6, 0.0, 0.0])
            boxCondition('No')

    elif exercise == 'backBend':
        ttext = 'Back Bend' + ' = ' + str("%.2f" % bendAngle)
        if bendAngle > 25.0 and bendAngle < 40.0:
            line2.set_color([0.0, 0.6, 0.0])
            boxCondition('Yes')

        else:
            line2.set_color([0.6, 0.0, 0.0])
            boxCondition('No')

    elif exercise == 'twist':
        ttext = 'Twist' + ' = ' + str("%.2f" % twistAngle)

        if abs(twistAngle) > 25.0:
            line3.set_color([0.0, 0.6, 0.0])
            line4.set_color([0.0, 0.6, 0.0])
            boxCondition('Yes')

        else:
            line3.set_color([0.6, 0.0, 0.0])
            line4.set_color([0.6, 0.0, 0.0])
            boxCondition('No')

    '' ' ----------- Data Logging and Screen Display ----------- '

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
    raw4 = str(off3x + skx53) + ', ' + str(off3y + sky53) + ', ' + str(off3z + skz53)
    raw5 = str(off3x + skx63) + ', ' + str(off3y + sky63) + ', ' + str(off3z + skz63)
    # combine raw data into one string
    writeRaw = raw1 + ', ' + raw2 + ', ' + raw3 + ', ' + raw4 + ', ' + raw5

    # add time and "conditional" data
    writeVar = str(time) + ', ' + writeQuat + ', ' + writeRaw + \
               ', ' + str(bendAngle) + ', ' + str(twistAngle) + ', ' + str(toeAngle)

    if WriteCheck == 1:
        file.write(writeVar)
        file.write("\n")
    elif WriteCheck == 0:
        x = 1

    return line1, line2, line3, line4, line5


# Setting the axes properties

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('Backster' + u"\N{TRADE MARK SIGN}")

# set axes limits
ax.set_xlim3d([-10, 10])
ax.set_ylim3d([-10, 10])
ax.set_zlim3d([-10, 10])
# call the animator.  blit=True means only re-draw the parts that have changed.
anim = animation.FuncAnimation(fig, animate, init_func=init,
                               frames=200, interval=20, blit=False)

plt.show()
