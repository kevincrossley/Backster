
import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
import matplotlib.animation as animation
import serial


def quatRotate(q,va0q):

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


def qmultiply(q1,q2):

    # Pull out the scalar part of each of our input quaternions.
    x0 = q1[0]
    y0 = q2[0]

    # Pull out the vector part of each of our input quaternions.
    xv = q1[1:4]
    yv = q2[1:4]

    # Calculate the scalar part of the output quaternion.
    z0 = x0*y0 - np.dot(xv, yv)

    # Calculate the vector part of the output quaternion.
    zv = x0*yv + y0*xv + np.cross(xv, yv)

    # Put them together in a row vector.
    z = np.array([z0,zv[0],zv[1],zv[2]])
    return z


# must connect to serial port
# baud rate must match that of arduino
ser = serial.Serial("/dev/cu.usbmodem1411", 9600)

'' ' -----------Setup line widths and colors----------- '
# set up the figure, the axis, and the plot element we want to animate
fig = plt.figure()
ax = p3.Axes3D(fig)

# Lines for sensor 1's axes and line to 2
line1, = ax.plot([], [], [], linewidth=2, color='r')
line2, = ax.plot([], [], [], linewidth=2, color='g')
line3, = ax.plot([], [], [], linewidth=2, color='b')
line4, = ax.plot([], [], [], linewidth=4, color='k')

# Lines for sensor 2's axes and line to 3
line5, = ax.plot([], [], [], linewidth=2, color='r')
line6, = ax.plot([], [], [], linewidth=2, color='g')
line7, = ax.plot([], [], [], linewidth=2, color='b')
line8, = ax.plot([], [], [], linewidth=4, color='k')

# Lines for sensor 3's axes and line to 4 and 5
line9, = ax.plot([], [], [], linewidth=2, color='r')
line10, = ax.plot([], [], [], linewidth=2, color='g')
line11, = ax.plot([], [], [], linewidth=2, color='b')
line12, = ax.plot([], [], [], linewidth=4, color='k')
line13, = ax.plot([], [], [], linewidth=4, color='k')

# Lines for sensor 4's axes
line14, = ax.plot([], [], [], linewidth=2, color='r')
line15, = ax.plot([], [], [], linewidth=2, color='g')
line16, = ax.plot([], [], [], linewidth=2, color='b')

# Lines for sensor 5's axes
line17, = ax.plot([], [], [], linewidth=2, color='r')
line18, = ax.plot([], [], [], linewidth=2, color='g')
line19, = ax.plot([], [], [], linewidth=2, color='b')


# initialization function
def init():
    line1.set_data([], [])
    line2.set_data([], [])
    line3.set_data([], [])
    line4.set_data([], [])
    line5.set_data([], [])
    line6.set_data([], [])
    line7.set_data([], [])
    line8.set_data([], [])
    line9.set_data([], [])
    line10.set_data([], [])
    line11.set_data([], [])
    line12.set_data([], [])
    line13.set_data([], [])
    line14.set_data([], [])
    line15.set_data([], [])
    line16.set_data([], [])
    line17.set_data([], [])
    line18.set_data([], [])
    line19.set_data([], [])

    line1.set_3d_properties([])
    line2.set_3d_properties([])
    line3.set_3d_properties([])
    line4.set_3d_properties([])
    line5.set_3d_properties([])
    line6.set_3d_properties([])
    line7.set_3d_properties([])
    line8.set_3d_properties([])
    line9.set_3d_properties([])
    line10.set_3d_properties([])
    line11.set_3d_properties([])
    line12.set_3d_properties([])
    line13.set_3d_properties([])
    line14.set_3d_properties([])
    line15.set_3d_properties([])
    line16.set_3d_properties([])
    line17.set_3d_properties([])
    line18.set_3d_properties([])
    line19.set_3d_properties([])

    return line1, line2, line3, line4, line5, line6, line7, line8, line9, line10, \
           line11, line12, line13, line14, line15, line16, line17, line18, line19


# animation function: this is called sequentially
def animate(i):

    '' ' -----------Read data and create variables----------- '

    # read data from serial port and convert to usable data
    current_line = ser.readline()
    split = current_line.split()

    # create variables for each quaternion, and time
    time = float(split[0])
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

    # create arrays of each quaternion
    quat1 = np.array([q1w, q1x, q1y, q1z])
    quat2 = np.array([q2w, q2x, q2y, q2z])
    quat3 = np.array([q3w, q3x, q3y, q3z])
    quat4 = np.array([q4w, q4x, q4y, q4z])
    quat5 = np.array([q5w, q5x, q5y, q5z])

    # create unit vectors representing axes in local frame
    qvectx = np.array([0.0, 1.0, 0.0, 0.0])
    qvecty = np.array([0.0, 0.0, 1.0, 0.0])
    qvectz = np.array([0.0, 0.0, 0.0, 1.0])

    # create vectors representing line connecting each sensor
    qv2to1 = np.array([0.0, 5.0, 0.0, 0.0]) # quaternion vector connecting 2 to 1
    qv3to2 = np.array([0.0, 5.0, 0.0, 0.0])
    qv4to3 = np.array([0.0, 2.0, 2.0, 0.0])
    qv5to3 = np.array([0.0, -2.0, 2.0, 0.0])

    '' ' -----------Transform unit vectors into global frame for each sensor----------- '

    # First create the axes for all the sensors
    # can remove this whole part later if we don't want them

    # sensor 1
    nqvx1 = quatRotate(quat1, qvectx) # New Quaternion representation of Vector X for sensor 1
    nqvy1 = quatRotate(quat1, qvecty)
    nqvz1 = quatRotate(quat1, qvectz)
    # sensor 2
    nqvx2 = quatRotate(quat2, qvectx) # New Quaternion representation of Vector X for sensor 2
    nqvy2 = quatRotate(quat2, qvecty)
    nqvz2 = quatRotate(quat2, qvectz)
    # sensor 3
    nqvx3 = quatRotate(quat3, qvectx) # New Quaternion representation of Vector X for sensor 3
    nqvy3 = quatRotate(quat3, qvecty)
    nqvz3 = quatRotate(quat3, qvectz)
    # sensor 4
    nqvx4 = quatRotate(quat4, qvectx) # New Quaternion representation of Vector X for sensor 4
    nqvy4 = quatRotate(quat4, qvecty)
    nqvz4 = quatRotate(quat4, qvectz)
    # sensor 5
    nqvx5 = quatRotate(quat5, qvectx) # New Quaternion representation of Vector X for sensor 5
    nqvy5 = quatRotate(quat5, qvecty)
    nqvz5 = quatRotate(quat5, qvectz)

    # now create skeleton line transformations
    nqv2to1 = quatRotate(quat2, qv2to1)
    nqv3to2 = quatRotate(quat3, qv3to2)
    nqv4to3 = quatRotate(quat4, qv4to3)
    nqv5to3 = quatRotate(quat5, qv5to3)

    '' ' -----------Create Relative Position Points----------- '
    # First create the axes for all the sensors
    # can remove this whole part later if we don't want them

    xx1 = nqvx1[1] # read as x axis, x point for sensor 1
    xy1 = nqvx1[2]
    xz1 = nqvx1[3]
    yx1 = nqvy1[1] # y axis, x point for sensor 1
    yy1 = nqvy1[2]
    yz1 = nqvy1[3]
    zx1 = nqvz1[1] # z axis, x point for sensor 1
    zy1 = nqvz1[2]
    zz1 = nqvz1[3]

    xx2 = nqvx2[1] # read as x axis, x point for sensor 2
    xy2 = nqvx2[2]
    xz2 = nqvx2[3]
    yx2 = nqvy2[1] # y axis, x point for sensor 2
    yy2 = nqvy2[2]
    yz2 = nqvy2[3]
    zx2 = nqvz2[1] # z axis, x point for sensor 2
    zy2 = nqvz2[2]
    zz2 = nqvz2[3]

    xx3 = nqvx3[1] # read as x axis, x point for sensor 3
    xy3 = nqvx3[2]
    xz3 = nqvx3[3]
    yx3 = nqvy3[1] # y axis, x point for sensor 3
    yy3 = nqvy3[2]
    yz3 = nqvy3[3]
    zx3 = nqvz3[1] # z axis, x point for sensor 3
    zy3 = nqvz3[2]
    zz3 = nqvz3[3]

    xx4 = nqvx4[1] # read as x axis, x point for sensor 4
    xy4 = nqvx4[2]
    xz4 = nqvx4[3]
    yx4 = nqvy4[1] # y axis, x point for sensor 4
    yy4 = nqvy4[2]
    yz4 = nqvy4[3]
    zx4 = nqvz4[1] # z axis, x point for sensor 4
    zy4 = nqvz4[2]
    zz4 = nqvz4[3]

    xx5 = nqvx5[1] # read as x axis, x point for sensor 5
    xy5 = nqvx5[2]
    xz5 = nqvx5[3]
    yx5 = nqvy5[1] # y axis, x point for sensor 5
    yy5 = nqvy5[2]
    yz5 = nqvy5[3]
    zx5 = nqvz5[1] # z axis, x point for sensor 5
    zy5 = nqvz5[2]
    zz5 = nqvz5[3]

    # now create points for actual skeleton lines
    skx21 = nqv2to1[1] # SKeleton line, X point, sensor 2 to 1
    sky21 = nqv2to1[2]
    skz21 = nqv2to1[3]
    skx32 = nqv3to2[1] # SKeleton line, X point, sensor 3 to 2
    sky32 = nqv3to2[2]
    skz32 = nqv3to2[3]
    skx43 = nqv4to3[1] # SKeleton line, X point, sensor 4 to 3
    sky43 = nqv4to3[2]
    skz43 = nqv4to3[3]
    skx53 = nqv5to3[1] # SKeleton line, X point, sensor 5 to 3
    sky53 = nqv5to3[2]
    skz53 = nqv5to3[3]

    '' ' -----------Create Offsets for Absolute Position----------- '

    # Create Offset variables for readability
    off3x = skx21 + skx32 # offset for sensor 3 lines' position in x direction
    off3y = sky21 + sky32
    off3z = skz21 + skz32
    off4x = skx21 + skx32 + skx43 # offset for sensor 4 lines' position in x direction
    off4y = sky21 + sky32 + sky43
    off4z = skz21 + skz32 + skz43
    off5x = skx21 + skx32 + skx53 # offset for sensor 3 lines' position in x direction
    off5y = sky21 + sky32 + sky53
    off5z = skz21 + skz32 + skz53

    # lines from sensor 1
    line1.set_data([0, xx1], [0, xy1])
    line2.set_data([0, yx1], [0, yy1])
    line3.set_data([0, zx1], [0, zy1])
    line4.set_data([0, skx21], [0, sky21])

    line1.set_3d_properties([0, xz1])
    line2.set_3d_properties([0, yz1])
    line3.set_3d_properties([0, zz1])
    line4.set_3d_properties([0, skz21])

    # lines from sensor 2
    line5.set_data([skx21, skx21 + xx2], [sky21, sky21 + xy2])
    line6.set_data([skx21, skx21 + yx2], [sky21, sky21 + yy2])
    line7.set_data([skx21, skx21 + zx2], [sky21, sky21 + zy2])
    line8.set_data([skx21, skx21 + skx32], [sky21, sky21 + sky32])

    line5.set_3d_properties([skz21, skz21 + xz2])
    line6.set_3d_properties([skz21, skz21 + yz2])
    line7.set_3d_properties([skz21, skz21 + zz2])
    line8.set_3d_properties([skz21, skz21 + skz32])

    # lines from sensor 3
    line9.set_data([off3x, off3x + xx3], [off3y, off3y + xy3])
    line10.set_data([off3x, off3x + yx3], [off3y, off3y + yy3])
    line11.set_data([off3x, off3x + zx3], [off3y, off3y + zy3])
    line12.set_data([off3x, off3x + skx43], [off3y, off3y + sky43])
    line13.set_data([off3x, off3x + skx53], [off3y, off3y + sky53])

    line9.set_3d_properties([off3z, off3z + xz3])
    line10.set_3d_properties([off3z, off3z + yz3])
    line11.set_3d_properties([off3z, off3z + zz3])
    line12.set_3d_properties([off3z, off3z + skz43])
    line13.set_3d_properties([off3z, off3z + skz53])

    # line from sensor 4
    line14.set_data([off4x, off4x + xx4], [off4y, off4y + xy4])
    line15.set_data([off4x, off4x + yx4], [off4y, off4y + yy4])
    line16.set_data([off4x, off4x + zx4], [off4y, off4y + zy4])

    line14.set_3d_properties([off4z, off4z + xz4])
    line15.set_3d_properties([off4z, off4z + yz4])
    line16.set_3d_properties([off4z, off4z + zz4])

    # lines from sensor 5
    line17.set_data([off5x, off5x + xx5], [off5y, off5y + xy5])
    line18.set_data([off5x, off5x + yx5], [off5y, off5y + yy5])
    line19.set_data([off5x, off5x + zx5], [off5y, off5y + zy5])

    line17.set_3d_properties([off5z, off5z + xz5])
    line18.set_3d_properties([off5z, off5z + yz5])
    line19.set_3d_properties([off5z, off5z + zz5])

    '' ' -----------If Statements to check for some conditions----------- '

    # sin(z height / link length) gives pitch angle
    pitch2 = np.arcsin(skz21 / 5.0) * (180.0 / np.pi)

    if pitch2 > -25 and pitch2 < 45:  # make up some parameters
        line4.set_color([0.0, 0.5, 0.0])

    else:
        line4.set_color([0.5, 0.0, 0.0])

    return line1, line2, line3, line4, line5, line6, line7, line8, line9, line10, \
           line11, line12, line13, line14, line15, line16, line17, line18, line19


# Setting the axes properties

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('3D Test')

# set axes limits
ax.set_xlim3d([-15, 15])
ax.set_ylim3d([-15, 15])
ax.set_zlim3d([-15, 15])
# call the animator.  blit=True means only re-draw the parts that have changed.
anim = animation.FuncAnimation(fig, animate, init_func=init,
                               frames=200, interval=20, blit=False)

plt.show()
