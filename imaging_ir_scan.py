"""
Receive 3D scanner data  and plot it such that the scanned letter appears.
"""

import matplotlib.pyplot as plt
import serial

# serial constants
PORT = "/dev/ttyACM0"
BAUDRATE = 9600
serialPort = serial.Serial(PORT, BAUDRATE, timeout=1)

# empty arrays for serial data and subsequent plot
ir_vals = []
pt_coords = []
xy_coords = [[], []]
colors = []

# pan/tilt mapping and coloring
pt_mins = [45, 0]  # pan min, tilt min
pt_maxes = [135, 1]  # pan max, tilt max
THRESHOLD = 300  # from calibration curve
TOTAL_PTS = 90 * 90 # pan degrees * tilt degrees
CLR1 = 10
CLR2 = 100


# helper function
def map_coord(val, min_val, max_val):
    """
    Map a coordinate value to a 0.0 - 1.0 scale. Because pan and tilt will have
    a different total number of points, this may warp
    """
    return (val - min_val) / (max_val - min_val)


# receive data from Arduino
while len(pt_coords) < TOTAL_PTS:
    lineOfData = serialPort.readline().decode()
    # there is data
    if len(lineOfData) > 0 and len(lineOfData.split(",")) == 3:
        print(lineOfData)
        ir, pan, tilt = lineOfData.split(",")
        ir_vals.append(int(ir))
        pt_coords.append([int(pan), int(tilt)])
    else:
        print(f"No line found. Current point count: {len(pt_coords)}")

# map points to an x/y frame, determine color
for i, point in enumerate(pt_coords):
    # create mapped xy points
    for j in range(2):
        xy_val = map_coord(point[j], pt_mins[j], pt_maxes[j])
        xy_coords[j].append(xy_val)

    # determine correct color for point (this will display the letter)
    if ir_vals[i] > THRESHOLD:
        colors.append(CLR1)
    else:
        colors.append(CLR2)


# plot points
plt.scatter(xy_coords[0], xy_coords[1], c=colors)
plt.show()