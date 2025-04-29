from rplidar import RPLidar
'''Animates distances and measurment quality'''
from rplidar import RPLidar
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.animation as animation

PORT_NAME = '/dev/ttyUSB0'
BAUDRATE = 256000
DMAX = 4000
IMIN = 0
IMAX = 50

def update_line(num, iterator, line):
    scan = next(iterator)
    offsets = np.array([(np.radians(meas[1]), meas[2]) for meas in scan])
    line.set_offsets(offsets)
    intens = np.array([meas[0] for meas in scan])
    line.set_array(intens)
    return line,

def run():
    lidar = RPLidar(port=PORT_NAME, baudrate=BAUDRATE)
    fig = plt.figure()
    ax = plt.subplot(111, projection='polar')
    line = ax.scatter([0, 0], [0, 0], s=5, c=[IMIN, IMAX],
                           cmap=plt.cm.Greys_r, lw=0)
    ax.set_rmax(DMAX)
    ax.grid(True)
    # Remove axis lines, ticks, and labels
    ax.set_axis_off()          # Removes frame, ticks, and background

    # Optional: make background white
    ax.set_facecolor('white')

    iterator = lidar.iter_scans()
    ani = animation.FuncAnimation(fig, update_line,
        fargs=(iterator, line), interval=50)
    plt.show()
    lidar.stop()
    lidar.stop_motor()
    lidar.disconnect()


if __name__ == '__main__':
    run()
# lidar = RPLidar('/dev/tty.usbserial-10', baudrate=256000)

# info = lidar.get_info()
# print(info)

# health = lidar.get_health()
# print(health)

# for i, scan in enumerate(lidar.iter_scans()):
#     print('%d: Got %d measurments' % (i, len(scan)))
#     if i > 10:
#         break

# lidar.stop()
# lidar.stop_motor()
# lidar.disconnect()