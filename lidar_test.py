from rplidar import RPLidar
'''Animates distances and measurment quality'''
from rplidar import RPLidar
import numpy as np

PORT_NAME = '/dev/ttyUSB0'   # Adjust for your system
BAUDRATE = 256000
DMAX = 12000                         # Max distance in mm
SCAN_SIZE = 360                     # Number of measurements per scan
MAP_SIZE_PIXELS = 500
MAP_SIZE_METERS = 10
LIDAR_FREQUENCY = 10.0               # Hz

lidar = RPLidar(PORT_NAME, baudrate=BAUDRATE)
lidar.start_motor()


# ------------------- FUNCTION TO GET FULL 360 SCAN -------------------
def get_full_scan(lidar):
    iterator = lidar.iter_scans()
    scan = next(iterator)  # scan is a list of (quality, angle, distance)
    theta = [t[1] for t in scan]  # extract angles
    r = [t[2] for t in scan]      # extract distances
    return theta, r
            
def get_scan(iterator):
    scan_data = [DMAX] * SCAN_SIZE
    #iterator = lidar.iter_scans()
    scan = next(iterator)
    for _,theta,r in scan:
        index = int(theta) % SCAN_SIZE
        distance = int(r)
        scan_data[index] = distance
    #print(scan_data)
    return scan_data

def collect_all_scans(iterator, max_frames=None):
    all_scans = []
    i = 0
    for scan in lidar.iter_scans():
        scan_data = [DMAX] * SCAN_SIZE
        for _, j, k in scan:
            index = int(j) % SCAN_SIZE
            distance = int(k)
            scan_data[index] = distance
        all_scans.append(scan_data)
        print(i)
        i+=1

    return all_scans

def run():
    try:
        while True:
            iterator = lidar.iter_scans()
            scan_data = get_scan(iterator)
            print(scan_data)
    except KeyboardInterrupt:
        print("KeyboardInterrupt: Exiting...")
    finally:
        lidar.stop()
        lidar.stop_motor()
        lidar.disconnect()
        print("Disconnected from LIDAR.")



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