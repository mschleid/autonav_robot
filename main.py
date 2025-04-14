from autonav_uwb import UWBTag
from an_secrets import *
import time
import requests
import numpy as np
from numpy import linalg
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise
import math

# -------- SECTION --------
#      KALMAN FILTER
# -------------------------
f = KalmanFilter (dim_x=4, dim_z=2)
xhat = np.array([5., 1., 5., 0.])    # velocity
f.x = xhat
dt = 0.5
f.F = np.array([[1, dt, 0, 0],  # Position_x update
                 [0, 1, 0, 0],  # Velocity_x update
                 [0, 0, 1, dt],  # Position_y update
                 [0, 0, 0, 1]])  # Velocity_y update
f.H = np.array([[1.,0.,0.,0],[0.,0.,1.,0.]])
f.P *= 1
f.R = np.array([[1.,0.], [0.,1.]])


# Generate 2D process noise covariance for each independent dimension
var = 0.12
Q_x = Q_discrete_white_noise(dim=2, dt=dt, var=var)  # For x
Q_y = Q_discrete_white_noise(dim=2, dt=dt, var=var)  # For y

# Construct block-diagonal matrix for full 4D system
f.Q = np.block([
    [Q_x, np.zeros_like(Q_x)],  # Top-left and top-right
    [np.zeros_like(Q_y), Q_y]  # Bottom-left and bottom-right
])

# -------- SECTION --------
#      UWB Tag/Anchor Communications
# -------------------------
uwb_tag = UWBTag(port='/dev/ttyTHS1', baudrate=115200, debug=False)
anchors = []
tag_distances_from_anchors = {}

def uwb_calculate_coordinates():
    global anchors
    global tag_distances_from_anchors

    # Get X and Y positions of anchors
    length = len(anchors)
    print(anchors)
    A = np.zeros([length,2])
    for ix in range(length):
        A[ix,0] = anchors[ix].pos_x
        A[ix,1] = anchors[ix].pos_y

    # Get tag's distances to anchors from input
    i = 0
    dists = np.zeros(length)
    b = 42.36565
    m = 1.46323

    for a in anchors:
        dists[i] = (tag_distances_from_anchors[a.address]-b)/m
        i+=1

    for n in range(0,i):
        dist_sqrt = math.pow(dists[n], 2)
        height_sqrt = math.pow(anchors[n].height, 2)

        if dist_sqrt >= height_sqrt:
            dists[n] = math.sqrt( dist_sqrt - height_sqrt )
        else:
            print("Shit!")
    
    # Math Stuff
    A = A[1:,:]

    y = 0.5*(A[:,0]**2 + A[:,1]**2 - dists[1:]**2 + dists[0]**2)

    xtemp = np.matmul(linalg.pinv(A),y)
    f.predict()
    f.update(xtemp)

    # KALMAN FILTER OUTPUT
    # xpos = f.x[0]
    # ypos = f.x[2]

    # WITHOUT KALMAN
    xpos = xtemp[0]
    ypos = xtemp[1]

    print(xpos, ypos)


# Callback Functions
def uwb_new_distance(*args, **kwargs):
    global tag_distances_from_anchors
    # print(f"new distance: {kwargs}")
    # print(kwargs)
    tag_distances_from_anchors[kwargs['address']] = kwargs['distance']
    print(tag_distances_from_anchors)
    if len(tag_distances_from_anchors) >= 3:
        uwb_calculate_coordinates()

def uwb_found_anchor(*args, **kwargs):
    print(f"found anchor: {args}")

def uwb_lost_anchor(*args, **kwargs):
    global tag_distances_from_anchors
    # print(f"lost anchor: {args}")
    tag_distances_from_anchors.pop(args[0], None)

# Initialzation
def init_uwb():
    global uwb_tag
    global anchors

    # Fetch Anchor Locations from database
    json_response = None
    while True:
        req = requests.get(API_ANCHORS_ALL, 
                        headers={"Content-Type":"application/json", "Authorization": f"Bearer {API_ACCESS_TOKEN}"})
        
        if req.status_code == 200:
            try:
                json_response = req.json()
                anchors = [anchor for anchor in json_response]
                break
            except Exception as e:
                print(f"Error parsing JSON: {e}")

        print("Failed to fetch anchor locations... retrying in 3 seconds", req.status_code, req.text)
        time.sleep(3)    
    
    # correct anchor positions by picking the first point as origin
    origin = anchors[0]
    offset_x, offset_y = (origin['pos_x'], origin['pos_y'])
    
    for anchor in anchors:
        anchor['pos_x'] -= offset_x
        anchor['pos_y'] -= offset_y
    
    # Set callback functions
    uwb_tag.on(uwb_tag.Event.DISTANCE, uwb_new_distance)
    uwb_tag.on(uwb_tag.Event.CONNECT, uwb_found_anchor)
    uwb_tag.on(uwb_tag.Event.DISCONNECT, uwb_lost_anchor)
    
if __name__ == "__main__":
    init_uwb()


    try:
        uwb_tag.start()

        while True:
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        uwb_tag.stop()


