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
#      UWB Tag/Anchor Communications
# -------------------------
uwb_tag = UWBTag(port='/dev/ttyTHS1', baudrate=115200, debug=False)
anchors = []
tag_distances_from_anchors = {}

# -------- SECTION --------
#      UWB Math
# -------------------------
A_np = None

def uwb_calculate_coordinates():
    global anchors
    global tag_distances_from_anchors
    global A_np

    # Get tag's distances to anchors from input
    i = 0
    dists = np.zeros(len(anchors))
    b = 42.36565
    m = 1.46323

    for a in anchors:
        dists[i] = (tag_distances_from_anchors[a['address']]-b)/m
        i+=1

    for n in range(0,i):
        dist_sqrt = math.pow(dists[n], 2)
        height_sqrt = math.pow(anchors[n]['height'], 2)

        if dist_sqrt >= height_sqrt:
            dists[n] = math.sqrt( dist_sqrt - height_sqrt )
        else:
            print("Shit!")
    
    # Math Stuff
    y = 0.5*(A_np[:,0]**2 + A_np[:,1]**2 - dists[1:]**2 + dists[0]**2)

    xtemp = np.matmul(linalg.pinv(A_np),y)

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

    print(anchors)

    exit()
    
    # setup numpy arrays
    global A_np

    length_anchors = len(anchors)
    A_np = np.zeros([length_anchors,2])

    for i in range(length_anchors):
        A_np[i,0] = anchors[i]['pos_x']
        A_np[i,1] = anchors[i]['pos_y']

    A_np= A_np[1:,:]

    print(A_np)
    
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


