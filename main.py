from autonav_uwb import UWBTag
from an_secrets import *
import time
import requests
import numpy as np
from numpy import linalg
import math
import subprocess
from jetbot import Robot


# -------- SECTION --------
#      UWB Tag/Anchor Communications
# -------------------------
uwb_tag = UWBTag(port='/dev/ttyTHS1', baudrate=115200, debug=False)
anchors = []
tag_distances_from_anchors = {}

# ------- SECTION --------
#      Motion Controls
# -------------------------
robot = Robot()

def clamp(value: float, min_value: float = -1.0, max_value: float = 1.0) -> float:
    return max(min_value, min(max_value, value))

def move(linear: float, angular: float) -> None:
    global robot
    left = clamp(linear - angular)
    right = clamp(linear + angular)

    robot.set_motors(left, right)


# -------- SECTION --------
#      UWB Math
# -------------------------

def uwb_calculate_coordinates():
    global anchors
    global tag_distances_from_anchors


    filtered_dists = {}

    # Filter out distances that are not in the range of 0-2000
    for k in tag_distances_from_anchors.keys():
        if tag_distances_from_anchors[k] > 0 and tag_distances_from_anchors[k] < 2000:
            filtered_dists[k] = tag_distances_from_anchors[k]

    if len(filtered_dists) < 3:
        return
    
    # Sort by distance
    sorted_dists = sorted(filtered_dists.items(), key=lambda item: item[1])

    a_dupe = []
    distances = []

    b = 42.36565
    m = 1.46323
    # Create final anchor and distance array
    for i in range(len(sorted_dists)):
        # Get the address and distance from sorted array
        this_addr = sorted_dists[i][0]
        this_dist = sorted_dists[i][1] 
        
        # Calibration
        this_dist = (this_dist-b)/m

        # Get anchor corresponding to address
        this_anchor = next((anchor for anchor in anchors if anchor['address'] == this_addr), None)
        this_height = this_anchor['height']

        if this_anchor is None:
            print(f"Anchor not found for address: {this_addr}")
            return
        
        # Caluclate height offset
        dist_sqrt = math.pow(this_dist, 2)
        height_sqrt = math.pow(this_anchor['height'], 2)

        if dist_sqrt >= height_sqrt:
            distances +=  [math.sqrt( dist_sqrt - height_sqrt )]
            a_dupe += [this_anchor]
        else:
            print("distance is less than height.  ABORT")


    if len(distances) >= 4:
        distances = distances[:4]
    elif len(distances) == 3:
        distances = distances[:3]
    else:
        return
    

    A_np = np.zeros([len(a_dupe),2])
    dists_np = np.zeros(len(distances))

    for i in range(len(distances)):
        dists_np[i] = distances[i]

    for i in range(len(a_dupe)):
        A_np[i,0] = a_dupe[i]['pos_x']
        A_np[i,1] = a_dupe[i]['pos_y']

    offset = A_np[0,:]
    A_np = A_np[1:,:] 
    A_np = A_np - offset
    
    # Math Stuff
    y = 0.5*(A_np[:,0]**2 + A_np[:,1]**2 - dists_np[1:]**2 + dists_np[0]**2)


    xtemp = np.matmul(linalg.pinv(A_np),y)
    xtemp += offset

    # WITHOUT KALMAN
    xpos = xtemp[0]
    ypos = xtemp[1]

    print(f"({xpos:.02f}, {ypos:.02f})")

    # send request
    url = API_BASE + "/position"
    data = {
        "address": "DD:DD:DD:DD:DD:DD",
        "pos_x": xpos,
        "pos_y": ypos
    }

    _ = requests.post(url, json=data)


# Callback Functions
def uwb_new_distance(*args, **kwargs):
    global anchors
    global tag_distances_from_anchors
    if anchors is None:
        return
    # print(f"new distance: {kwargs}")
    # print(kwargs)
    tag_distances_from_anchors[kwargs['address']] = kwargs['distance']
    # print(tag_distances_from_anchors)
    # if len(tag_distances_from_anchors) >= 3:
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

    # Set callback functions
    uwb_tag.on(uwb_tag.Event.DISTANCE, uwb_new_distance)
    uwb_tag.on(uwb_tag.Event.CONNECT, uwb_found_anchor)
    uwb_tag.on(uwb_tag.Event.DISCONNECT, uwb_lost_anchor)

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

    print(anchors)
    
if __name__ == "__main__":
    move(0.0, 0.0)
    time.sleep(1)
    move(0.5, 0.0)
    time.sleep(1)
    move(0.5, 90.0)
    time.sleep(1)
    move(0.5, 0.0)
    time.sleep(2)
    move(0.0, 0.0)
    time.sleep(2)



    # init_uwb()


    # try:
    #     uwb_tag.start()

    #     while True:
    #         time.sleep(0.1)
            
    # except KeyboardInterrupt:
    #     uwb_tag.stop()


