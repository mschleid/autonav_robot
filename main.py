from autonav_uwb import UWBTag
from an_secrets import *
import time
import requests
import numpy as np
from numpy import linalg
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

    a_tmp = []

    for a in anchors:
        if a['address'] in tag_distances_from_anchors:
            a_tmp += [a]

    # Get tag's distances to anchors from input
    i = 0
    dists = np.zeros(len(a_tmp))
    b = 42.36565
    m = 1.46323

    for a in a_tmp:
        distance = tag_distances_from_anchors[a['address']]
        if distance <= 2000:
            dists[i] = (distance-b)/m
            i+=1

    if i < 3:
        print("Not enough anchors to calculate position")
        return
    
    print("Distances to anchors:")
    print(dists)

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
    global anchors
    global tag_distances_from_anchors
    if anchors is None:
        return
    # print(f"new distance: {kwargs}")
    print(kwargs)
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
    
    # correct anchor positions by picking the first point as origin
    origin = anchors[0]
    offset_x, offset_y = (origin['pos_x'], origin['pos_y'])
    
    for anchor in anchors:
        anchor['pos_x'] -= offset_x
        anchor['pos_y'] -= offset_y

    # print(anchors)
    return

    # setup numpy arrays
    global A_np

    length_anchors = len(anchors)
    A_np = np.zeros([length_anchors,2])

    for i in range(length_anchors):
        A_np[i,0] = anchors[i]['pos_x']
        A_np[i,1] = anchors[i]['pos_y']

    print(A_np)

    A_np = A_np[1:,:]

    print(A_np)
    
if __name__ == "__main__":
    init_uwb()


    try:
        uwb_tag.start()

        while True:
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        uwb_tag.stop()


