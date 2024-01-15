# Helper functions for segmentation
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import cv2 as cv
import rospy

# Generated color map from the csv file
color_map = {0: {'name': 'unlabeled', 'rgb': [0, 0, 0]},
1: {'name': 'paved-area', 'rgb': [128, 64, 128]},
2: {'name': 'dirt', 'rgb': [130, 76, 0]},
3: {'name': 'grass', 'rgb': [0, 102, 0]},
4: {'name': 'gravel', 'rgb': [112, 103, 87]},
5: {'name': 'water', 'rgb': [28, 42, 168]},
6: {'name': 'rocks', 'rgb': [48, 41, 30]},
7: {'name': 'pool', 'rgb': [0, 50, 89]},
8: {'name': 'vegetation', 'rgb': [107, 142, 35]},
9: {'name': 'roof', 'rgb': [70, 70, 70]},
10: {'name': 'wall', 'rgb': [102, 102, 156]},
11: {'name': 'window', 'rgb': [254, 228, 12]},
12: {'name': 'door', 'rgb': [254, 148, 12]},
13: {'name': 'fence', 'rgb': [190, 153, 153]},
14: {'name': 'fence-pole', 'rgb': [153, 153, 153]},
15: {'name': 'person', 'rgb': [255, 22, 96]},
16: {'name': 'dog', 'rgb': [102, 51, 0]},
17: {'name': 'car', 'rgb': [9, 143, 150]},
18: {'name': 'bicycle', 'rgb': [119, 11, 32]},
19: {'name': 'tree', 'rgb': [51, 51, 0]},
20: {'name': 'bald-tree', 'rgb': [190, 250, 190]},
21: {'name': 'ar-marker', 'rgb': [112, 150, 146]},
22: {'name': 'obstacle', 'rgb': [2, 135, 115]},
23: {'name': 'truck', 'rgb': [0, 102, 0]},
24: {'name': 'bus', 'rgb': [150, 150, 0]},
}

class_map = {'unlabeled': 0,
 'paved-area': 1,
 'dirt': 2,
 'grass': 3,
 'gravel': 4,
 'water': 5,
 'rocks': 6,
 'pool': 7,
 'vegetation': 8,
 'roof': 9,
 'wall': 10,
 'window': 11,
 'door': 12,
 'fence': 13,
 'fence-pole': 14,
 'person': 15,
 'dog': 16,
 'car': 17,
 'bicycle': 18,
 'tree': 19,
 'bald-tree': 20,
 'ar-marker': 21,
 'obstacle': 22,
 'truck':23,
 'bus':24}

# Get the colored mask from the segmented mask
def mask_to_color(mask,map_dict):
    rgb_mask = np.zeros(mask.shape + (3,),dtype=np.uint8)
    for key in np.unique(mask):
        rgb_mask[mask==key] = map_dict[key]['rgb']
    return rgb_mask

def detect_mask(boxes,classes=None):
    '''
    if not set(classes).issubset(class_map.keys()):
        print("Invalid classes")
        return None
    '''
    count = 0
    mask = np.zeros((1080,1440),dtype=np.uint8)         #TODO: add image info as ROS param
    for box in boxes:
        if box.Class in classes:
            mask[box.ymin:box.ymax+1,box.xmin:box.xmax+1] = class_map[box.Class]
            count += 1
    return mask,count
