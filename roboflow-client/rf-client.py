#!/usr/bin/env python3
import math
#import ntcore
import time
from roboflowoak import RoboflowOak
import cv2
import numpy as np
import json

h_fov = math.radians(69/2)
h_pixels = 416/2
v_fov = math.radians(41.25/2)
v_pixels = 416/2
scale = h_fov/h_pixels

# x, y, z offsets from 0,0,0 (center of robot x and y, floor z) in that order
offsets = [0, -0.2, 0.5]

def coords_to_position(bbox):
    # scale x coordinate to x position aiming coordinates
    h_a = ((bbox.x - h_pixels) / h_pixels) * h_fov
    x = math.sin(h_a) * bbox.depth / 1000

    # scale y coordinate to z position based aiming coordinates
    v_a = -1 * ((bbox.y - v_pixels) / v_pixels) * v_fov
    z = math.sin(v_a) * bbox.depth / 1000

    # scale depth to y position based aiming coordinates
    y = math.cos(h_a) * bbox.depth / 1000

    bbox.x = x + offsets[0]
    bbox.z = z + offsets[1]
    bbox.y = y + offsets[2]

    return bbox
def update(t0):
    result, frame, raw_frame, depth = rf.detect()
    predictions = result["predictions"]

    predictions = [coords_to_position(prediction) for prediction in predictions if prediction.depth != 0]

    x_coords = [prediction.x for prediction in predictions]
    y_coords = [prediction.y for prediction in predictions]
    z_coords = [prediction.z for prediction in predictions]
    classes = [prediction.class_name for prediction in predictions]

    x_coords_pub.set(x_coords)
    y_coords_pub.set(y_coords)
    z_coords_pub.set(z_coords)
    classes_pub.set(classes)


if __name__ == "__main__":
    nt_inst = ntcore.NetworkTableInstance.getDefault()
    nt_inst.startClient4("Roboflow client")
    nt_inst.setServerTeam(2914)
    rf_table = nt_inst.getTable("rf_data")

    x_coords_pub = rf_table.getDoubleArrayTopic("x_coords").publish()
    y_coords_pub = rf_table.getDoubleArrayTopic("y_coords").publish()
    z_coords_pub = rf_table.getDoubleArrayTopic("z_coords").publish()
    classes_pub = rf_table.getDoubleArrayTopic("classes").publish()

    rf = RoboflowOak(
        model="2023-charged-up", 
        confidence=0.5,
        overlap=0.5,
        version="8",
        api_key="vf99FOKNvqSqONChPKvD",
        advanced_config = {"wide_fov": True},
        rgb=True,
        depth=True,
        device=None,
        blocking=True)

    
    while True:
        #time.sleep(0.1)
        update(time.time())
        time.sleep(0.00001)
