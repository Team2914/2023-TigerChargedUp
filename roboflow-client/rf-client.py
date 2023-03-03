#!/usr/bin/env python3

import ntcore
import time
from roboflowoak import RoboflowOak
import cv2
import numpy as np

def update(t0):
    result, frame, raw_frame, depth = rf.detect()
    predictions = result["predictions"]
    depth = result["depth"]

    target_name_pub.setDefault("none", t0)
    target_name_pub.set(predictions[0]["class"])

    t = time.time() - t0
    fps_pub.set(1 / t)


if __name__ == "__rf-client__":
    nt_inst = ntcore.NetworkTableInstance.getDefault()
    nt_inst.startClient4("Roboflow client")
    nt_inst.setServerTeam(2914)

    target_name_pub = nt_inst.getStringTopic("/roboflow/targetName").publish(ntcore.PubSubOptions(periodic=0.1))
    fps_pub = nt_inst.getDoubleTopic("/roboflow/fps").publish(ntcore.PubSubOptions(periodic=0.1))

    rf = RoboflowOak(
        model="", 
        confidence=0.05, 
        overlap=0.5,
        version="",
        api_key="",
        rgb=True,
        depth=True,
        device=None,
        blocking=True)

    nt_inst.startClient4("Roboflow client")
    
    while True:
        time.sleep(0.1)
        update(time.time())
