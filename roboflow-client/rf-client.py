#!/usr/bin/env python3

import ntcore
import time
from roboflowoak import RoboflowOak
import cv2
import numpy as np
import json

def update(t0):
    result, frame, raw_frame, depth = rf.detect()
    predictions = result["predictions"]
    #depth = result["depth"]

    #target_name_pub.setDefault()
    #if len(predictions) > 0:
        #target_name_pub.set(predictions[0]["x"])

    t = time.time() - t0
    fps_pub.set(1 / t)

    cv2.imshow("frame", frame)


if __name__ == "__main__":
    nt_inst = ntcore.NetworkTableInstance.getDefault()
    nt_inst.startClient4("Roboflow client")
    nt_inst.setServerTeam(2914)

    target_name_pub = nt_inst.getStringTopic("/roboflow/targetName").publish(ntcore.PubSubOptions(periodic=0.1))
    fps_pub = nt_inst.getDoubleTopic("/roboflow/fps").publish(ntcore.PubSubOptions(periodic=0.1))

    rf = RoboflowOak(
        model="2023-charged-up", 
        confidence=0.3, 
        overlap=0.5,
        version="5",
        api_key="vf99FOKNvqSqONChPKvD",
        rgb=True,
        depth=True,
        device=None,
        blocking=True)

    nt_inst.startClient4("Roboflow client")
    
    while True:
        #time.sleep(0.1)
        update(time.time())

        if cv2.waitKey(1) == ord('q'):
            break
