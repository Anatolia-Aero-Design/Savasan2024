#!/usr/bin/env python
from scipy.spatial.transform import Rotation as R
from roslibpy import Message
import json
import math

def calculate_speed(x, y, z):
    speed = math.sqrt(x**2 + y**2 + z**2)
    return speed

def mode_guided(mode):
    if mode == True:
        return 0
    else: 
        return 1


def quaternion_to_euler(x, y, z, w):
    quat = [w, x, y, z]  # Quaternion sırası w, x, y, z
    r = R.from_quat(quat)
    euler = r.as_euler('zyx', degrees=True)  # Yaw, Pitch, Roll sırası
    return euler[0], euler[1], euler[2]