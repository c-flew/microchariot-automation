import socket
import time
from typing import List

import ahrs.common.quaternion

from IMU import IMU
import numpy as np
from ahrs.filters import AngularRate, Tilt
from squaternion import Quaternion

UDP_IP = '172.20.10.7'
UDP_PORT = 63654

sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
sock.bind((UDP_IP, UDP_PORT))

imu = IMU(filter_type="M", use_mag=True)


def tilt_orientation():
    q_filter = Tilt()
    accel, _, _, = imu.get_calibrated_sensor_data()
    accel *= 9.8 / np.linalg.norm(accel)
    q = ahrs.common.quaternion.Quaternion(q_filter.estimate(accel))
    euler = np.rad2deg(q.to_angles())
    return euler

data, address = sock.recvfrom(4096)
print("received message: %s" % str(data))
print("address is %s" % str(address))

prev_time = time.time()

q = np.array([1.0, 0.0, 0.0, 0.0])
imu.reset(new_orientation_q=q)

while True:

    r_euler = tilt_orientation()
    imu.update_data(dt=time.time() - prev_time)
    print(time.time() - prev_time)
    prev_time = time.time()
    m_q, acc, gyro, _ = imu.get_data()
    m_euler = np.rad2deg(ahrs.common.quaternion.Quaternion(m_q).to_angles())

    
    message = np.concatenate([r_euler, m_euler, acc, gyro]).tobytes()
    sock.sendto(message, address)
