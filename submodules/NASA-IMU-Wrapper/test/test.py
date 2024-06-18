from AttitudeEstimator import AttitudeEstimatorGPSIMU
from PoseEstimator import PoseEstimator
import time
from squaternion import Quaternion
from IMUService import IMUService
import numpy as np

if __name__ == '__main__':
    
    attitude_estimator = AttitudeEstimatorGPSIMU(freq=40, alpha=0.7, gps_yaw_offset=282)
    pose_estimator = PoseEstimator(attitude_estimator)
    pose_estimator.start()
    while True:
        angles = pose_estimator.get_orientation()
        print(angles)
        time.sleep(0.1)
