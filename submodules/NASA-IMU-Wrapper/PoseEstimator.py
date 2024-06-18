from AttitudeEstimator import AttitudeEstimatorGPSIMU
from multiprocessing import Process, Pipe
import numpy as np

class PoseEstimator:

    def __init__(self, attitude_estimator: AttitudeEstimatorGPSIMU):

        self.__attitude_estimator = attitude_estimator
        
        # self.__orientation_arr = RawArray('d', 3)
        # self.__orientation = np.frombuffer(self.__orientation_arr, dtype=double, count=len(self.__orientation_arr))
        
        self.__request_data = Pipe()
        self.__receive_data = Pipe()

        self.__pose_estimator_process = Process(target=self.__update_pose, args=(attitude_estimator, self.__request_data[0], self.__receive_data[1]), daemon=True)


    def __update_pose(self, attitude_estimator, data_requested, send_data):
        
        attitude_estimator.start()
        print("Attitude Estimator has started", flush=True)
        while True:
            data_requested.recv()
            data = attitude_estimator.get_orientation()
            send_data.send(data)

    
    def start(self):
        self.__pose_estimator_process.start()
        print("Pose Estimator Process Started!", flush=True)


    def get_orientation(self):
        self.__request_data[1].send(True)
        if self.__receive_data[0].poll(timeout=0.01):
            return self.__receive_data[0].recv()
        return np.zeros(3)

    def reset(self, reset_orientation=np.zeros(3), gps_yaw_offset=0):
        self.__attitude_estimator.reset(reset_orientation, gps_yaw_offset)
