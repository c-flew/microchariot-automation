from IMU import IMU
import threading
import time
import numpy as np


class IMUService:

    def __init__(self, freq: float, filter_type="K", use_mag=False):
        self.__thread = threading.Thread(target=self.__update, daemon=True)
        self.__imu = IMU(freq=freq, filter_type=filter_type, use_mag=use_mag)
        self.__freq = freq
        self.__data = [None, None, None, None]
        self.__thread.start()
        print("IMU Thread Started")

    def __update(self):
        print("IMU Service: Calibrating... Please don't touch the IMU")
        self.__imu.calibrate()
        print("IMU Service: Calibration has finished")
        while True:
            start_time = time.time()
            self.__imu.update_data(1/self.__freq)
            self.__data = self.__imu.get_data()
            sleep_time = max(1/self.__freq - (time.time() - start_time), 0)
            if sleep_time == 0:
                print("IMU Service: Decrease filter frequency. Filter cannot run at requested speed")
            time.sleep(sleep_time)
            # print("IMU Service dt: ", time.time() - start_time)
    
    def get_data(self):
        return self.__data

    def reset(self, q=np.array([1.0, 0.0, 0.0, 0.0])):
        self.__imu.reset(q)
