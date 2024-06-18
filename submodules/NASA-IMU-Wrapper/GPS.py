import adafruit_gps
import time
import board
import serial


class GPS:

    def __init__(self):   
        # setup sensor hardware
        self.__uart = serial.Serial("/dev/ttyUSB0", baudrate=9600, timeout=10)
        self.__gps = adafruit_gps.GPS(self.__uart, debug=False)
        self.__gps.send_command(b"PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0")
        self.__gps.send_command(b"PMTK220,500")
    
        # data
        self.__location = [None, None, None, None]
        self.__track_angle_deg = None
        self.__speed_knots = None
        self.__new_message_receved = False

    def get_data(self):
        self.__new_message_received = self.__gps.update()

        if self.__new_message_received:
            self.__location = [self.__gps.latitude_degrees, self.__gps.latitude_minutes, self.__gps.longitude_degrees, self.__gps.longitude_minutes]
            self.__track_angle_deg = self.__gps.track_angle_deg
            self.__speed_knots = self.__gps.speed_knots 
        return self.__location, self.__track_angle_deg, self.__speed_knots, self.__new_message_received


    def acquire_gps_fix(self, timeout=-1):
        start = time.time()
        while not self.__gps.has_fix:
            if time.time() - start > timeout and timeout >= 0:
                break
            self.__gps.update()
            print("GPS: Waiting for fix...")
            time.sleep(1)
        
        has_fix = self.__gps.has_fix

        if has_fix:
            print("GPS: Fix Acquired")
        else:
            print("GPS: Failed to acquire fix")
        
        return has_fix





