from GPS import GPS
from IMUService import IMUService
import time

if __name__ == "__main__":
    gps = GPS()
    gps.acquire_gps_fix(-1)
    avg_time = 0
    count = 0
    while True:
        start = time.monotonic()
        data = gps.get_data()
        avg_time = (avg_time * count + (time.monotonic() - start)) / (count + 1)
        count += 1
        print(data[3], avg_time)
        time.sleep(0.1)

