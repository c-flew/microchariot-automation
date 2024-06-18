import time
from threading import Thread


wake_up_errors = 0
duration_errors = 0

def busy_wait(t):
    start = time.time()
    while time.time() - start < t:
        pass


def simulated_task_loop(task_time, frequency, t_num):
    global wake_up_errors, duration_errors
    expected_wake_time = time.time()
    while True:
        start = time.time()
        offset_time = start - expected_wake_time
        if t_num == 0:
            print("=" * 20)
            print(offset_time)
        if offset_time > 0.002:
            wake_up_errors += 1
            print("Thread %d is not waking up consistently. Unable to meet specified frequency" % t_num)
        busy_wait(task_time)
        sleep_time = max(1/frequency - (time.time() - start) - offset_time, 0)
        expected_wake_time += 1/frequency
        if sleep_time == 0:
            duration_errors += 1
            print("Thread %d taking longer than expected. Unable to meet specified frequency" % t_num)
        if t_num == 0:
            print(sleep_time)
        time.sleep(max(sleep_time - 0.001, 0))


if __name__ == "__main__":

    num_threads = 9
    task_time = 0.002
    frequency = 50
    
    t = None
    for i in range(num_threads):
        t = Thread(target=simulated_task_loop, args=(task_time, frequency, i), daemon=True)
        t.start()
    
    time.sleep(30)
    print("Wake Up Errors: " + str(wake_up_errors) + " Duration Errors: " + str(duration_errors))
