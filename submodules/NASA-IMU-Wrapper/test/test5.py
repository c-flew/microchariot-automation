import signal
import time


def timeout(function, t):
    
    def timeout_handler(signum, frame):
        raise Exception("Function call timed out")


    signal.signal(signal.SIGALRM, timeout_handler)
    signal.setitimer(signal.ITIMER_REAL, t)
    
    try:
        function()  # Call the function
    except:
        print("Function call timed out")
    else:
        print("Function call completed within the timeout")

    signal.setitimer(signal.ITIMER_REAL, 0)  # Cancel the timer
    print("Hello")



def my_function():
    # Your function code here
    # This is the code that you want to potentially time out
    while True:
        print("Inside function call")
        time.sleep(0.01)


timeout(my_function, 0.05)

