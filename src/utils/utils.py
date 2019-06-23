import time

def execute_func_from_t_secs(func, time_secs):
    """
    Execute a function for t_secs.
    """
    start_time = time.time()
    curr_time = time.time()
    while(((curr_time - start_time) % 60) <= time_secs):
        func()
        curr_time = time.time()
