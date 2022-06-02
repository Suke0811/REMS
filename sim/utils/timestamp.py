import time
from datetime import datetime


def time_str():
    timestamp = time.time()
    # convert to datetime
    date_time = datetime.fromtimestamp(timestamp)

    # convert timestamp to string in dd-mm-yyyy HH:MM:SS
    return date_time.strftime("%m_%d_%Y_%H_%M_%S")

