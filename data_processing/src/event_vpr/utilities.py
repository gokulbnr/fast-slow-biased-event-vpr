import os 
import numpy as np
import random
import datetime
import logging


def data_from_dirpath(directory_path):
    pattern = r'(\d+\.\d+)_(\d+\.\d+)_(\d+\.\d+)\.png'
    odom = []
    for filename in sorted(os.listdir(directory_path)):
        if filename.lower().endswith('.png'):
            filename = os.path.basename(filename).split('.png')[0]
            timestamp = float(filename.split('_')[0])
            x = float(filename.split('_')[1])
            y = float(filename.split('_')[2])
            odom.append([timestamp, x, y])
    return np.array(odom)

def ros_data_from_dirpath(directory_path):
    pattern = r'(\d)_(\d+\.\d+)_(\d+\.\d+)_(\d+\.\d+)\.png'
    odom = []
    for filename in sorted(os.listdir(directory_path)):
        if filename.lower().endswith('.png'):
            filename = os.path.basename(filename).split('.png')[0]
            timestamp = float(filename.split('_')[0])
            x = float(filename.split('_')[1])
            y = float(filename.split('_')[2])
            odom.append([timestamp, x, y])
    return np.array(odom)

def generate_random_pixel_locations(image_width, image_height, num_locations):
    pixel_locations = []
    for _ in range(num_locations):
        x = random.randint(0, image_width - 1)
        y = random.randint(0, image_height - 1)
        pixel_locations.append([x, y])
    return pixel_locations

def get_system_timestamp():
    current_datetime = datetime.datetime.now()
    timestamp = current_datetime.strftime("%H:%M:%S %d-%m-%Y")
    filename = f"{timestamp}"
    return filename

# Define a function that logs parameters
def log_parameters(param, param_label='Parameter'):
    logger = logging.getLogger(__name__)
    # Create a log message with the parameters
    log_message = f'{param_label}: {param}'
    logger.info(log_message)