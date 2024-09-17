import cv2
import os
import numpy as np
import argparse
from matplotlib import pyplot as plt


def remove_hot_pixels(image_path, output_directory, threshold=100):
    red_non_zero_counts = []
    blue_non_zero_counts = []
    for path in image_path[:]:
        image = cv2.imread(path)
        red_num_non_zero_pixels = cv2.countNonZero(image[:,:,0])
        red_non_zero_counts.append(red_num_non_zero_pixels)
        blue_num_non_zero_pixels = cv2.countNonZero(image[:,:,2])
        blue_non_zero_counts.append(blue_num_non_zero_pixels)

    red_mean = np.mean(red_non_zero_counts)
    red_std_dev = np.std(red_non_zero_counts)
    red_threshold = red_mean + 3 * red_std_dev

    blue_mean = np.mean(blue_non_zero_counts)
    blue_std_dev = np.std(blue_non_zero_counts)
    blue_threshold = blue_mean + 3 * blue_std_dev

    for path in image_path[:]:
        image = cv2.imread(path)
        red_num_non_zero_pixels = cv2.countNonZero(image[:,:,0])
        blue_num_non_zero_pixels = cv2.countNonZero(image[:,:,2])
        if red_num_non_zero_pixels < red_threshold and blue_num_non_zero_pixels < blue_threshold:
            channels = cv2.split(image)
            equalized_channels = [cv2.equalizeHist(channel) for channel in channels]
            equalized_img = cv2.merge(equalized_channels)
            cv2.imwrite(os.path.join(output_directory, os.path.basename(path)), equalized_img)


def main(directory_path, output_directory):
    # Get all image paths in the specified directory
    image_paths = [os.path.join(directory_path, file) for file in os.listdir(directory_path) if file.endswith(('.png', '.jpg', '.jpeg'))]

    if not image_paths:
        print("No images found in the directory.")
        return

    remove_hot_pixels(image_paths, output_directory, threshold=5)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-I", "--input_directory", type=str, required=True)
    parser.add_argument("-W", "--output_directory", type=str, required=True)
    args = parser.parse_args()
    
    # Replace 'path/to/your/directory' with the actual path to your image directory
    input_directory = args.input_directory # '/home/gokul/Data/sweep/dvs/frames_with_stationary/+0+2+0_h_ref'
    output_directory = args.output_directory # '/home/gokul/Data/sweep/dvs/frames_hot_pixels_removed/+0+2+0_h_ref'
    os.makedirs(output_directory, exist_ok=True)
    
    main(input_directory, output_directory)