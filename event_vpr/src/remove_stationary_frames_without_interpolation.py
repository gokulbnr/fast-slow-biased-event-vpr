import os
import shutil
import argparse

def parse_filename(filename):
    # Function to parse the filename and extract timestamp, location_x, and location_y
    parts = filename[:-4].split('_')  # Remove '.png' and split the remaining string
    timestamp = float(parts[0])
    location_x = float(parts[1])
    location_y = float(parts[2])
    return timestamp, location_x, location_y

def copy_images(source_dir, destination_dir):
    # Function to copy images with a net displacement greater than 0 between consecutive images
    if not os.path.exists(destination_dir):
        os.makedirs(destination_dir)

    images = sorted(os.listdir(source_dir))

    for i in range(len(images) - 1):
        current_image = images[i]
        next_image = images[i + 1]

        timestamp1, curr_x, curr_y = parse_filename(current_image)
        timestamp2, next_x, next_y = parse_filename(next_image)

        displacement_x = next_x - curr_x
        displacement_y = next_y - curr_y
    
        if displacement_x == 0 and displacement_y == 0:
            continue

        source_path = os.path.join(source_dir, current_image)
        destination_path = os.path.join(destination_dir, current_image)
        shutil.copy(source_path, destination_path)

if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("-I", "--input_dir_path", type=str, required=True)
    parser.add_argument("-W", "--output_dir_path", type=str, required=True)
    args = parser.parse_args()

    source_directory = args.input_dir_path
    destination_directory = args.output_dir_path

    copy_images(source_directory, destination_directory)