from event_vpr.image_iterator import ImageIterator
from tqdm import tqdm
import numpy as np
import argparse
import os
from event_vpr.image_descriptors import compute_random_seed_pixels, patchnorm_feats, get_prob_proportionally_to_event_means, get_random_pixels
from event_vpr.utilities import ros_data_from_dirpath, get_system_timestamp, log_parameters
import cv2
import logging


if __name__ == "__main__":

    # Parse arguments
    parser = argparse.ArgumentParser()
    parser.add_argument("-S", "--seed_filepath", type=str)
    parser.add_argument("-I", "--image_directory_path", type=str, required=True)
    parser.add_argument("-W", "--save_directory_path", type=str, required=True)
    parser.add_argument("-M", "--descriptor_method", type=str, required=True, choices=['random','patchnorm','probabilistic'])
    parser.add_argument("-L", "--log_directory_path", type=str)
    args = parser.parse_args()

    # Setup directory structure
    filepath = os.path.join(args.save_directory_path, os.path.basename(args.image_directory_path))
    os.makedirs(os.path.dirname(filepath), exist_ok=True)

    # Set up logging
    logging.basicConfig(filename=os.path.join(args.log_directory_path, 'descriptors.log'), level=logging.INFO, format='%(message)s')

    # Load ground truth
    odom = ros_data_from_dirpath(args.image_directory_path)

    image_iterator = ImageIterator(args.image_directory_path)
    image_descriptors = []

    # Generate decriptors from images
    if args.descriptor_method == 'random':
        seed = np.load(args.seed_filepath)
        for image in tqdm(image_iterator, total=len(image_iterator), desc="Processing Images"):
            image_descriptors.append(compute_random_seed_pixels(image, seed))
    elif args.descriptor_method == 'patchnorm':
        for image in tqdm(image_iterator, total=len(image_iterator), desc="Processing Images"):
            image_descriptors.append(patchnorm_feats(cv2.resize(image, None, fx=0.25, fy=0.25, interpolation=cv2.INTER_AREA)))
    # elif args.descriptor_method == 'probabilistic':
    #     img_list = []
    #     temp_iterator = ImageIterator(args.image_directory_path)
    #     for image in tqdm(temp_iterator, total=len(temp_iterator), desc="Computing Means"):
    #         img_list.append(image)
    #     img_list = np.array(img_list)
    #     # print('img_list: ', img_list)
    #     event_means = [event_frame_total.mean(axis=2) for event_frame_total in img_list]
    #     prob_to_draw_from = get_prob_proportionally_to_event_means(np.array(event_means[0]))
    #     print(prob_to_draw_from.shape)
    #     seed = get_random_pixels(10000, 260, 346, 7, prob_to_draw_from)
    #     for image in tqdm(image_iterator, total=len(image_iterator), desc="Processing Images"):
    #         image_descriptors.append(compute_random_seed_pixels(image, seed))
    image_descriptors = np.array(image_descriptors)

    # Log the parameters
    log_parameters(os.path.basename(args.image_directory_path), '\nImage set')
    log_parameters(get_system_timestamp(), 'Generation Timestamp')
    log_parameters(image_descriptors.shape[0], 'Number of images')
    log_parameters(image_descriptors.shape[1], 'Descriptor size per image')
    
    # Save descriptors
    np.savez(filepath, descriptors=image_descriptors[200:], odom=odom[200:])
