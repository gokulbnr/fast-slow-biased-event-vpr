from event_vpr.descriptor_comparisons import sum_of_absolute_differences
from event_vpr.utilities import log_parameters, get_system_timestamp
import argparse
import numpy as np
import os
import logging

if __name__ == "__main__":

    # Parse arguments
    parser = argparse.ArgumentParser()
    parser.add_argument("-R", "--reference_descriptors", type=str, required=True)
    parser.add_argument("-Q", "--query_descriptors", type=str, required=True)
    parser.add_argument("-W", "--save_directory_path", type=str, required=True)
    parser.add_argument("-M", "--comparison_method", type=str, required=True, choices=['sad'])
    parser.add_argument("-L", "--log_directory_path", type=str)
    args = parser.parse_args()

    # Set up logging
    logging.basicConfig(filename=os.path.join(args.log_directory_path, 'similarity_experiments.log'), level=logging.INFO, format='%(message)s')

    # Load descriptors and ground truth
    reference_run = np.load(args.reference_descriptors)
    reference_descriptors = reference_run['descriptors']
    reference_odom = reference_run['odom']
    query_run = np.load(args.query_descriptors)
    query_descriptors = query_run['descriptors']
    query_odom = query_run['odom']
    
    # Generte similarity matrix
    if args.comparison_method == 'sad':
        descriptor_distances = sum_of_absolute_differences(reference_descriptors[:], query_descriptors[:])
    
    # Log the parameters
    log_parameters(os.path.basename(os.path.basename(args.reference_descriptors).split('.')[0]), '\nReference set')
    log_parameters(os.path.basename(os.path.basename(args.query_descriptors).split('.')[0]), 'Query set')
    log_parameters(get_system_timestamp(), 'Generation Timestamp')

    # Save similarity matrix
    filepath = os.path.join(args.save_directory_path, 'R_' + os.path.basename(args.reference_descriptors).split('.')[0] + '_Q_' + os.path.basename(args.query_descriptors).split('.')[0] + '_M_' + args.comparison_method)
    os.makedirs(os.path.dirname(filepath), exist_ok=True)
    np.savez(filepath, distances=descriptor_distances, reference_odom=reference_odom, query_odom=query_odom)