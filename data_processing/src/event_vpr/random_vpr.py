import argparse
import sys
import random
from tqdm import tqdm
import numpy as np

from event_vpr.image_iterator import ImageIterator
from event_vpr.image_descriptors import compute_random_seed_descriptors
from event_vpr.descriptor_comparisons import sum_of_absolute_differences
from event_vpr.plot_results import plot_distance_matrix


def generate_random_pixel_locations(image_width, image_height, num_locations):
    pixel_locations = []

    for _ in range(num_locations):
        x = random.randint(0, image_width - 1)
        y = random.randint(0, image_height - 1)
        pixel_locations.append([x, y])

    return pixel_locations



def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-rs', '--reference_set', type=str, required=True, help='Path to directory containing reference event frames')
    parser.add_argument('-qs', '--query_set', type=str, required=True, help='Path to directory containing query event frames')
    args = parser.parse_args()

    reference_image_iterator = ImageIterator(args.reference_set)
    query_image_iterator = ImageIterator(args.query_set)

    reference_descriptors = []
    query_descriptors = []
    seed = generate_random_pixel_locations(260, 346, 10000)

    for reference_image in tqdm(reference_image_iterator, total=len(reference_image_iterator), desc="Processing Reference Images"):
        reference_descriptors.append(compute_random_seed_descriptors(reference_image, seed))

    for query_image in tqdm(query_image_iterator, total=len(query_image_iterator), desc="Processing Query Images"):
        query_descriptors.append(compute_random_seed_descriptors(query_image, seed))

    reference_descriptors = np.array(reference_descriptors)
    query_descriptors = np.array(query_descriptors)

    # size = np.min([reference_descriptors.shape[0], query_descriptors.shape[0]])
    distances = sum_of_absolute_differences(reference_descriptors, query_descriptors)

    plot_distance_matrix(distances)

    return 0


if __name__ == "__main__":
    sys.exit(main())  # pragma: no cover