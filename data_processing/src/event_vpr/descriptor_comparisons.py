
import numpy as np
from tqdm import tqdm

"""_summary_of_absolute_differences(descriptors_1, descriptors_2)
"""
def sum_of_absolute_differences(descriptors_1, descriptors_2):
    """Compute the sum of absolute differences between two sets of descriptors

    Args:
        descriptors_1 (np array): 2D array of descriptors
        descriptors_2 (np array): 2D array of descriptors

    Returns:
        float: Sum of absolute differences between the two sets of descriptors
    """
    distances = np.zeros((descriptors_1.shape[0], descriptors_2.shape[0]))
    for i in tqdm(range(descriptors_1.shape[0]), desc="Computing distances"):
        for j in range(descriptors_2.shape[0]):
            distances[i, j] = np.sum(np.average(np.abs(descriptors_1[i] - descriptors_2[j])))

    # Initialize the distances array with the same shape as the output
    # distances = np.empty((descriptors_1.shape[0], descriptors_2.shape[0]))

    # # descriptors_1[descriptors_1 == 0] = -10000
    # # descriptors_2[descriptors_2 == 0] = +10000
    
    # # Compute the distances
    # for i in tqdm(range(descriptors_1.shape[0]), desc="Computing distances"):
    #     diff = np.abs(descriptors_2 - descriptors_1[i])
    #     distances[i,:] = np.sum(diff, axis=1)


    return distances


def brute_force_hamming(descriptor_1, descriptor_2):
    """Compute the hamming distance between two descriptors

    Args:
        descriptor_1 (np array): 1D array of descriptors
        descriptor_2 (np array): 1D array of descriptors

    Returns:
        float: Hamming distance between the two descriptors
    """
    return np.sum(np.abs(descriptor_1 - descriptor_2))