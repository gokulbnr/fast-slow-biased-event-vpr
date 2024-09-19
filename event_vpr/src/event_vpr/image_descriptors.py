import cv2
import numpy as np
import copy
from tqdm import tqdm
# import matplotlib.pyplot as plt


def get_prob_proportionally_to_event_means(event_means_ref, hot_pixel_workaround=True):
    prob_to_draw_from = np.copy(event_means_ref)
    # prob_to_draw_from[np.logical_and(prob_to_draw_from < 0.5, prob_to_draw_from != 0.0)] = 0.01

    # Set low prob for "hot pixels"
    if hot_pixel_workaround:
        prob_to_draw_from[prob_to_draw_from > (event_means_ref.mean() + 2 * event_means_ref.std())] = 0.01
    prob_sum = prob_to_draw_from.sum()
    prob_to_draw_from = prob_to_draw_from / prob_sum
    return prob_to_draw_from

def get_random_pixels(
    num_pixels,
    im_width,
    im_height,
    local_suppression_radius,
    prob_to_draw_from=None,
):
    random_pixels = []
    num_subsequent_rejections = 0
    with tqdm(total=num_pixels, desc="Pick random pixels") as pbar:
        while len(random_pixels) < num_pixels:
            random_idx_flat = np.random.choice(
                np.arange(0, im_height * im_width),
                p=prob_to_draw_from.reshape(-1) if prob_to_draw_from is not None else None,
            )
            random_pixel = np.unravel_index(random_idx_flat, (im_height, im_width))
            if len(random_pixels) == 0 or np.all(np.linalg.norm(np.array(random_pixels) - np.array(random_pixel), axis=1) > local_suppression_radius):
                random_pixels.append(random_pixel)
                num_subsequent_rejections = 0
                pbar.update(1)
            else:
                num_subsequent_rejections = num_subsequent_rejections + 1
                if num_subsequent_rejections > 100:
                    raise ValueError("Could not find new random pixel after 100 iterations")

    # check that number of unique elements equals the number of requested pixels
    assert len(list(set(random_pixels))) == num_pixels

    return random_pixels

def compute_random_seed_pixels(image, seed):
    descriptor = []
    for pixel in seed:
        descriptor.append(image[pixel[0], pixel[1]])
    return descriptor

def get_sift(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    sift = cv2.SIFT_create()
    kp, des = sift.detectAndCompute(gray, None)
    return des

def patchnorm_feats(image):
    ft, _ = patchNormalizeImage(image, 8)
    return ft.flatten()

def patchNormalizeImage(img1,patchLength):
    numZeroStd = []
    img1 = img1.astype(float)
    img2 = copy.deepcopy(img1)
    imgMask = np.ones(img1.shape,dtype=bool)
    
    if patchLength == 1:
        return img2

    for i in range(img1.shape[0]//patchLength):
        iStart = i*patchLength
        iEnd = (i+1)*patchLength
        for j in range(img1.shape[1]//patchLength):
            jStart = j*patchLength
            jEnd = (j+1)*patchLength
            tempData = img1[iStart:iEnd, jStart:jEnd].copy()
            mean1 = np.mean(tempData)
            std1 = np.std(tempData)
            tempData = (tempData - mean1)
            if std1 == 0:
                std1 = 0.1 
                numZeroStd.append(1)
                imgMask[iStart:iEnd,jStart:jEnd] = np.zeros([patchLength,patchLength],dtype=bool)
            tempData /= std1
            img2[iStart:iEnd, jStart:jEnd] = tempData.copy()
    # plt.imshow(img1)
    # plt.show()
    # plt.imshow(img2)
    # plt.show()
    return img2, imgMask