import cv2
import os

"""Image Iterator Class to read sequences of images given a directory path

Raises:
    StopIteration: _description_

Returns:
    _type_: _description_
"""

class ImageIterator:
    def __init__(self, image_dir, image_extensions=(".jpg", ".jpeg", ".png")):
        self.image_dir = image_dir
        self.image_files = sorted([os.path.join(image_dir, filename) for filename in os.listdir(image_dir) if filename.endswith(image_extensions)])
        self.index = 0

    def __iter__(self):
        return self

    def __next__(self):
        if self.index < len(self.image_files):
            image_file = self.image_files[self.index]
            image = cv2.imread(image_file)
            image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

            # if image is not None:
                # You can perform any processing or analysis on the image here
                # For example, you can resize the image or apply some filters
                # For demonstration, we'll just print the image shape
                # print(f"Processing {image_file}: shape = {image.shape}")
 
            self.index += 1
            return image
        else:                
            raise StopIteration

    def __len__(self):
        return len(self.image_files)
