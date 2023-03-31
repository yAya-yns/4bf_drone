import numpy as np
import cv2 


class Vision():
    def __init__(self) -> None:
        self.depth_threshold = 2.0 

        # TODO: tune the color parameters
        self.red = np.array([0, 0, 200], dtype = "uint8")
        self.green = np.array([0, 0, 255], dtype = "uint8")

        self.color_threshold = 0.9
        self.count_threshold = 200

    def detect_close_to_cylinder(self, img : np.ndarray):
        assert type(img) == np.ndarray
        assert img.ndim == 3    # (rgb + depth) by row by column

        depth = img[3, :, :]

        return np.any(depth < self.depth_threshold)


    def get_turn_direction(self, img : np.ndarray):
        '''already assuming that the obstacle is within self.depth_threshold'''
        assert type(img) == np.ndarray
        assert img.ndim == 3    # (rgb + depth) by row by column

        # threshold based on the depth
        depth = (img[3, :, :] < self.depth_threshold).astype(np.int8)
        
        # draw contours
        contours, hierarchy = cv2.findContours(depth, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        print(contours)
        
        # crop image to only the region within contour, assuming the obstacle will be verticle in the image
        x, y, w, h = cv2.boundingRect(contours[0])
        img_cropped = img[:, y:y + h, x:x + w]
        img_rgb = img_cropped[0:3, :, :]

        img_rgb_normalized = img_rgb / np.linalg.norm(img_rgb, axis=0)

        dot_product_red = img_rgb_normalized * self.red.reshape(3, 1, 1) / np.linalg.norm(self.red)  # expected broadcast
        dot_product_green = img_rgb_normalized * self.green.reshape(3, 1, 1) / np.linalg.norm(self.green)  # expected broadcast

        dot_product_red = np.sum(dot_product_red, axis=0)
        dot_product_green = np.sum(dot_product_green, axis=0)

        # detect based on cosine similarity
        if np.sum(dot_product_red > self.color_threshold) > self.count_threshold:
            return 'red'
        elif np.sum(dot_product_green > self.color_threshold) > self.count_threshold:
            return 'green'
        else:
            return ''
        
# the unit tests
if __name__ == '__main__':
    path = r''

    vision = Vision()
    img = cv2.imread(path)
    vision.get_turn_direction(img)
    # TODO: use realsense camera to get some images for testing
