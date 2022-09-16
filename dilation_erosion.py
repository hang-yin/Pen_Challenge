import pyrealsense2 as rs
# Import Numpy for easy array manipulation
import numpy as np
# Import OpenCV for easy image rendering
import cv2

def dilatation(images):
    dilatation_size = 12
    dilation_shape = cv2.MORPH_ELLIPSE
    element = cv2.getStructuringElement(dilation_shape, (2 * dilatation_size + 1, 2 * dilatation_size + 1),
                                       (dilatation_size, dilatation_size))
    dilatation_dst = cv2.dilate(images, element)
    return dilatation_dst

def erosion(images):
    erosion_size = 1
    erosion_shape = cv2.MORPH_ELLIPSE
    element = cv2.getStructuringElement(erosion_shape, (2 * erosion_size + 1, 2 * erosion_size + 1),
                                       (erosion_size, erosion_size))
    erosion_dst = cv2.erode(images, element)
    return erosion_dst