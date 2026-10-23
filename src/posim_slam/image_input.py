"""Strict 8-bit camera layout validation independent of ROS and Python cv2."""
import numpy as np


def gray8(width,height,step,encoding,data):
    channels={'mono8':1,'rgb8':3,'bgr8':3}.get(encoding)
    if (type(width) is not int or type(height) is not int or type(step) is not int
        or not 32<=width<=1920 or not 32<=height<=1080 or not channels
        or step<width*channels or len(data)!=step*height):
        raise ValueError('image_shape_encoding_or_stride')
    image=np.ndarray((height,width,channels),dtype=np.uint8,buffer=bytes(data),
                     strides=(step,channels,1))
    if channels==1:return image[:,:,0].copy()
    rgb=(image if encoding=='rgb8' else image[:,:,::-1]).astype(np.uint16)
    return ((77*rgb[:,:,0]+150*rgb[:,:,1]+29*rgb[:,:,2]+128)>>8).astype(np.uint8)

