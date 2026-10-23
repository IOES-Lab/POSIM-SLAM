"""Select the matched SDK OpenCV C ABI or the explicit research cv2 backend."""
import ctypes
import os
import sys
from pathlib import Path
import numpy as np


class NativeOpenCV:
    def __init__(self, path):
        self.library=ctypes.CDLL(str(path))
        lib=self.library
        lib.wwos_acoustic_abi.restype=ctypes.c_int
        lib.wwos_acoustic_opencv_version.restype=ctypes.c_char_p
        if lib.wwos_acoustic_abi()!=1:raise ValueError('unsupported_acoustic_abi')
        self.__version__=lib.wwos_acoustic_opencv_version().decode('ascii')
        byte=ctypes.POINTER(ctypes.c_uint8);number=ctypes.POINTER(ctypes.c_float)
        lib.wwos_acoustic_single_thread.argtypes=[]
        lib.wwos_acoustic_single_thread.restype=None
        lib.wwos_acoustic_blur.argtypes=[ctypes.c_int,ctypes.c_int,byte,byte]
        lib.wwos_acoustic_corners.argtypes=[ctypes.c_int,ctypes.c_int,byte,byte,number,ctypes.c_int]
        lib.wwos_acoustic_flow.argtypes=[ctypes.c_int,ctypes.c_int,byte,byte,number,ctypes.c_int,number,byte,number]
        self.setNumThreads(1)

    def setNumThreads(self, count):
        if count!=1:raise ValueError('acoustic_requires_single_thread')
        self.library.wwos_acoustic_single_thread()

    @staticmethod
    def image(value):
        a=np.asarray(value)
        if (a.dtype!=np.uint8 or a.ndim!=2 or min(a.shape)<1 or max(a.shape)>4096 or a.size>2_000_000):
            raise ValueError('invalid_acoustic_image')
        return np.ascontiguousarray(a)

    @staticmethod
    def pointer(a, kind):return a.ctypes.data_as(ctypes.POINTER(kind))

    def GaussianBlur(self, value, size, sigma, **kwargs):
        if size!=(5,5) or sigma!=0 or kwargs:raise ValueError('unsupported_acoustic_filter')
        a=self.image(value);output=np.empty_like(a)
        code=self.library.wwos_acoustic_blur(a.shape[1],a.shape[0],self.pointer(a,ctypes.c_uint8),self.pointer(output,ctypes.c_uint8))
        if code:raise RuntimeError('native_acoustic_blur_failed')
        return output

    def goodFeaturesToTrack(self, value, maximum, quality, separation, *, mask, blockSize):
        if (maximum,quality,separation,blockSize)!=(300,.04,5,7):raise ValueError('unsupported_acoustic_corners')
        a=self.image(value);m=self.image(mask)
        if m.shape!=a.shape:raise ValueError('invalid_acoustic_mask')
        output=np.empty((300,1,2),dtype=np.float32)
        count=self.library.wwos_acoustic_corners(a.shape[1],a.shape[0],self.pointer(a,ctypes.c_uint8),self.pointer(m,ctypes.c_uint8),self.pointer(output,ctypes.c_float),300)
        if not 0<=count<=300:raise RuntimeError('native_acoustic_corners_failed')
        return output[:count] if count else None

    def calcOpticalFlowPyrLK(self, previous, current, pixels, initial, *, winSize, maxLevel):
        if initial is not None or winSize!=(21,21) or maxLevel!=2:raise ValueError('unsupported_acoustic_flow')
        a=self.image(previous);b=self.image(current);points=np.asarray(pixels)
        if (a.shape!=b.shape or points.shape[1:]!=(1,2) or not 1<=len(points)<=300 or points.dtype!=np.float32):
            raise ValueError('invalid_acoustic_flow_input')
        points=np.ascontiguousarray(points);moved=np.empty_like(points)
        status=np.empty((len(points),1),dtype=np.uint8);error=np.empty((len(points),1),dtype=np.float32)
        code=self.library.wwos_acoustic_flow(a.shape[1],a.shape[0],self.pointer(a,ctypes.c_uint8),self.pointer(b,ctypes.c_uint8),self.pointer(points,ctypes.c_float),len(points),self.pointer(moved,ctypes.c_float),self.pointer(status,ctypes.c_uint8),self.pointer(error,ctypes.c_float))
        if code:raise RuntimeError('native_acoustic_flow_failed')
        return moved,status,error


native_library=os.environ.get('POSIM_SLAM_ACOUSTIC_LIBRARY') or os.environ.get('WWOS_ACOUSTIC_NATIVE_LIBRARY')
if native_library:
    from .native_provider import checked_library
    cv=NativeOpenCV(checked_library(native_library,os.environ.get('CONDA_PREFIX')))
else:
    import cv2 as cv
