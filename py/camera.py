import pyrealsense2 as rs
import numpy as np
import cv2


class Camera:
    def __init__(self):
        self.WIDTH = 640
        self.HEIGHT = 480
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, self.WIDTH, self.HEIGHT, rs.format.bgr8, 30)
        self.align = rs.align(rs.stream.color)
    
    
    def init(self):
        self.profile = self.pipeline.start(self.config)
        self.K, self.Kcoeffs = self.get_intrinsics_parameters()
    
    
    def close(self):
        self.pipeline.stop()
    
    
    def get_intrinsics_parameters(self):
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        color_frame = aligned_frames.get_color_frame()
        intrinsics = color_frame.profile.as_video_stream_profile().intrinsics
        K = np.array([
            [intrinsics.fx, 0, intrinsics.ppx],
            [0, intrinsics.fy, intrinsics.ppy],
            [0, 0, 1]
        ])
        return K, np.array(intrinsics.coeffs)
    
    
    def get_color_frame(self):
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        color_image = np.asanyarray(color_frame.get_data())
        return color_image

