import numpy as np
import rospy
import cv2
import time

from image_geometry import PinholeCameraModel


def timeit(function):
  def new_function(*args, **kwargs):
    start_time = time.time()
    r = function(*args, **kwargs)
    elapsed = time.time() - start_time
    print(f'Function "{function.__name__}" took {elapsed} seconds to complete.')
    return r
  return new_function


class Augmenter:
    """
    AR utility class. Can be used to rectify camera images and project line segments on it.
    """
    def __init__(self, camera_info, homography, debug=False):
        self.pcm = PinholeCameraModel()
        self.pcm.fromCameraInfo(camera_info)
        self.H = homography  # maps points on ground plane to image plane
        self.debug = debug

        self.mapx, self.mapy = self._init_rectification()

    def _init_rectification(self):
        """
        Establish rectification mapping.
        """
        w = self.pcm.width
        h = self.pcm.height
        mapx = np.ndarray(shape=(h, w, 1), dtype='float32')
        mapy = np.ndarray(shape=(h, w, 1), dtype='float32')
        mapx, mapy = cv2.initUndistortRectifyMap(self.pcm.K, self.pcm.D, self.pcm.R,
                                                 self.pcm.P, (w, h),
                                                 cv2.CV_32FC1, mapx, mapy)
        return mapx, mapy

    def process_image(self, img_raw, interpolation=cv2.INTER_NEAREST):
        """
        Rectify image.
        """
        return cv2.remap(img_raw, self.mapx, self.mapy, interpolation)

    def ground2pixel(self, points_ground):
        """
        Project a 2D point from ground to image plane using a homography matrix.
        """
        self.log("\n\n ------ ground2pixel --------")
        self.log(f"ground point(s): {points_ground}")
        points_ground = np.array([[points_ground[0]], [points_ground[1]], [1]])  # [x, y]
        points_image = self.H.dot(points_ground)
        points_image = [int(points_image[1][0]/points_image[2][0]), int(points_image[0][0]/points_image[2][0])]  # [y, x]
        self.log(f"image point(s): {points_image}")
        return points_image

    def image01_to_pixel(self, point):
        """
        Scale point coordinates to image dimensions.
        """
        point[0] *= self.pcm.height
        point[1] *= self.pcm.width
        return point

    def render_segments(self, img, map_dict):
        """
        Render all the segments defined in 'map_dict' onto 'img'.
        """
        for seg in map_dict['segments']:
            pt_1_string = seg['points'][0]
            pt_1 = map_dict['points'][pt_1_string][1]
            pt_2_string = seg['points'][1]
            pt_2 = map_dict['points'][pt_2_string][1]
            self.draw_segment(img, pt_1, pt_2, seg['color'])
        return img

    @staticmethod
    def draw_segment(image, pt_1, pt_2, color):
        """
        Draw a line defined by two points into image. The points are assumed to contain [y, x] which
        corresponds to [row, column] of the image.
        """
        defined_colors = {
            'red': ['rgb', [1, 0, 0]],
            'green': ['rgb', [0, 1, 0]],
            'blue': ['rgb', [0, 0, 1]],
            'yellow': ['rgb', [1, 1, 0]],
            'magenta': ['rgb', [1, 0, 1]],
            'cyan': ['rgb', [0, 1, 1]],
            'white': ['rgb', [1, 1, 1]],
            'black': ['rgb', [0, 0, 0]]}
        _color_type, [r, g, b] = defined_colors[color]
        cv2.line(image, (pt_1[1], pt_1[0]), (pt_2[1], pt_2[0]), (b * 255, g * 255, r * 255), 5)
        return image

    def log(self, msg):
        if self.debug:
            rospy.loginfo(msg)

