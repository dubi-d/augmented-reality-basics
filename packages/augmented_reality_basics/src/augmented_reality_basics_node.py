#!/usr/bin/env python3
import numpy as np
import rospy
import cv2
from cv_bridge import CvBridge
import os
import yaml
import time

from image_geometry import PinholeCameraModel
from sensor_msgs.msg import CompressedImage, CameraInfo

from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType
from duckietown_msgs.msg import WheelEncoderStamped


def timeit(function):
  def new_function(*args, **kwargs):
    start_time = time.time()
    r = function(*args, **kwargs)
    elapsed = time.time() - start_time
    print(f'Function "{function.__name__}" took {elapsed} seconds to complete.')
    return r
  return new_function


class AugmentedRealityBasicsNode(DTROS):
    def __init__(self, node_name):
        """
        AR basics
        """
        # Initialize the DTROS parent class
        super(AugmentedRealityBasicsNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)

        self.robot_name = rospy.get_namespace().strip('/')
        self._map_file_name = os.environ.get("DT_MAP_NAME", "hud")  # calibration_pattern
        self._repo_path = os.environ.get("DT_REPO_PATH")

        # load camera intrinsics
        calibration_data = self.read_yaml_file(f"/data/config/calibrations/camera_intrinsic/{self.robot_name}.yaml")
        camera_info = self.camera_info_from_yaml(calibration_data)
        self.log(camera_info)

        # load camera extrinsics
        extrinsics = self.read_yaml_file(f"/data/config/calibrations/camera_extrinsic/{self.robot_name}.yaml")
        homography = np.array(extrinsics["homography"]).reshape(3, 3)
        homography = np.linalg.inv(homography)  # map ground to image plane

        # initialize augmenter utility class
        self.augmenter = Augmenter(camera_info, homography, debug=True)

        # for cv2 and msg conversions
        self.bridge = CvBridge()

        # load map file
        self.map_dict = self.read_yaml_file(f"{self._repo_path}/packages/augmented_reality_basics/maps/{self._map_file_name}.yaml")
        self.log(self.map_dict)

        # subscribe to camera stream
        self.sub_camera_img = rospy.Subscriber("camera_node/image/compressed", CompressedImage, self.cb_camera_img, queue_size=1)

        # publish modified image
        self.pub_modified_img = rospy.Publisher(f"~{self._map_file_name}/image/compressed", CompressedImage, queue_size=1)

        # map point coordinates in map_dict to image plane
        self.project_map_points(self.map_dict["points"])
        self.log(f"\nMap dict modified: {self.map_dict}")

        self.log("Letsgoooooooooooooooooo")

    def cb_camera_img(self, msg):
        """
        project map features onto image and publish it
        """
        img = self.bridge.compressed_imgmsg_to_cv2(msg)  # convert to cv2 img

        img_rectified = self.augmenter.process_image(img) # rectify img

        img_modified = self.augmenter.render_segments(img_rectified, self.map_dict)  # draw stuff on img

        # publish modified img
        img_out = self.bridge.cv2_to_compressed_imgmsg(img_modified)
        img_out.header = msg.header
        img_out.format = msg.format
        self.pub_modified_img.publish(img_out)

    def read_yaml_file(self, filename):
        """
        Reads the YAML file in the path specified by 'fname'.
        E.G. :
            the calibration file is located in : `/data/config/calibrations/filename/DUCKIEBOT_NAME.yaml`
        """
        with open(filename, 'r') as in_file:
            try:
                yaml_dict = yaml.load(in_file)
                return yaml_dict
            except yaml.YAMLError as exc:
                self.log("YAML syntax error. File: %s fname. Exc: %s"
                         % (filename, exc), type='fatal')
                rospy.signal_shutdown()
                return

    @staticmethod
    def camera_info_from_yaml(calib_data):
        cam_info = CameraInfo()
        cam_info.width = calib_data['image_width']
        cam_info.height = calib_data['image_height']
        cam_info.K = calib_data['camera_matrix']['data']
        cam_info.D = calib_data['distortion_coefficients']['data']
        cam_info.R = calib_data['rectification_matrix']['data']
        cam_info.P = calib_data['projection_matrix']['data']
        cam_info.distortion_model = calib_data['distortion_model']
        return cam_info

    def project_map_points(self, points_dict):
        for item in points_dict.values():
            self.log(item)
            point = [item[1][0], item[1][1]]
            frame = item[0]
            if frame == "image01":
                transformed_point = self.augmenter.image01_to_pixel(point)
            elif frame == "axle":
                transformed_point = self.augmenter.ground2pixel(point)
            elif frame == "image":
                transformed_point = point
            else:
                raise Exception("[AugmentedRealityBasicsNode.project_map_points] Invalid frame")
            item[0] = "image"
            item[1] = transformed_point


class Augmenter:

    def __init__(self, camera_info, homography, debug=False):
        """
        AR basics
        """
        self.pcm = PinholeCameraModel()
        self.pcm.fromCameraInfo(camera_info)
        self.H = homography  # maps points on ground plane to image plane
        self.debug = debug

        self.mapx, self.mapy = self._init_rectification()

    def _init_rectification(self):
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
        Rectify image
        """
        #cv_image_rectified = np.empty_like(img_raw)
        return cv2.remap(img_raw, self.mapx, self.mapy, interpolation)

    def ground2pixel(self, points_ground):
        """
        Transform 3D points expressed in axle frame to points in the image. axle frame --extr-> camera frame --P-> image.

        - we only need to transform points from axle to image frame (hud is just a test, can use different function)
        - using a list of points is better that using the dict from yaml file, because of reusability for the next exercise

        """
        self.log("\n\n ------ ground2pixel --------")
        self.log(f"ground point(s): {points_ground}")
        points_ground = np.array([[points_ground[0]], [points_ground[1]], [1]])  # [x, y]
        points_image = self.H.dot(points_ground)
        points_image = [int(points_image[1][0]/points_image[2][0]), int(points_image[0][0]/points_image[2][0])]  # [y, x]
        self.log(f"image point(s): {points_image}")
        return points_image

    def image01_to_pixel(self, point):
        point[0] *= self.pcm.height
        point[1] *= self.pcm.width
        return point

    def render_segments(self, img, map_dict):
        for seg in map_dict['segments']:
            pt_1_string = seg['points'][0]
            pt_1 = map_dict['points'][pt_1_string][1]
            pt_2_string = seg['points'][1]
            pt_2 = map_dict['points'][pt_2_string][1]
            self.draw_segment(img, pt_1, pt_2, seg['color'])
        return img

    @staticmethod
    def draw_segment(image, pt_1, pt_2, color):
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

if __name__ == '__main__':
    node = AugmentedRealityBasicsNode(node_name='augmented_reality_basics_node')
    rospy.spin()
    rospy.loginfo("augmented_reality_basics_node is up and running...")


#self.robot_name = rospy.get_namespace().strip('/')
        #self._map_file_name = os.environ.get("DT_MAP_NAME", "calibration_pattern")  #
        #self._repo_path = os.environ.get("DT_REPO_PATH")
        #
        ## load camera intrinsics
        #calibration_data = self.read_yaml_file(f"/data/config/calibrations/camera_intrinsic/{self.robot_name}.yaml")
        #self.log(calibration_data)
        #self._K, self._D, self._R, self._P, self._width, self._height, self._distortion_model = self.extract_camera_data(calibration_data)
        #self._K_rect, self._roi = cv2.getOptimalNewCameraMatrix(self._K, self._D, (self._width, self._height), 1)
        #self.log(f"\nroi: {self._roi}")
        #self.log(f"\nK_rect: {self._K_rect}")
        #self.log(f"\nK: {self._K}")
        #self.log(f"\nD: {self._D}")
        #self.log(f"\nR: {self._R}")
        #self.log(f"\nP: {self._P}")
        #self.log(f"\nwidth: {self._width}")
        #self.log(f"height: {self._height}")
        #
        ## for cv2 and msg conversions
        #self.bridge = CvBridge()
        #
        ## load camera extrinsic matrix
        #extrinsics = self.read_yaml_file(f"/data/config/calibrations/camera_extrinsic/{self.robot_name}.yaml")
        #self._extrinsic = np.array(extrinsics["homography"]).reshape(3, 3)
        #self._extrinsic = np.linalg.inv(self._extrinsic)
        #self._extrinsic = self._K_rect.dot(np.linalg.inv(self._K)).dot(self._extrinsic)
        #self.log(f"\nExtrinsic: {self._extrinsic}")
        #
        ## load map file
        #self.map_dict = self.read_yaml_file(f"{self._repo_path}/packages/augmented_reality_basics/maps/{self._map_file_name}.yaml")
        #self.log(self.map_dict)
        #
        ## subscribe to camera stream
        #self.sub_camera_img = rospy.Subscriber("camera_node/image/compressed", CompressedImage, self.cb_camera_img, queue_size=1)
        #
        ## publish modified image
        #self.pub_modified_img = rospy.Publisher(f"~{self._map_file_name}/image/compressed", CompressedImage, queue_size=1)
        #
        ## map point coordinates in map_dict to image plane
        ##self.image0_to_pixel_test(self.map_dict["points"])
        #self.ground2pixel(self.map_dict["points"])
        #self.log(f"\nMap dict modified: {self.map_dict}")
        #
        #self.log("Letsgoooooooooooooooooo")
        #


# self.log(point)
# self.log(f"shape: {point.shape}")
# transformed_point = cv2.perspectiveTransform(point , self._extrinsic)
# self.log(f"transformed: {transformed_point}")
# transformed_point_2 = np.linalg.inv(self._extrinsic) * np.array([[item[1][0], item[1][1], 1]], dtype=np.float32).transpose()
# transformed_point_2 = self._extrinsic.dot(point)
# self.log(f"matrix mult: {transformed_point_2}")
# item[1] = [int(transformed_point_2[1][0] / transformed_point_2[2][0]), int(transformed_point_2[0][0] / transformed_point_2[2][0])]
##item[1] = [int(transformed_point[0][0][0]), int(transformed_point[0][0][1])]
# self.log(f"Transformed point 2: {transformed_point_2[1][0] / transformed_point_2[2][0]}, {transformed_point_2[0][0] / transformed_point_2[2][0]}")
# self.log(f"item: {item}")
#