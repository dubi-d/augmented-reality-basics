#!/usr/bin/env python3
import numpy as np
import rospy
import cv2
from cv_bridge import CvBridge
import os
import yaml
import time

from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType
from duckietown_msgs.msg import WheelEncoderStamped
from sensor_msgs.msg import CompressedImage


def timeit(function):
  def new_function(*args, **kwargs):
    start_time = time.time()
    r = function(*args, **kwargs)
    elapsed = time.time() - start_time
    print(f'Function "{function.__name__}" took {elapsed} seconds to complete.')
    return r
  return new_function


class AugmentedRealityBascisNode(DTROS):

    def __init__(self, node_name):
        """
        AR basics
        """
        # Initialize the DTROS parent class
        super(AugmentedRealityBascisNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)

        self.robot_name = rospy.get_namespace().strip('/')
        self._map_file_name = os.environ.get("DT_MAP_NAME", "calibration_pattern")
        self._repo_path = os.environ.get("DT_REPO_PATH")

        # load camera intrinsics
        calibration_data = self.read_yaml_file(f"/data/config/calibrations/camera_intrinsic/{self.robot_name}.yaml")
        self.log(calibration_data)
        self._K, self._D, self._R, self._P, self._width, self._height, self._distortion_model = self.extract_camera_data(calibration_data)
        self._K_rect, self._roi = cv2.getOptimalNewCameraMatrix(self._K, self._D, (self._width, self._height), 1)
        self.log(f"\nroi: {self._roi}")
        self.log(f"\nK_rect: {self._K_rect}")
        self.log(f"\nK: {self._K}")
        self.log(f"\nD: {self._D}")
        self.log(f"\nR: {self._R}")
        self.log(f"\nP: {self._P}")
        self.log(f"\nwidth: {self._width}")
        self.log(f"height: {self._height}")

        # for cv2 and msg conversions
        self.bridge = CvBridge()

        # load camera extrinsic matrix
        extrinsics = self.read_yaml_file(f"/data/config/calibrations/camera_extrinsic/{self.robot_name}.yaml")
        self._extrinsic = np.array(extrinsics["homography"])
        self.log(f"\nExtrinsic: {self._extrinsic}")

        # load map file
        self.map_dict = self.read_yaml_file(f"{self._repo_path}/packages/augmented_reality_basics/maps/{self._map_file_name}.yaml")
        self.log(self.map_dict)

        # subscribe to camera stream
        self.sub_camera_img = rospy.Subscriber("camera_node/image/compressed", CompressedImage, self.cb_camera_img, queue_size=1)

        # publish modified image
        self.pub_modified_img = rospy.Publisher(f"~{self._map_file_name}/image/compressed", CompressedImage, queue_size=1)

        # map point coordinates in map_dict to image plane
        #self.image0_to_pixel_test(self.map_dict["points"])
        self.ground2pixel(self.map_dict["points"])
        self.log(f"\nMap dict modified: {self.map_dict}")

        self.log("Letsgoooooooooooooooooo")

    @timeit
    def cb_camera_img(self, msg):
        """
        project map features onto image and publish it
        """

        # convert to cv2 img
        img = self.bridge.compressed_imgmsg_to_cv2(msg)

        # rectify img
        img_rectified = self.process_image(img)

        # draw stuff on img
        img_modified = self.render_segments(img_rectified)

        # publish modified img
        img_out = self.bridge.cv2_to_compressed_imgmsg(img_modified)
        img_out.header = msg.header
        img_out.format = msg.format
        self.pub_modified_img.publish(img_out)

    @timeit
    def process_image(self, img):
        """
        Rectify image
        """
        undistorted_img = cv2.undistort(img, self._K, self._D, None, self._K_rect)
        # Optionally crop image to ROI
        #x, y, w, h = self._roi
        #undistorted_img = undistorted_img[y:y + h, x:x + w]
        return undistorted_img

    def ground2pixel(self, points_dict):
        """
        Transform 3D points expressed in axle frame to points in the image. axle frame --extr-> camera frame --P-> image.

        - we only need to transform points from axle to image frame (hud is just a test, can use different function)
        - using a list of points is better that using the dict from yaml file, because of reusability for the next exercise

        """
        for item in points_dict.values():
            self.log(np.array([[item[1][0], item[1][1]]]))
            transformed_point = cv2.perspectiveTransform(np.array([[item[1][0], item[1][1]]]).transpose(), self._extrinsic)
            self.log(transformed_point)
            item[1] = [transformed_point[0], transformed_point[1]]
            self.log(item)
        self.log(points_dict)


    def image0_to_pixel_test(self, points_dict):
        for item in points_dict.values():
            item[1][0] *= self._height
            item[1][1] *= self._width

    @timeit
    def render_segments(self, img):
        for seg in self.map_dict['segments']:
            pt_1_string = seg['points'][0]
            pt_1 = self.map_dict['points'][pt_1_string][1]
            pt_2_string = seg['points'][1]
            pt_2 = self.map_dict['points'][pt_2_string][1]

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
    def extract_camera_data(data):
        k = np.array(data['camera_matrix']['data']).reshape(3, 3)
        d = np.array(data['distortion_coefficients']['data'])
        r = np.array(data['rectification_matrix']['data']).reshape(3, 3)
        p = np.array(data['projection_matrix']['data']).reshape(3, 4)
        width = data['image_width']
        height = data['image_height']
        distortion_model = data['distortion_model']
        return k, d, r, p, width, height, distortion_model

if __name__ == '__main__':
    node = AugmentedRealityBascisNode(node_name='augmented_reality_basics_node')
    rospy.spin()
    rospy.loginfo("augmented_reality_basics_node is up and running...")
