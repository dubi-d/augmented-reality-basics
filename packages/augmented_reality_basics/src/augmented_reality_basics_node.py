#!/usr/bin/env python3
import numpy as np
import rospy
import cv2
from cv_bridge import CvBridge
import os
import yaml

from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType
from duckietown_msgs.msg import WheelEncoderStamped
from sensor_msgs.msg import CompressedImage


class AugmentedRealityBascisNode(DTROS):

    def __init__(self, node_name):
        """
        AR basics
        """
        # Initialize the DTROS parent class
        super(AugmentedRealityBascisNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)

        self.robot_name = rospy.get_namespace().strip('/')
        self._map_file_name = os.environ.get("DT_MAP_NAME", "hud")
        self._repo_path = os.environ.get("DT_REPO_PATH")

        # load camera intrinsics
        calibration_data = self.read_yaml_file(f"/data/config/calibrations/camera_intrinsic/{self.robot_name}.yaml")
        self.log(calibration_data)
        self._K, self._D, self._R, self._P, self._width, self._height, self._distortion_model = self.extract_camera_data(calibration_data)
        self._K_rect, self._roi = cv2.getOptimalNewCameraMatrix(self._K, self._D, (self._width, self._height), 1)
        self.log(f"roi: {self._roi}")
        self.log(f"K_rect: {self._K_rect}")
        self.log(f"K: {self._K}")
        self.log(f"D: {self._D}")
        self.log(f"R: {self._R}")
        self.log(f"P: {self._P}")
        self.log(f"width: {self._width}")
        self.log(f"height: {self._height}")

        # for cv2 and msg conversions
        self.bridge = CvBridge()

        # load camera extrinsic matrix
        extrinsics = self.read_yaml_file(f"/data/config/calibrations/camera_extrinsic/{self.robot_name}.yaml")
        self._extrinsic = np.array(extrinsics["homography"]).reshape(3, 3)
        self.log(self._extrinsic)

        # load map file
        self._map_data = self.read_yaml_file(f"{self._repo_path}/packages/augmented_reality_basics/maps/{self._map_file_name}.yaml")
        self.log(self._map_data)

        # subscribe to camera stream
        self.sub_camera_img = rospy.Subscriber("camera_node/image/compressed", CompressedImage, self.cb_camera_img, queue_size=1)

        # publish modified image
        self.pub_modified_img = rospy.Publisher(f"~{self._map_file_name}/image/compressed", CompressedImage, queue_size=1)

        self.log("Letsgoooooooooooooooooo")

    def cb_camera_img(self, msg):
        # project map features onto image and publish it
        img = self.bridge.compressed_imgmsg_to_cv2(msg)
        img_rectified = self.process_image(img)
        img_out = self.bridge.cv2_to_compressed_imgmsg(img_rectified)
        img_out.header = msg.header
        img_out.format = msg.format
        self.pub_modified_img.publish(img_out)

    def process_image(self, img):
        """
        Rectify image
        """
        undistorted_img = cv2.undistort(img, self._K, self._D, None, self._K_rect)
        # Optionally crop image to ROI
        #x, y, w, h = self._roi
        #undistorted_img = undistorted_img[y:y + h, x:x + w]
        return undistorted_img

    def ground2pixel(self, points):
        """
        Transform 3D points expressed in axle frame to points in the image. axle frame --extr-> camera frame --P-> image.

        - we only need to transform points from axle to image frame (hud is just a test, can use different function)
        - using a list of points is better that using the dict from yaml file, because of reusability for the next exercise
        """
        pass

    def render_segments(self, segments, img):
        pass

    def draw_segment(self, image, pt_x, pt_y, color):
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
        cv2.line(image, (pt_x[0], pt_y[0]), (pt_x[1], pt_y[1]), (b * 255, g * 255, r * 255), 5)
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
