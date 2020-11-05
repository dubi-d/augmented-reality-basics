#!/usr/bin/env python3
import numpy as np
import rospy
import cv2
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

        # load camera intrinsics and extrinsics
        calibration_data = self.read_yaml_file(f"/data/config/calibrations/camera_intrinsic/{self.robot_name}.yaml")
        self.log(calibration_data)
        self._K, self._D, self._R, self._P = self.extract_camera_data(calibration_data)
        self.log(f"K: {self._K}")
        self.log(f"D: {self._D}")
        self.log(f"R: {self._R}")
        self.log(f"P: {self._P}")

        # load map file
        self._map_data = self.read_yaml_file(f"{self._repo_path}/packages/augmented_reality_basics/maps/{self._map_file_name}.yaml")
        self.log(self._map_data)

        # subscribe to camera stream
        self.sub_camera_img = rospy.Subscriber("camera_node/image/compressed", CompressedImage, self.cb_camera_img)

        # publish modified image
        self.pub_modified_img = rospy.Publisher(f"~/{self._map_file_name}/image/compressed", CompressedImage, queue_size=1)

        self.log("Letsgoooooooooooooooooo")

    def cb_camera_img(self, msg):
        # project map features onto image and publish it
        modified_img = CompressedImage()
        self.pub_modified_img.publish(modified_img)
        pass

    def process_image(self, img):
        pass

    def ground2pixel(self, points):
        pass

    def render_segments(self, segments, img):
        for seg in segments:

            self.draw_segment(img, )

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
        return data['camera_matrix']['data'], data['distortion_coefficients']['data'], \
               data['rectification_matrix']['data'], data['projection_matrix']['data']


if __name__ == '__main__':
    node = AugmentedRealityBascisNode(node_name='augmented_reality_basics_node')
    rospy.spin()
    rospy.loginfo("augmented_reality_basics_node is up and running...")
