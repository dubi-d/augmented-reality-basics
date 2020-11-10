#!/usr/bin/env python3
import numpy as np
import rospy
from cv_bridge import CvBridge
import os
import yaml

from sensor_msgs.msg import CompressedImage, CameraInfo
from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType

from augmenter import Augmenter


class AugmentedRealityBasicsNode(DTROS):
    """
    AR basics. Project lines with predefined end points into the camera image.
    """
    def __init__(self, node_name):
        # Initialize the DTROS parent class
        super(AugmentedRealityBasicsNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)

        self.robot_name = rospy.get_namespace().strip('/')
        self._map_file_name = os.environ.get("DT_MAP_NAME", "hud")
        self._repo_path = os.environ.get("DT_REPO_PATH")

        # load camera intrinsics
        calibration_data = self.read_yaml_file(f"/data/config/calibrations/camera_intrinsic/{self.robot_name}.yaml")
        camera_info = self.camera_info_from_yaml(calibration_data)

        # load camera extrinsics
        extrinsics = self.read_yaml_file(f"/data/config/calibrations/camera_extrinsic/{self.robot_name}.yaml")
        homography = np.array(extrinsics["homography"]).reshape(3, 3)
        homography = np.linalg.inv(homography)  # map ground to image plane

        # initialize augmenter utility class
        self.augmenter = Augmenter(camera_info, homography, debug=False)

        # for cv2 and msg conversions
        self.bridge = CvBridge()

        # load map file
        self.map_dict = self.read_yaml_file(f"{self._repo_path}/packages/augmented_reality_basics/maps/{self._map_file_name}.yaml")

        # subscribe to camera stream
        self.sub_camera_img = rospy.Subscriber("camera_node/image/compressed", CompressedImage, self.callback, queue_size=1)

        # publish modified image
        self.pub_modified_img = rospy.Publisher(f"~{self._map_file_name}/image/compressed", CompressedImage, queue_size=1)

        # map point coordinates in map_dict to image plane
        self.project_map_points(self.map_dict["points"])

        self.log("Letsgoooooooooooooooooo")

    def callback(self, msg):
        """
        On camera image callback, rectify image, project line segments onto it and publish it.
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
        """
        Express calibration data (intrinsics) as a CameraInfo instance.

        :param calib_data: dict, loaded from yaml file
        :return: intrinsics as CameraInfo instance
        """
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
        """
        Project points in 'points_dict' into image frame. Modifies original dict.
        """
        for item in points_dict.values():
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


if __name__ == '__main__':
    node = AugmentedRealityBasicsNode(node_name='augmented_reality_basics_node')
    rospy.spin()
    rospy.loginfo("augmented_reality_basics_node is up and running...")
