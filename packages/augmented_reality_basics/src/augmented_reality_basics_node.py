#!/usr/bin/env python3
import numpy as np
import rospy
import cv2

from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType
from duckietown_msgs.msg import WheelEncoderStamped
from sensor_msgs.msg import CompressedImage

def draw_segment(self, image, pt_x, pt_y, color):
    defined_colors = {
        'red': ['rgb', [1, 0, 0]],
        'green': ['rgb', [0, 1, 0]],
        'blue': ['rgb', [0, 0, 1]],
        'yellow': ['rgb', [1, 1, 0]],
        'magenta': ['rgb', [1, 0 , 1]],
        'cyan': ['rgb', [0, 1, 1]],
        'white': ['rgb', [1, 1, 1]],
        'black': ['rgb', [0, 0, 0]]}
    _color_type, [r, g, b] = defined_colors[color]
    cv2.line(image, (pt_x[0], pt_y[0]), (pt_x[1], pt_y[1]), (b * 255, g * 255, r * 255), 5)
    return image


def readYamlFile(self,fname):
    """
    Reads the YAML file in the path specified by 'fname'.
    E.G. :
        the calibration file is located in : `/data/config/calibrations/filename/DUCKIEBOT_NAME.yaml`
    """
    with open(fname, 'r') as in_file:
        try:
            yaml_dict = yaml.load(in_file)
            return yaml_dict
        except yaml.YAMLError as exc:
            self.log("YAML syntax error. File: %s fname. Exc: %s"
                     %(fname, exc), type='fatal')
            rospy.signal_shutdown()
            return
        

class AugmentedRealityBascisNode(DTROS):

    def __init__(self, node_name):
        """
        AR basics
        """
        # Initialize the DTROS parent class
        super(AugmentedRealityBascisNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)


        self._map_file_name = "hud"

        # load camera intrinsics and extrinsics
        # ToDo

        # subscribe to camera stream
        self.sub_camera_img = rospy.Subscriber("camera_node/image/compressed", CompressedImage, self.cb_camera_img)

        # publish modified image
        self.pub_modified_img = rospy.Publisher(f"~/{self._map_file_name}/image/compressed")

        self.log("Letsgoooooooooooooooooo")

    def cb_camera_img(self, msg):
        # project map features onto image and publish it
        pass

    def process_image(self, img):
        pass

    def ground2pixel(self, points):
        pass

    def render_segments(self, segments):
        pass



if __name__ == '__main__':
    node = AugmentedRealityBascisNode(node_name='augmented_reality_basics_node')
    rospy.spin()
    rospy.loginfo("augmented_reality_basics_node is up and running...")
