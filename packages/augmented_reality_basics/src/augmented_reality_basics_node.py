#!/usr/bin/env python3
import numpy as np
import rospy

from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType
from duckietown_msgs.msg import WheelEncoderStamped
from std_msgs.msg import Header, Float32
from std_srvs.srv import Trigger, TriggerResponse

class AugmentedRealityBascisNode(DTROS):

    def __init__(self, node_name):
        """
        AR basics
        """

        # Initialize the DTROS parent class
        super(AugmentedRealityBascisNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
        self.log("Letsgoooooooooooooooooo")


if __name__ == '__main__':
    node = AugmentedRealityBascisNode(node_name='augmented_reality_basics_node')
    rospy.spin()
    rospy.loginfo("augmented_reality_basics_node is up and running...")
