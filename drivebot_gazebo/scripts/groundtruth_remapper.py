#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry

import sys


class GroundtruthRemapper:
    def __init__(self, name):
        rospy.init_node(name)
        if rospy.has_param("/groundtruth_remapper/subscribe_to"):
            self.sub_topic = rospy.get_param("/groundtruth_remapper/subscribe_to")
        else:
            rospy.logerr("Missing '/groundtruth_remapper/subscribe_to' parameter")
        if rospy.has_param("/groundtruth_remapper/publish_to"):
            self.pub_topic = rospy.get_param("/groundtruth_remapper/publish_to")
        else:
            rospy.logerr("Missing '/groundtruth_remapper/publish_to' parameter")
        self.groundtruth_sub = rospy.Subscriber(self.sub_topic, Odometry, self.onOdometry)
        self.groundtruth_pub = rospy.Publisher(self.pub_topic, Odometry, queue_size=10)

    def onOdometry(self, msg):
        messageOut = msg
        messageOut.header.frame_id = "odom"
        self.groundtruth_pub.publish(messageOut)

    def run(self):
        rospy.loginfo("[GroundtruthRemapper]: Spinning node!")
        rospy.spin()


if __name__ == '__main__':
    if len(sys.argv) < 2:
        rospy.logerr("Missing node name parameter")
    remapper = GroundtruthRemapper(sys.argv[1])
    remapper.run()
