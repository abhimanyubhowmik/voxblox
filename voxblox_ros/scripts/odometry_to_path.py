#!/usr/bin/env python3
"""
odometry_to_path.py
Subscribes to a nav_msgs/Odometry topic and publishes the accumulated
trajectory as nav_msgs/Path.

Parameters (ROS, all optional):
  ~odometry_topic   (str,   default: /rovio/odometry)  input odometry
  ~path_topic       (str,   default: ~path)             output path
  ~max_poses        (int,   default: 10000)             max stored poses
                                                         (0 = unlimited)
  ~min_distance_m   (float, default: 0.02)              only append pose if
                                                         robot moved >= this
"""

import rospy
import math
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped


def dist(p1, p2):
    dx = p1.x - p2.x
    dy = p1.y - p2.y
    dz = p1.z - p2.z
    return math.sqrt(dx * dx + dy * dy + dz * dz)


class OdometryToPath:
    def __init__(self):
        rospy.init_node("odometry_to_path", anonymous=False)

        odometry_topic = rospy.get_param("~odometry_topic", "/rovio/odometry")
        path_topic     = rospy.get_param("~path_topic",     "~path")
        self.max_poses    = rospy.get_param("~max_poses",      10000)
        self.min_dist     = rospy.get_param("~min_distance_m", 0.02)

        self.path = Path()
        self.last_pos = None

        self.pub = rospy.Publisher(path_topic, Path, queue_size=1, latch=True)
        rospy.Subscriber(odometry_topic, Odometry, self.odom_cb, queue_size=10)

        rospy.loginfo("[odometry_to_path] Subscribing to: %s", odometry_topic)
        rospy.loginfo("[odometry_to_path] Publishing path on: %s", path_topic)
        rospy.loginfo("[odometry_to_path] min_distance=%.3f m  max_poses=%d",
                      self.min_dist, self.max_poses)
        rospy.spin()

    def odom_cb(self, msg):
        pos = msg.pose.pose.position

        # Skip if robot hasn't moved enough
        if self.last_pos is not None and dist(pos, self.last_pos) < self.min_dist:
            return

        self.last_pos = pos

        # Build PoseStamped from odometry
        ps = PoseStamped()
        ps.header = msg.header
        ps.pose   = msg.pose.pose

        # Cap history length
        if self.max_poses > 0 and len(self.path.poses) >= self.max_poses:
            self.path.poses.pop(0)

        self.path.poses.append(ps)
        self.path.header = msg.header   # keep frame_id and latest stamp

        self.pub.publish(self.path)


if __name__ == "__main__":
    try:
        OdometryToPath()
    except rospy.ROSInterruptException:
        pass
