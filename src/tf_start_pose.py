#!/usr/bin/env python3

import rospy
import geometry_msgs.msg
import tf2_ros
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry


class FixedStartPoseBroadcaster:

    def __init__(self):
        local_position_msg = rospy.wait_for_message('/uav0/mavros/local_position/odom', Odometry)
        start_pose: Pose = local_position_msg.pose.pose
        rospy.loginfo(f"Local position received, pose: \n{start_pose}")

        self.broadcaster = tf2_ros.StaticTransformBroadcaster()
        t = geometry_msgs.msg.TransformStamped()
        t.header.frame_id = "map"
        t.header.stamp = rospy.Time.now()
        t.child_frame_id = "start_pose"
        t.transform.translation.x = start_pose.position.x
        t.transform.translation.y = start_pose.position.y
        t.transform.translation.z = start_pose.position.z

        t.transform.rotation.x = start_pose.orientation.x
        t.transform.rotation.y = start_pose.orientation.y
        t.transform.rotation.z = start_pose.orientation.z
        t.transform.rotation.w = start_pose.orientation.w
        self.broadcaster.sendTransform(t)
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('fixed_start_pose_tf_broadcaster')
    tfb = FixedStartPoseBroadcaster()
    rospy.spin()
