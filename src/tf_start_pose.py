#!/usr/bin/env python3

import rospy
import tf2_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry


class FixedStartPoseBroadcaster:

    def __init__(self):
        self.pub_tf = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=1)
        self.start_pose: Pose = None
        self.local_position_subscriber = rospy.Subscriber('/uav0/mavros/local_position/odom', Odometry, self.local_position_callback)

        while not rospy.is_shutdown():
            rospy.sleep(0.1)
            if self.start_pose is not None:
                t = geometry_msgs.msg.TransformStamped()
                t.header.frame_id = "map"
                t.header.stamp = rospy.Time.now()
                t.child_frame_id = "start_pose"
                t.transform.translation.x = self.start_pose.position.x
                t.transform.translation.y = self.start_pose.position.y
                t.transform.translation.z = self.start_pose.position.z

                t.transform.rotation.x = self.start_pose.orientation.x
                t.transform.rotation.y = self.start_pose.orientation.y
                t.transform.rotation.z = self.start_pose.orientation.z
                t.transform.rotation.w = self.start_pose.orientation.w

                tfm = tf2_msgs.msg.TFMessage([t])
                self.pub_tf.publish(tfm)

    def local_position_callback(self, msg):
        if self.start_pose is None:
            self.start_pose = msg.pose.pose
            rospy.loginfo("Start pose set: {}".format(self.start_pose))


if __name__ == '__main__':
    rospy.init_node('fixed_start_pose_tf_broadcaster')
    tfb = FixedStartPoseBroadcaster()
    rospy.spin()
