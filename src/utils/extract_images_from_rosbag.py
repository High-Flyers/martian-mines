# images after extraction can be converted to video using ffmpeg: ffmpeg -framerate 30 -i frame_%06d.png -codec copy output.mp4

import argparse
import rospy
import rosbag
import cv2
from cv_bridge import CvBridge
from cv_bridge import CvBridgeError


class ImageFromBag:

    def __init__(self, bag_path, output_path):
        self.bridge = CvBridge()

        with rosbag.Bag(bag_path) as bag:
            for idx, (topic, msg, t) in enumerate(bag.read_messages(topics=['/uav0/color/image_raw'])):
                try:
                    cv_image = self.bridge.imgmsg_to_cv2(msg)
                    image_name = f'frame_{idx:06d}.png'
                    cv2.imwrite(output_path + image_name, cv_image)
                    print(f"Wrote image: {image_name}")
                except CvBridgeError as e:
                    print(e)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Extract images from a ROS bag file.')
    parser.add_argument('bag_path', type=str, help='Path to the ROS bag file')
    parser.add_argument('output_path', type=str, help='Path to the output directory')
    args = parser.parse_args()

    try:
        image_from_bag = ImageFromBag(args.bag_path, args.output_path)
    except rospy.ROSInterruptException:
        pass
